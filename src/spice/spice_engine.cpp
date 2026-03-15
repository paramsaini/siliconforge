// SPICE Simulation Engine — MNA solver implementation
#include "spice/spice_engine.hpp"

#include <cmath>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <stdexcept>
#include <cstdio>
#include <set>

namespace sf {

// ════════════════════════════════════════════════════════════════════════
// Helpers
// ════════════════════════════════════════════════════════════════════════

static std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return s;
}

// ════════════════════════════════════════════════════════════════════════
// Load circuit
// ════════════════════════════════════════════════════════════════════════

void SpiceEngine::load(const SpiceCircuit& circuit) {
    circuit_ = circuit;
    build_node_map();
}

void SpiceEngine::add_model(const std::string& name, const MosfetModel& model) {
    models_[name] = model;

    // Re-resolve any MOSFET devices that reference this model
    // (handles the common case where add_model() is called after load())
    for (auto& rd : resolved_devices_) {
        if (rd.dev.type == SpiceDeviceType::MOSFET &&
            rd.dev.model_name == name && !rd.has_model) {
            rd.mos_model = model;
            rd.has_model = true;
            // Apply per-instance W/L overrides
            if (rd.dev.params.count("w"))
                rd.mos_model.W = rd.dev.params.at("w");
            if (rd.dev.params.count("l"))
                rd.mos_model.L = rd.dev.params.at("l");
        }
    }
}

void SpiceEngine::add_pwl(const std::string& source_name, const PWLStimulus& stim) {
    pwl_stimuli_[source_name] = stim;
}

// ════════════════════════════════════════════════════════════════════════
// Node map construction
// ════════════════════════════════════════════════════════════════════════

void SpiceEngine::build_node_map() {
    node_map_.clear();
    resolved_devices_.clear();
    vsrc_names_.clear();
    num_nodes_ = 0;
    num_vsrc_  = 0;

    // Ground nodes — index 0 (excluded from matrix)
    node_map_["0"]   = 0;
    node_map_["gnd"] = 0;
    node_map_["GND"] = 0;

    // Collect all devices (top-level + flattened subcircuits)
    std::vector<SpiceDevice> all_devices;

    // Top-level instances
    for (auto& dev : circuit_.instances) {
        all_devices.push_back(dev);
    }

    // Flatten subcircuit instances (single level for now)
    for (auto& sub : circuit_.subcircuits) {
        for (auto& dev : sub.devices) {
            all_devices.push_back(dev);
        }
    }

    // Assign node indices
    auto ensure_node = [&](const std::string& name) {
        if (name.empty()) return;
        std::string n = name;
        if (node_map_.find(n) == node_map_.end()) {
            // Check if it's a ground alias
            std::string nl = lower(n);
            if (nl == "0" || nl == "gnd" || nl == "vss") {
                node_map_[n] = 0;
            } else {
                num_nodes_++;
                node_map_[n] = num_nodes_;
            }
        }
    };

    for (auto& dev : all_devices) {
        for (auto& t : dev.terminals) {
            ensure_node(t);
        }
    }

    // Resolve devices — attach model info
    for (auto& dev : all_devices) {
        ResolvedDevice rd;
        rd.dev = dev;
        rd.has_model = false;

        if (dev.type == SpiceDeviceType::MOSFET) {
            // Look up model
            if (models_.count(dev.model_name)) {
                rd.mos_model = models_[dev.model_name];
                rd.has_model = true;
            } else if (circuit_.models.count(dev.model_name)) {
                // Build MosfetModel from parsed SpiceModel
                auto& sm = circuit_.models[dev.model_name];
                MosfetModel mm;
                mm.name = sm.name;
                mm.level = sm.level;
                mm.is_pmos = (lower(sm.type) == "pmos");
                auto get = [&](const std::string& key, double def) -> double {
                    std::string lk = lower(key);
                    if (sm.params.count(lk)) return sm.params.at(lk);
                    return def;
                };
                mm.VTH0 = get("vth0", mm.VTH0);
                mm.K1   = get("k1", mm.K1);
                mm.K2   = get("k2", mm.K2);
                mm.U0   = get("u0", mm.U0);
                mm.TOXE = get("toxe", mm.TOXE);
                mm.VSAT = get("vsat", mm.VSAT);
                mm.ETA0 = get("eta0", mm.ETA0);
                mm.PCLM = get("pclm", mm.PCLM);
                rd.mos_model = mm;
                rd.has_model = true;
            }

            // Apply instance-level W/L overrides
            if (dev.params.count("w"))
                rd.mos_model.W = dev.params.at("w");
            if (dev.params.count("l"))
                rd.mos_model.L = dev.params.at("l");
        }

        if (dev.type == SpiceDeviceType::VSOURCE) {
            vsrc_names_.push_back(dev.name);
            num_vsrc_++;
        }

        resolved_devices_.push_back(std::move(rd));
    }

    mna_size_ = num_nodes_ + num_vsrc_;
}

int SpiceEngine::node_idx(const std::string& name) const {
    auto it = node_map_.find(name);
    if (it != node_map_.end()) return it->second;
    // Try lowercase
    std::string nl = lower(name);
    if (nl == "0" || nl == "gnd" || nl == "vss") return 0;
    return -1;
}

// ════════════════════════════════════════════════════════════════════════
// MNA stamping
// ════════════════════════════════════════════════════════════════════════

void SpiceEngine::stamp_mna(std::vector<std::vector<double>>& G,
                            std::vector<double>& I,
                            const std::vector<double>& V,
                            double source_factor,
                            double time,
                            double dt,
                            const std::vector<double>* V_prev) {
    int N = mna_size_;

    // Zero out
    for (int i = 0; i < N; ++i) {
        I[i] = 0.0;
        for (int j = 0; j < N; ++j)
            G[i][j] = 0.0;
    }

    int vsrc_idx = 0;

    for (auto& rd : resolved_devices_) {
        auto& dev = rd.dev;

        switch (dev.type) {
        case SpiceDeviceType::RESISTOR: {
            double R = 1e6;  // default 1M
            if (dev.params.count("r")) R = dev.params.at("r");
            if (R <= 0) R = 1e-3;
            double g = 1.0 / R;

            int n1 = node_idx(dev.terminals[0]);
            int n2 = node_idx(dev.terminals[1]);

            // Stamp conductance
            auto stamp_g = [&](int a, int b, double val) {
                if (a > 0 && b > 0) G[a-1][b-1] += val;
            };
            // G matrix: g added to (n1,n1), (n2,n2), subtracted from (n1,n2), (n2,n1)
            if (n1 > 0) G[n1-1][n1-1] += g;
            if (n2 > 0) G[n2-1][n2-1] += g;
            if (n1 > 0 && n2 > 0) {
                G[n1-1][n2-1] -= g;
                G[n2-1][n1-1] -= g;
            }
            break;
        }

        case SpiceDeviceType::CAPACITOR: {
            if (dt <= 0) break;  // DC: capacitor is open circuit

            double C = 1e-12;  // default 1pF
            if (dev.params.count("c")) C = dev.params.at("c");

            int n1 = node_idx(dev.terminals[0]);
            int n2 = node_idx(dev.terminals[1]);

            // Backward Euler companion: G_eq = C/dt, I_eq = C/dt * V_prev
            double g_eq = C / dt;

            if (n1 > 0) G[n1-1][n1-1] += g_eq;
            if (n2 > 0) G[n2-1][n2-1] += g_eq;
            if (n1 > 0 && n2 > 0) {
                G[n1-1][n2-1] -= g_eq;
                G[n2-1][n1-1] -= g_eq;
            }

            // History current: I_eq = g_eq * V_prev_diff
            if (V_prev) {
                double v1_prev = (n1 > 0) ? (*V_prev)[n1-1] : 0.0;
                double v2_prev = (n2 > 0) ? (*V_prev)[n2-1] : 0.0;
                double i_hist = g_eq * (v1_prev - v2_prev);
                if (n1 > 0) I[n1-1] += i_hist;
                if (n2 > 0) I[n2-1] -= i_hist;
            }
            break;
        }

        case SpiceDeviceType::VSOURCE: {
            int n_plus  = node_idx(dev.terminals[0]);
            int n_minus = node_idx(dev.terminals[1]);
            int row = num_nodes_ + vsrc_idx;  // extra MNA row (0-based)

            double V_val = 0.0;
            if (dev.params.count("dc")) V_val = dev.params.at("dc");

            // Check for PWL stimulus
            if (time >= 0.0 && pwl_stimuli_.count(dev.name)) {
                V_val = pwl_stimuli_[dev.name].eval(time);
            } else if (time >= 0.0 && dev.params.count("pwl")) {
                // Reconstruct PWL from parsed params
                int n_pts = static_cast<int>(dev.params.count("pwl_n") ?
                            dev.params.at("pwl_n") : 0);
                if (n_pts > 0) {
                    PWLStimulus stim;
                    for (int k = 0; k < n_pts; ++k) {
                        stim.times.push_back(dev.params.at("pwl_t" + std::to_string(k)));
                        stim.values.push_back(dev.params.at("pwl_v" + std::to_string(k)));
                    }
                    V_val = stim.eval(time);
                }
            }

            V_val *= source_factor;

            // Stamp voltage source:
            // V(n+) - V(n-) = V_val
            // Extra variable: current through source (i_vsrc)
            if (n_plus > 0) {
                G[n_plus-1][row] += 1.0;
                G[row][n_plus-1] += 1.0;
            }
            if (n_minus > 0) {
                G[n_minus-1][row] -= 1.0;
                G[row][n_minus-1] -= 1.0;
            }
            I[row] = V_val;

            vsrc_idx++;
            break;
        }

        case SpiceDeviceType::MOSFET: {
            if (!rd.has_model) break;
            if (dev.terminals.size() < 4) break;

            int nd = node_idx(dev.terminals[0]);  // drain
            int ng = node_idx(dev.terminals[1]);  // gate
            int ns = node_idx(dev.terminals[2]);  // source
            int nb = node_idx(dev.terminals[3]);  // bulk

            auto vnode = [&](int n) -> double {
                if (n <= 0) return 0.0;
                if (n-1 < static_cast<int>(V.size())) return V[n-1];
                return 0.0;
            };

            double Vg = vnode(ng);
            double Vd = vnode(nd);
            double Vs = vnode(ns);
            double Vb = vnode(nb);

            double Vgs = Vg - Vs;
            double Vds = Vd - Vs;
            double Vbs = Vb - Vs;

            auto& mm = rd.mos_model;
            double Ids = mm.evaluate_ids(Vgs, Vds, Vbs);
            double gm_val  = mm.gm(Vgs, Vds, Vbs);
            double gds_val = mm.gds(Vgs, Vds, Vbs);
            double gmb_val = mm.gmb(Vgs, Vds, Vbs);

            // Add minimum conductance for numerical stability
            double g_min = 1e-12;
            gds_val = std::max(gds_val, g_min);

            // Newton-Raphson linearized stamp:
            // I_d = Ids0 + gm*(Vgs - Vgs0) + gds*(Vds - Vds0) + gmb*(Vbs - Vbs0)
            // which simplifies to stamping:
            //   Current into drain node:   +Ids0 + gm*(Vgs-Vgs0) + gds*(Vds-Vds0) + gmb*(Vbs-Vbs0)
            //   Current out of source node: same magnitude

            // The companion current (evaluated at current operating point)
            double I_eq = Ids - gm_val * Vgs - gds_val * Vds - gmb_val * Vbs;

            // Stamp Jacobian entries (conductance contributions)
            auto stamp = [&](int r, int c, double val) {
                if (r > 0 && c > 0) G[r-1][c-1] += val;
            };

            // gm contributions: dIds/dVg to drain, -dIds/dVg to source
            stamp(nd, ng, gm_val);   stamp(nd, ns, -gm_val);
            stamp(ns, ng, -gm_val);  stamp(ns, ns, gm_val);

            // gds contributions: dIds/dVd to drain, etc.
            stamp(nd, nd, gds_val);  stamp(nd, ns, -gds_val);
            stamp(ns, nd, -gds_val); stamp(ns, ns, gds_val);

            // gmb contributions
            stamp(nd, nb, gmb_val);  stamp(nd, ns, -gmb_val);
            stamp(ns, nb, -gmb_val); stamp(ns, ns, gmb_val);

            // RHS current injection
            if (nd > 0) I[nd-1] -= I_eq;
            if (ns > 0) I[ns-1] += I_eq;

            // Add capacitor companions in transient mode
            if (dt > 0 && V_prev) {
                MosfetCaps caps = mm.compute_caps(Vgs, Vds, Vbs);

                // Cgs companion
                double g_cgs = caps.Cgs / dt;
                stamp(ng, ng, g_cgs);  stamp(ng, ns, -g_cgs);
                stamp(ns, ng, -g_cgs); stamp(ns, ns, g_cgs);

                double vgs_prev = 0.0;
                if (ng > 0 && ns > 0) vgs_prev = (*V_prev)[ng-1] - (*V_prev)[ns-1];
                else if (ng > 0) vgs_prev = (*V_prev)[ng-1];
                else if (ns > 0) vgs_prev = -(*V_prev)[ns-1];

                double i_cgs = g_cgs * vgs_prev;
                if (ng > 0) I[ng-1] += i_cgs;
                if (ns > 0) I[ns-1] -= i_cgs;

                // Cgd companion
                double g_cgd = caps.Cgd / dt;
                stamp(ng, ng, g_cgd);  stamp(ng, nd, -g_cgd);
                stamp(nd, ng, -g_cgd); stamp(nd, nd, g_cgd);

                double vgd_prev = 0.0;
                if (ng > 0 && nd > 0) vgd_prev = (*V_prev)[ng-1] - (*V_prev)[nd-1];
                else if (ng > 0) vgd_prev = (*V_prev)[ng-1];
                else if (nd > 0) vgd_prev = -(*V_prev)[nd-1];

                double i_cgd = g_cgd * vgd_prev;
                if (ng > 0) I[ng-1] += i_cgd;
                if (nd > 0) I[nd-1] -= i_cgd;
            }

            break;
        }

        case SpiceDeviceType::ISOURCE: {
            int n_plus  = node_idx(dev.terminals[0]);
            int n_minus = node_idx(dev.terminals[1]);
            double I_val = 0.0;
            if (dev.params.count("dc")) I_val = dev.params.at("dc");
            I_val *= source_factor;

            // Current flows from n+ to n-
            if (n_plus > 0)  I[n_plus-1]  -= I_val;
            if (n_minus > 0) I[n_minus-1] += I_val;
            break;
        }

        default:
            break;
        }
    }

    // Add GMIN to diagonal for convergence
    double gmin = 1e-12;
    for (int i = 0; i < num_nodes_; ++i) {
        G[i][i] += gmin;
    }
}

// ════════════════════════════════════════════════════════════════════════
// Dense LU solver with partial pivoting
// ════════════════════════════════════════════════════════════════════════

bool SpiceEngine::lu_solve(std::vector<std::vector<double>>& A,
                           std::vector<double>& b,
                           std::vector<double>& x) {
    int n = static_cast<int>(b.size());
    x.resize(n, 0.0);

    // Forward elimination with partial pivoting
    std::vector<int> piv(n);
    std::iota(piv.begin(), piv.end(), 0);

    for (int k = 0; k < n; ++k) {
        // Find pivot
        double max_val = 0.0;
        int max_row = k;
        for (int i = k; i < n; ++i) {
            double av = std::abs(A[i][k]);
            if (av > max_val) { max_val = av; max_row = i; }
        }

        if (max_val < 1e-30) {
            // Singular — add small perturbation
            A[k][k] = 1e-15;
        }

        if (max_row != k) {
            std::swap(A[k], A[max_row]);
            std::swap(b[k], b[max_row]);
            std::swap(piv[k], piv[max_row]);
        }

        // Eliminate below
        for (int i = k + 1; i < n; ++i) {
            double factor = A[i][k] / A[k][k];
            A[i][k] = factor;
            for (int j = k + 1; j < n; ++j) {
                A[i][j] -= factor * A[k][j];
            }
            b[i] -= factor * b[k];
        }
    }

    // Back substitution
    for (int i = n - 1; i >= 0; --i) {
        x[i] = b[i];
        for (int j = i + 1; j < n; ++j) {
            x[i] -= A[i][j] * x[j];
        }
        x[i] /= A[i][i];
    }

    return true;
}

// ════════════════════════════════════════════════════════════════════════
// DC Operating Point — Newton-Raphson with source stepping
// ════════════════════════════════════════════════════════════════════════

std::map<std::string, double> SpiceEngine::dc_operating_point() {
    int N = mna_size_;
    if (N <= 0) return {};

    std::vector<double> V(N, 0.0);

    // Source stepping ramp: start with a fraction, ramp to 1.0
    std::vector<double> steps;
    if (source_stepping) {
        steps = {0.1, 0.2, 0.4, 0.6, 0.8, 1.0};
    } else {
        steps = {1.0};
    }

    for (double sf : steps) {
        for (int iter = 0; iter < max_iterations; ++iter) {
            std::vector<std::vector<double>> G(N, std::vector<double>(N, 0.0));
            std::vector<double> I(N, 0.0);

            stamp_mna(G, I, V, sf);

            // RHS = I - G*V
            std::vector<double> rhs(N, 0.0);
            for (int i = 0; i < N; ++i) {
                rhs[i] = I[i];
                for (int j = 0; j < N; ++j) {
                    rhs[i] -= G[i][j] * V[j];
                }
            }

            // Check convergence
            double max_err = 0.0;
            for (int i = 0; i < N; ++i)
                max_err = std::max(max_err, std::abs(rhs[i]));

            if (max_err < convergence_tol) break;

            // Solve G * dV = rhs
            std::vector<double> dV;
            auto G_copy = G;
            auto rhs_copy = rhs;
            lu_solve(G_copy, rhs_copy, dV);

            // Update with damping
            double alpha = 1.0;
            // Limit voltage step to prevent divergence
            double max_dv = 0.0;
            for (auto& d : dV) max_dv = std::max(max_dv, std::abs(d));
            if (max_dv > 0.5) alpha = 0.5 / max_dv;

            for (int i = 0; i < N; ++i) {
                V[i] += alpha * dV[i];
            }
        }
    }

    // Build result map
    std::map<std::string, double> result;
    for (auto& [name, idx] : node_map_) {
        if (idx == 0) {
            result[name] = 0.0;
        } else if (idx - 1 < static_cast<int>(V.size())) {
            result[name] = V[idx - 1];
        }
    }

    return result;
}

// ════════════════════════════════════════════════════════════════════════
// Transient Analysis — Backward Euler
// ════════════════════════════════════════════════════════════════════════

TransientResult SpiceEngine::transient(double tstep, double tstop) {
    TransientResult result;
    int N = mna_size_;
    if (N <= 0) return result;

    // DC initial condition
    auto dc = dc_operating_point();
    std::vector<double> V(N, 0.0);
    for (auto& [name, idx] : node_map_) {
        if (idx > 0 && idx - 1 < N) {
            if (dc.count(name)) V[idx-1] = dc[name];
        }
    }

    // Record initial point
    result.time_points.push_back(0.0);
    for (auto& [name, idx] : node_map_) {
        if (idx > 0) {
            result.node_voltages[name].push_back(V[idx-1]);
        }
    }

    std::vector<double> V_prev = V;

    for (double t = tstep; t <= tstop + tstep * 0.5; t += tstep) {
        V_prev = V;

        // Newton-Raphson at each time step
        for (int iter = 0; iter < 50; ++iter) {
            std::vector<std::vector<double>> G(N, std::vector<double>(N, 0.0));
            std::vector<double> I(N, 0.0);

            stamp_mna(G, I, V, 1.0, t, tstep, &V_prev);

            // RHS = I - G*V
            std::vector<double> rhs(N, 0.0);
            for (int i = 0; i < N; ++i) {
                rhs[i] = I[i];
                for (int j = 0; j < N; ++j)
                    rhs[i] -= G[i][j] * V[j];
            }

            double max_err = 0.0;
            for (int i = 0; i < N; ++i)
                max_err = std::max(max_err, std::abs(rhs[i]));

            if (max_err < convergence_tol) break;

            std::vector<double> dV;
            auto G_copy = G;
            auto rhs_copy = rhs;
            lu_solve(G_copy, rhs_copy, dV);

            double alpha = 1.0;
            double max_dv = 0.0;
            for (auto& d : dV) max_dv = std::max(max_dv, std::abs(d));
            if (max_dv > 0.3) alpha = 0.3 / max_dv;

            for (int i = 0; i < N; ++i)
                V[i] += alpha * dV[i];
        }

        // Record
        result.time_points.push_back(t);
        for (auto& [name, idx] : node_map_) {
            if (idx > 0) {
                result.node_voltages[name].push_back(V[idx-1]);
            }
        }
    }

    return result;
}

// ════════════════════════════════════════════════════════════════════════
// Measurement: threshold crossing time
// ════════════════════════════════════════════════════════════════════════

double SpiceEngine::measure_delay(const std::vector<double>& time,
                                  const std::vector<double>& waveform,
                                  double threshold,
                                  bool rising) {
    if (time.size() != waveform.size() || time.size() < 2) return -1.0;

    for (size_t i = 1; i < waveform.size(); ++i) {
        bool crossed = false;
        if (rising)  crossed = (waveform[i-1] < threshold && waveform[i] >= threshold);
        else         crossed = (waveform[i-1] > threshold && waveform[i] <= threshold);

        if (crossed) {
            // Linear interpolation for exact crossing
            double frac = (threshold - waveform[i-1]) / (waveform[i] - waveform[i-1]);
            return time[i-1] + frac * (time[i] - time[i-1]);
        }
    }
    return -1.0;
}

// ════════════════════════════════════════════════════════════════════════
// Measurement: slew rate (rise/fall time between percentages)
// ════════════════════════════════════════════════════════════════════════

double SpiceEngine::measure_slew(const std::vector<double>& time,
                                 const std::vector<double>& waveform,
                                 double low_pct, double high_pct,
                                 bool rising) {
    if (waveform.empty()) return -1.0;

    double v_min = *std::min_element(waveform.begin(), waveform.end());
    double v_max = *std::max_element(waveform.begin(), waveform.end());
    double v_range = v_max - v_min;
    if (v_range < 1e-12) return -1.0;

    double v_low  = v_min + low_pct * v_range;
    double v_high = v_min + high_pct * v_range;

    double t_low  = measure_delay(time, waveform, v_low, rising);
    double t_high = measure_delay(time, waveform, v_high, rising);

    if (t_low < 0.0 || t_high < 0.0) return -1.0;
    return std::abs(t_high - t_low);
}

// ════════════════════════════════════════════════════════════════════════
// Measurement: average power over a period
// ════════════════════════════════════════════════════════════════════════

double SpiceEngine::measure_power(const std::vector<double>& v_wave,
                                  const std::vector<double>& i_wave,
                                  const std::vector<double>& time,
                                  double period) {
    if (v_wave.size() != i_wave.size() || v_wave.size() != time.size())
        return 0.0;
    if (time.size() < 2) return 0.0;

    double t_start = time.back() - period;
    if (t_start < time.front()) t_start = time.front();

    double energy = 0.0;
    double t_span = 0.0;

    for (size_t i = 1; i < time.size(); ++i) {
        if (time[i] < t_start) continue;
        double dt = time[i] - time[i-1];
        double p = 0.5 * (v_wave[i]*i_wave[i] + v_wave[i-1]*i_wave[i-1]);
        energy += p * dt;
        t_span += dt;
    }

    return (t_span > 0) ? energy / t_span : 0.0;
}

} // namespace sf
