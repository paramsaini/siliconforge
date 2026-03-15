// SiliconForge — PDN Analyzer Industrial Implementation
// Frequency-domain impedance, resonance detection, decap optimization.
#include "timing/pdn.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

namespace sf {

// ── Auto Config ──────────────────────────────────────────────────────────

void PdnAnalyzer::auto_config(double vdd, double current_ma) {
    cfg_.vdd = vdd;
    cfg_.total_current_ma = current_ma;
    cfg_.max_transient_current_ma = current_ma * 3;
    cfg_.pad_count = 4;
    cfg_.stripes.clear();

    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    int h_stripes = std::max(2, (int)(die_h / 20.0));
    int v_stripes = std::max(2, (int)(die_w / 20.0));

    for (int i = 0; i < h_stripes; ++i) {
        double offset = pd_.die_area.y0 + (i + 0.5) * die_h / h_stripes;
        cfg_.stripes.push_back({PdnStripe::HORIZONTAL, offset, 1.0, 3, 0.05});
    }
    for (int i = 0; i < v_stripes; ++i) {
        double offset = pd_.die_area.x0 + (i + 0.5) * die_w / v_stripes;
        cfg_.stripes.push_back({PdnStripe::VERTICAL, offset, 1.0, 4, 0.05});
    }
}

double PdnAnalyzer::nearest_stripe_resistance(double x, double y) const {
    double min_dist = 1e18;
    double min_res = 1.0;
    for (auto& s : cfg_.stripes) {
        double dist = (s.direction == PdnStripe::HORIZONTAL) ?
            std::abs(y - s.offset) : std::abs(x - s.offset);
        if (dist < min_dist) {
            min_dist = dist;
            min_res = dist * s.resistance_per_um + cfg_.pad_resistance;
        }
    }
    return min_res;
}

// ── Target Impedance ─────────────────────────────────────────────────────
// Z_target = VDD × ripple% / I_transient_peak
// This is the maximum PDN impedance allowed at any frequency.

double PdnAnalyzer::compute_target_impedance() const {
    if (cfg_.target_impedance_mohm > 0) return cfg_.target_impedance_mohm;
    double i_trans_a = cfg_.max_transient_current_ma * 1e-3;
    if (i_trans_a <= 0) i_trans_a = cfg_.total_current_ma * 3e-3;
    double ripple_v = cfg_.vdd * cfg_.voltage_ripple_pct / 100.0;
    if (i_trans_a <= 0) return 100.0; // default 100 mΩ
    return (ripple_v / i_trans_a) * 1000.0; // mΩ
}

// ── Single Decap Impedance ───────────────────────────────────────────────
// Z_decap = ESR + j(ωL - 1/(ωC))

std::complex<double> PdnAnalyzer::decap_impedance(const DecapModel& d, double freq_hz) const {
    double omega = 2.0 * M_PI * freq_hz;
    double r = d.esr_mohm * 1e-3;     // Ohm
    double l = d.esl_ph * 1e-12;      // H
    double c = d.capacitance_nf * 1e-9; // F

    double x_l = omega * l;
    double x_c = (omega * c > 0) ? 1.0 / (omega * c) : 1e12;
    return std::complex<double>(r, x_l - x_c);
}

// ── PDN Mesh Impedance ───────────────────────────────────────────────────
// Simple RL model: R_mesh from stripes, L from loop inductance

std::complex<double> PdnAnalyzer::mesh_impedance(double freq_hz) const {
    double omega = 2.0 * M_PI * freq_hz;

    // Aggregate mesh resistance from stripes
    double r_mesh = 0;
    if (!cfg_.stripes.empty()) {
        double total_conductance = 0;
        for (auto& s : cfg_.stripes) {
            double die_dim = (s.direction == PdnStripe::HORIZONTAL) ?
                pd_.die_area.width() : pd_.die_area.height();
            double stripe_r = s.resistance_per_um * die_dim / s.width;
            if (stripe_r > 0) total_conductance += 1.0 / stripe_r;
        }
        r_mesh = (total_conductance > 0) ? 1.0 / total_conductance : 0.1;
    } else {
        r_mesh = 0.1; // default
    }

    // Bump inductance contribution (parallel bumps)
    double l_bump = 0;
    int power_bumps = 0;
    for (auto& b : cfg_.bumps) {
        if (b.type == BumpPad::POWER || b.type == BumpPad::GROUND) {
            power_bumps++;
        }
    }
    if (power_bumps > 0) {
        double avg_l = cfg_.default_bump_l_ph;
        l_bump = avg_l * 1e-12 / power_bumps; // parallel inductance
    } else {
        l_bump = cfg_.default_bump_l_ph * 1e-12;
    }

    // Via array contribution (series R and L)
    double r_via = 0, l_via = 0;
    for (auto& va : cfg_.via_arrays) {
        r_via += va.total_resistance_mohm() * 1e-3;
        l_via += va.total_inductance_ph() * 1e-12;
    }

    double total_r = r_mesh + r_via;
    double total_l = l_bump + l_via;

    return std::complex<double>(total_r, omega * total_l);
}

// ── Full PDN Impedance ───────────────────────────────────────────────────
// Parallel combination: Z_pdn = mesh || decap1 || decap2 || ... || die_cap

std::complex<double> PdnAnalyzer::pdn_impedance_at(double freq_hz) const {
    // Start with mesh impedance
    auto z_mesh = mesh_impedance(freq_hz);

    // Total admittance = 1/Z_mesh + sum(1/Z_decap) + 1/Z_die
    std::complex<double> y_total = 1.0 / z_mesh;

    // Add decap contributions
    for (auto& d : cfg_.decaps) {
        if (d.quantity <= 0) continue;
        auto z_single = decap_impedance(d, freq_hz);
        if (std::abs(z_single) > 0)
            y_total += (double)d.quantity / z_single;
    }

    // On-die capacitance (pure cap, low ESR)
    double omega = 2.0 * M_PI * freq_hz;
    double c_die = cfg_.on_die_cap_nf * 1e-9;
    if (omega * c_die > 0) {
        std::complex<double> z_die(0.001, -1.0 / (omega * c_die)); // 1mΩ ESR
        y_total += 1.0 / z_die;
    }

    if (std::abs(y_total) > 0)
        return 1.0 / y_total;
    return std::complex<double>(0.1, 0);
}

// ── Impedance Sweep ──────────────────────────────────────────────────────

std::vector<ImpedancePoint> PdnAnalyzer::impedance_sweep() const {
    std::vector<ImpedancePoint> profile;
    double target = compute_target_impedance();

    double log_start = std::log10(cfg_.freq_start_mhz);
    double log_stop = std::log10(cfg_.freq_stop_mhz);
    int npts = std::max(10, cfg_.freq_points);

    for (int i = 0; i < npts; ++i) {
        double log_f = log_start + (log_stop - log_start) * i / (npts - 1);
        double f_mhz = std::pow(10.0, log_f);
        double f_hz = f_mhz * 1e6;

        auto z = pdn_impedance_at(f_hz);
        double mag = std::abs(z) * 1000.0; // Ohm → mΩ
        double phase = std::atan2(z.imag(), z.real()) * 180.0 / M_PI;

        ImpedancePoint pt;
        pt.freq_mhz = f_mhz;
        pt.magnitude_mohm = mag;
        pt.phase_deg = phase;
        pt.z = z;
        pt.exceeds_target = (mag > target);
        profile.push_back(pt);
    }
    return profile;
}

// ── Resonance Detection ──────────────────────────────────────────────────

std::vector<Resonance> PdnAnalyzer::find_resonances(
    const std::vector<ImpedancePoint>& profile) const {
    std::vector<Resonance> res;
    if (profile.size() < 3) return res;

    for (size_t i = 1; i + 1 < profile.size(); ++i) {
        double z_prev = profile[i-1].magnitude_mohm;
        double z_curr = profile[i].magnitude_mohm;
        double z_next = profile[i+1].magnitude_mohm;

        // Local minimum → series resonance (low impedance)
        if (z_curr < z_prev && z_curr < z_next) {
            double q = (z_prev + z_next) / (2.0 * z_curr);
            res.push_back({profile[i].freq_mhz, z_curr, q, Resonance::SERIES});
        }
        // Local maximum → parallel resonance (anti-resonance, high impedance)
        if (z_curr > z_prev && z_curr > z_next) {
            double q = z_curr / ((z_prev + z_next) / 2.0);
            res.push_back({profile[i].freq_mhz, z_curr, q, Resonance::PARALLEL});
        }
    }
    return res;
}

// ── Spatial Voltage Analysis ─────────────────────────────────────────────

void PdnAnalyzer::spatial_analysis(int grid_res, PdnResult& r) {
    if (cfg_.stripes.empty()) auto_config();

    double cell_w = pd_.die_area.width() / grid_res;
    double cell_h = pd_.die_area.height() / grid_res;

    std::vector<std::vector<double>> current_map(grid_res, std::vector<double>(grid_res, 0));
    double total_cell_area = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        total_cell_area += c.width * c.height;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, grid_res-1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, grid_res-1);
        current_map[gy][gx] += c.width * c.height;
    }
    if (total_cell_area > 0) {
        for (auto& row : current_map)
            for (auto& val : row)
                val = (val / total_cell_area) * cfg_.total_current_ma;
    }

    double worst_drop = 0, total_drop = 0;
    int total_nodes = 0;

    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            double px = pd_.die_area.x0 + (x + 0.5) * cell_w;
            double py = pd_.die_area.y0 + (y + 0.5) * cell_h;
            double r_path = nearest_stripe_resistance(px, py);
            double i_local = current_map[y][x];
            double drop_v = i_local * r_path * 0.001;
            double drop_mv = drop_v * 1000;
            double voltage = cfg_.vdd * 1000 - drop_mv;

            r.nodes.push_back({px, py, voltage / 1000.0, i_local});
            worst_drop = std::max(worst_drop, drop_mv);
            total_drop += drop_mv;
            total_nodes++;

            // EM check
            for (auto& s : cfg_.stripes) {
                double dist = (s.direction == PdnStripe::HORIZONTAL) ?
                    std::abs(py - s.offset) : std::abs(px - s.offset);
                if (dist < cell_w) {
                    double cd = i_local / s.width;
                    r.worst_current_density = std::max(r.worst_current_density, cd);
                    if (cd > cfg_.em_limit_ma_per_um) r.em_violations++;
                }
            }
        }
    }

    r.worst_drop_mv = worst_drop;
    r.worst_drop_pct = (cfg_.vdd > 0) ? worst_drop / (cfg_.vdd * 10) : 0;
    r.avg_drop_mv = total_nodes > 0 ? total_drop / total_nodes : 0;
}

// ── Full Analysis ────────────────────────────────────────────────────────

PdnResult PdnAnalyzer::analyze(int grid_res) {
    auto t0 = std::chrono::high_resolution_clock::now();
    PdnResult r;

    // Spatial voltage analysis
    spatial_analysis(grid_res, r);

    // Target impedance
    r.target_impedance_mohm = compute_target_impedance();

    // Impedance sweep
    r.impedance_profile = impedance_sweep();
    for (auto& pt : r.impedance_profile) {
        if (pt.magnitude_mohm > r.worst_impedance_mohm) {
            r.worst_impedance_mohm = pt.magnitude_mohm;
            r.worst_impedance_freq_mhz = pt.freq_mhz;
        }
        if (pt.exceeds_target) r.target_violations++;
    }

    // Resonance detection
    r.resonances = find_resonances(r.impedance_profile);
    r.num_resonances = (int)r.resonances.size();

    // Decap summary
    for (auto& d : cfg_.decaps) {
        if (d.quantity > 0) {
            r.total_decap_nf += d.capacitance_nf * d.quantity;
            r.total_decap_area_um2 += d.area_um2 * d.quantity;
            r.total_decap_leakage_ua += d.leakage_ua * d.quantity;
            r.decap_types_used++;
        }
    }

    // Via array summary
    for (auto& va : cfg_.via_arrays) {
        r.total_via_resistance_mohm += va.total_resistance_mohm();
    }

    // Bump summary
    for (auto& b : cfg_.bumps) {
        if (b.type == BumpPad::POWER) r.power_bumps++;
        else if (b.type == BumpPad::GROUND) r.ground_bumps++;
    }
    r.bump_inductance_ph = cfg_.default_bump_l_ph;

    // Power integrity signoff
    bool drop_pass = r.worst_drop_pct < cfg_.voltage_ripple_pct;
    bool impedance_pass = r.target_violations == 0;
    r.pi_signoff_pass = drop_pass && impedance_pass;
    r.pi_summary = "Drop: " + std::to_string((int)r.worst_drop_pct) + "% (" +
        (drop_pass ? "PASS" : "FAIL") + "), Impedance violations: " +
        std::to_string(r.target_violations) + " (" +
        (impedance_pass ? "PASS" : "FAIL") + "), Resonances: " +
        std::to_string(r.num_resonances);

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "PDN: worst drop " + std::to_string((int)r.worst_drop_mv) + "mV (" +
                std::to_string((int)r.worst_drop_pct) + "%), " +
                std::to_string(r.em_violations) + " EM violations, " +
                std::to_string(r.num_resonances) + " resonances" +
                (r.pi_signoff_pass ? " [PI PASS]" : " [PI FAIL]");
    return r;
}

// ── Enhanced: Target Impedance Detail ────────────────────────────────

TargetImpedance PdnAnalyzer::compute_target_impedance_detail(
    double voltage, double max_current_a, double ripple_pct) {
    TargetImpedance ti;
    ti.voltage = voltage;
    ti.max_current = max_current_a;
    ti.ripple_pct = ripple_pct;
    ti.frequency_hz = 1e9;
    if (max_current_a > 0)
        ti.z_target_ohms = (voltage * ripple_pct / 100.0) / max_current_a;
    else
        ti.z_target_ohms = 0.1;
    double z_actual = std::abs(pdn_impedance_at(ti.frequency_hz));
    ti.meets_target = z_actual <= ti.z_target_ohms;
    return ti;
}

// ── Enhanced: Decoupling Optimization ────────────────────────────────

DecapOptResult PdnAnalyzer::optimize_decoupling(double budget_area) {
    DecapOptResult result{};

    // Run spatial analysis to find highest IR-drop locations
    PdnResult baseline;
    baseline.nodes.clear();
    spatial_analysis(10, baseline);

    // Sort nodes by voltage ascending (worst drop first)
    auto sorted = baseline.nodes;
    std::sort(sorted.begin(), sorted.end(),
              [](const PdnResult::PdnNode& a, const PdnResult::PdnNode& b) {
                  return a.voltage < b.voltage;
              });

    double z_before = std::abs(pdn_impedance_at(1e9)) * 1000.0;
    double area_used = 0;
    double cap_per_decap = 10.0;
    double area_per_decap = 100.0;
    int max_decaps = budget_area > 0 ? (int)(budget_area / area_per_decap) : (int)sorted.size();
    max_decaps = std::min(max_decaps, (int)sorted.size());

    for (int i = 0; i < max_decaps; ++i) {
        if (budget_area > 0 && area_used + area_per_decap > budget_area) break;
        result.locations.push_back({sorted[i].x, sorted[i].y});
        result.decaps_added++;
        result.total_capacitance_nf += cap_per_decap;
        area_used += area_per_decap;
        cfg_.on_die_cap_nf += cap_per_decap;
    }

    double z_after = std::abs(pdn_impedance_at(1e9)) * 1000.0;
    result.impedance_improvement_pct = z_before > 0 ?
        (z_before - z_after) / z_before * 100.0 : 0;
    return result;
}

// ── Enhanced: AC Impedance Profile ───────────────────────────────────

ACImpedanceProfile PdnAnalyzer::compute_ac_impedance(
    double f_start, double f_end, int points) {
    ACImpedanceProfile prof;
    prof.first_resonance_hz = 0;
    prof.anti_resonance_hz = 0;

    double log_s = std::log10(f_start);
    double log_e = std::log10(f_end);
    int npts = std::max(10, points);

    for (int i = 0; i < npts; ++i) {
        double log_f = log_s + (log_e - log_s) * i / (npts - 1);
        double f = std::pow(10.0, log_f);

        // Include package model contribution
        double omega = 2.0 * M_PI * f;
        double r_pkg = pkg_model_.pkg_resistance_mohm * 1e-3;
        double l_pkg = pkg_model_.pkg_inductance_nh * 1e-9;
        double c_pkg = pkg_model_.pkg_capacitance_pf * 1e-12;
        double l_brd = pkg_model_.board_inductance_nh * 1e-9;
        double c_brd = pkg_model_.board_capacitance_uf * 1e-6;

        auto z_pdn = pdn_impedance_at(f);

        // Package: R + jwL + 1/jwC (series RLC)
        std::complex<double> z_pkg(r_pkg,
            omega * l_pkg - (omega * c_pkg > 0 ? 1.0 / (omega * c_pkg) : 0));

        // Board decap contribution (parallel)
        std::complex<double> z_brd(0.001,
            omega * l_brd - (omega * c_brd > 0 ? 1.0 / (omega * c_brd) : 0));

        // Combine: (pdn || board) + package in series
        std::complex<double> y_par = 1.0 / z_pdn;
        if (std::abs(z_brd) > 0) y_par += 1.0 / z_brd;
        std::complex<double> z_total = (std::abs(y_par) > 0 ? 1.0 / y_par : z_pdn) + z_pkg;

        double mag = std::abs(z_total);
        double phase = std::atan2(z_total.imag(), z_total.real()) * 180.0 / M_PI;

        prof.frequencies.push_back(f);
        prof.impedance_mag.push_back(mag);
        prof.impedance_phase.push_back(phase);

        // Detect first resonance (local min) and anti-resonance (local max)
        if (i >= 2) {
            double m2 = prof.impedance_mag[i - 2];
            double m1 = prof.impedance_mag[i - 1];
            double m0 = mag;
            if (prof.first_resonance_hz == 0 && m1 < m2 && m1 < m0)
                prof.first_resonance_hz = prof.frequencies[i - 1];
            if (prof.anti_resonance_hz == 0 && m1 > m2 && m1 > m0)
                prof.anti_resonance_hz = prof.frequencies[i - 1];
        }
    }
    return prof;
}

// ── Enhanced: Set Package Model ──────────────────────────────────────

void PdnAnalyzer::set_package_model(const PdnPackageModel& pkg) {
    pkg_model_ = pkg;
}

// ── Enhanced: Full Enhanced PDN Run ──────────────────────────────────

PdnResult PdnAnalyzer::run_enhanced() {
    // 1. Static analysis (spatial IR drop + EM checks)
    PdnResult r = analyze(10);

    // 2. AC impedance profile (includes package model)
    auto ac = compute_ac_impedance();

    // 3. Target impedance check
    double i_max_a = cfg_.max_transient_current_ma * 1e-3;
    auto ti = compute_target_impedance_detail(cfg_.vdd, i_max_a, cfg_.voltage_ripple_pct);
    if (!ti.meets_target) {
        r.pi_signoff_pass = false;
        r.pi_summary += " | Target-Z FAIL (need " +
            std::to_string(ti.z_target_ohms * 1000) + "mOhm)";
    }

    // 4. Decap optimization if impedance fails
    if (!ti.meets_target) {
        auto decap = optimize_decoupling(0);
        r.total_decap_nf += decap.total_capacitance_nf;
    }

    // 5. EM-aware IR hotspot fixing: auto-fix if IR drop or EM violations are bad
    if (r.worst_drop_pct > cfg_.voltage_ripple_pct || r.em_violations > 0) {
        IrFixConfig fix_cfg;
        fix_cfg.target_drop_pct = cfg_.voltage_ripple_pct;
        fix_cfg.max_iterations = 5;
        fix_cfg.stripe_width = 2.0;
        fix_cfg.stripe_r_per_um = 0.01;
        auto fix = fix_ir_hotspots(fix_cfg);
        if (fix.converged) {
            // Re-run analysis with fixed stripes
            PdnResult r2 = analyze(10);
            r.worst_drop_mv = r2.worst_drop_mv;
            r.worst_drop_pct = r2.worst_drop_pct;
            r.avg_drop_mv = r2.avg_drop_mv;
            r.em_violations = r2.em_violations;
            r.worst_current_density = r2.worst_current_density;
            r.nodes = r2.nodes;
        }
        r.message += " | IR-fix: " + fix.message;
    }

    // 6. Final EM signoff check
    if (r.em_violations > 0) {
        r.pi_signoff_pass = false;
        r.pi_summary += " | EM FAIL (" + std::to_string(r.em_violations) + " violations, "
                       "worst J=" + std::to_string(r.worst_current_density) +
                       " mA/um, limit=" + std::to_string(cfg_.em_limit_ma_per_um) + ")";
    }

    r.message += " [enhanced]";
    return r;
}

// ── IR-Aware Stripe Insertion ────────────────────────────────────────

IrFixResult PdnAnalyzer::fix_ir_hotspots(IrFixConfig fix_cfg, int grid_res) {
    IrFixResult fix;

    // Initial analysis
    PdnResult r = analyze(grid_res);
    fix.initial_drop_pct = r.worst_drop_pct;

    if (r.worst_drop_pct <= fix_cfg.target_drop_pct) {
        fix.converged = true;
        fix.final_drop_pct = r.worst_drop_pct;
        fix.message = "IR drop already meets target"
                      " (EM limit=" + std::to_string(cfg_.em_limit_ma_per_um) + " mA/um)";
        return fix;
    }

    for (int iter = 0; iter < fix_cfg.max_iterations; ++iter) {
        fix.iterations = iter + 1;

        // Find hotspot nodes: nodes with voltage drop > threshold × worst_drop
        double drop_threshold = fix_cfg.hotspot_threshold * r.worst_drop_mv;
        double die_w = pd_.die_area.width();
        double die_h = pd_.die_area.height();

        // Collect hotspot y-coords for horizontal stripes, x-coords for vertical
        bool add_horizontal = (fix.iterations % 2 == 1);
        std::vector<double> hotspot_coords;
        std::vector<double> hotspot_currents;  // track current at each hotspot

        for (auto& node : r.nodes) {
            double node_drop = cfg_.vdd * 1000.0 - node.voltage * 1000.0;
            if (node_drop >= drop_threshold) {
                double coord = add_horizontal ? node.y : node.x;
                // Quantize to grid to avoid duplicate stripes
                double quantized = std::round(coord * 10.0) / 10.0;
                bool dup = false;
                for (size_t k = 0; k < hotspot_coords.size(); ++k) {
                    if (std::abs(hotspot_coords[k] - quantized) < fix_cfg.stripe_width * 2) {
                        // Accumulate current for EM sizing
                        hotspot_currents[k] = std::max(hotspot_currents[k], node.current_draw);
                        dup = true;
                        break;
                    }
                }
                if (!dup) {
                    hotspot_coords.push_back(quantized);
                    hotspot_currents.push_back(node.current_draw);
                }
            }
        }

        if (hotspot_coords.empty()) break;

        // Add stripes at hotspot locations with EM-aware width sizing
        for (size_t si = 0; si < hotspot_coords.size(); ++si) {
            double coord = hotspot_coords[si];
            double local_current = hotspot_currents[si];

            // EM-aware width: ensure current density stays below EM limit
            // J = I / width  =>  width_min = I / J_max
            double em_min_width = (cfg_.em_limit_ma_per_um > 0)
                ? local_current / cfg_.em_limit_ma_per_um : fix_cfg.stripe_width;
            // Use the larger of user-specified width and EM-required width,
            // with 20% margin for reliability
            double stripe_w = std::max(fix_cfg.stripe_width, em_min_width * 1.2);

            PdnStripe stripe;
            stripe.direction = add_horizontal ? PdnStripe::HORIZONTAL : PdnStripe::VERTICAL;
            stripe.offset = coord;
            stripe.width = stripe_w;
            stripe.layer = fix_cfg.stripe_layer;
            stripe.resistance_per_um = fix_cfg.stripe_r_per_um;
            cfg_.stripes.push_back(stripe);
            fix.stripes_added++;
        }

        // Re-analyze with added stripes
        r = analyze(grid_res);
        fix.final_drop_pct = r.worst_drop_pct;

        if (r.worst_drop_pct <= fix_cfg.target_drop_pct) {
            fix.converged = true;
            break;
        }
    }

    fix.message = "IR fix: " + std::to_string(fix.stripes_added) + " stripes added over " +
                  std::to_string(fix.iterations) + " iterations, drop " +
                  std::to_string(fix.initial_drop_pct) + "% -> " +
                  std::to_string(fix.final_drop_pct) + "%" +
                  " (EM limit=" + std::to_string(cfg_.em_limit_ma_per_um) + " mA/um)";
    if (fix.converged) fix.message += " (CONVERGED)";
    else fix.message += " (NOT CONVERGED)";
    return fix;
}

// ══════════════════════════════════════════════════════════════════════
// Via-to-Via Mutual Inductance (Neumann's Formula)
//
// For two parallel cylindrical conductors of length L separated by
// center-to-center distance d:
//
//   M = (mu_0 * L) / (2 * pi) * [ln(2L/d) - 1 + d/L + ...]
//
// where mu_0 = 4*pi*1e-7 H/m.  This is the first-order Neumann
// approximation valid when L >> d (typical for through-layer vias).
//
// For short vias where d ~ L, we use the more accurate form:
//   M = (mu_0 / 2pi) * [L*ln((L + sqrt(L^2 + d^2))/d) - sqrt(L^2 + d^2) + d]
//
// Self-inductance of a single via (for coupling coefficient):
//   L_self = (mu_0 * L) / (2 * pi) * [ln(2L/r) - 1 + r/(2L)]
//   where r = via radius
// ══════════════════════════════════════════════════════════════════════

std::vector<ViaCouplingResult> PdnAnalyzer::compute_via_coupling(
    double via_height_um, double max_distance_um) const {

    std::vector<ViaCouplingResult> results;

    // Physical constants
    constexpr double mu_0 = 4.0 * M_PI * 1e-7; // H/m
    // Convert dimensions to meters for the formula
    double L = via_height_um * 1e-6;            // via height in meters
    double r_via = 0.5e-6;                       // default via radius 0.5um

    // Self-inductance of a single via (for coupling coefficient k = M/sqrt(L1*L2))
    double ln_term_self = std::log(2.0 * L / r_via) - 1.0 + r_via / (2.0 * L);
    double L_self_H = (mu_0 / (2.0 * M_PI)) * L * ln_term_self;
    double L_self_ph = L_self_H * 1e12; // convert to picohenry

    // Iterate over all via pairs in the physical design
    const auto& vias = pd_.vias;
    for (size_t i = 0; i < vias.size(); ++i) {
        for (size_t j = i + 1; j < vias.size(); ++j) {
            // Only consider vias on overlapping layer ranges (parallel conductors)
            bool layer_overlap = (vias[i].upper_layer >= vias[j].lower_layer &&
                                  vias[i].lower_layer <= vias[j].upper_layer);
            if (!layer_overlap) continue;

            double dx = (vias[i].position.x - vias[j].position.x) * 1e-6; // meters
            double dy = (vias[i].position.y - vias[j].position.y) * 1e-6;
            double d = std::sqrt(dx * dx + dy * dy);

            // Skip if too far apart or coincident
            double d_um = d * 1e6;
            if (d_um > max_distance_um || d < 1e-12) continue;

            // Neumann mutual inductance for parallel conductors
            // M = (mu_0 / 2pi) * [L * ln((L + sqrt(L^2 + d^2)) / d) - sqrt(L^2 + d^2) + d]
            double L2d2 = L * L + d * d;
            double sqrt_L2d2 = std::sqrt(L2d2);
            double M_H = (mu_0 / (2.0 * M_PI)) *
                          (L * std::log((L + sqrt_L2d2) / d) - sqrt_L2d2 + d);
            double M_ph = M_H * 1e12; // picohenry

            // Coupling coefficient k = M / sqrt(L1 * L2)
            // Both vias assumed identical: k = M / L_self
            double k = (L_self_ph > 0) ? M_ph / L_self_ph : 0;
            k = std::min(k, 1.0); // clamp to physical range

            ViaCouplingResult vcr;
            vcr.via_pair = {static_cast<int>(i), static_cast<int>(j)};
            vcr.mutual_inductance_ph = M_ph;
            vcr.coupling_coefficient = k;
            vcr.distance_um = d_um;
            results.push_back(vcr);
        }
    }

    // Sort by coupling coefficient descending (strongest coupling first)
    std::sort(results.begin(), results.end(),
              [](const ViaCouplingResult& a, const ViaCouplingResult& b) {
                  return a.coupling_coefficient > b.coupling_coefficient;
              });

    return results;
}

// ══════════════════════════════════════════════════════════════════════
// Package-to-Die Interaction Model
//
// Lumped RLC model of the package interconnect:
//
//   BGA ball → package trace → bond wire/bump → die pad
//
// IR drop on VDD rail:
//   V_drop = I_total * (R_bond / N_power_pins)
//
// SSN (Simultaneous Switching Noise):
//   V_ssn = N_switching * L_eff * dI/dt
//   where L_eff = lead_inductance / N_ground_pins (parallel pins)
//   and dI/dt = I_per_pin / t_rise = I_per_pin * slew_rate / V_swing
//
// Package-die resonance:
//   f_res = 1 / (2*pi * sqrt(L_pkg * C_pkg))
//   This resonance can amplify supply noise if it coincides with
//   the clock frequency or its harmonics.
// ══════════════════════════════════════════════════════════════════════

PackageInteractionResult PdnAnalyzer::analyze_package_interaction(
    const PackageModel& pkg) const {

    PackageInteractionResult result;

    // Effective resistance: parallel bond wires across all power pins
    if (pkg.power_pins > 0) {
        result.effective_pkg_resistance_mohm =
            pkg.bond_wire_resistance_mohm / pkg.power_pins;
    }

    // Effective inductance: parallel leads across power pins
    if (pkg.power_pins > 0) {
        result.effective_pkg_inductance_nh =
            pkg.lead_inductance_nh / pkg.power_pins;
    }

    // VDD-to-VSS loop inductance: power path inductance + ground return inductance
    double l_vdd = (pkg.power_pins > 0) ? pkg.lead_inductance_nh / pkg.power_pins : pkg.lead_inductance_nh;
    double l_vss = (pkg.ground_pins > 0) ? pkg.lead_inductance_nh / pkg.ground_pins : pkg.lead_inductance_nh;
    result.total_loop_inductance_nh = l_vdd + l_vss;

    // Package-level IR drop
    // V_drop = I_total * R_eff
    double total_current_ma = cfg_.total_current_ma;
    result.pkg_ir_drop_mv = total_current_ma * result.effective_pkg_resistance_mohm * 1e-3;
    // Convert mohm * mA = uV, so * 1e-3 for mV:
    // Actually: I(A) * R(ohm) = V; I(mA) * R(mohm) = I*1e-3 * R*1e-3 = V*1e-6 = uV
    // So mV = I(mA) * R(mohm) * 1e-3
    result.pkg_ir_drop_mv = total_current_ma * result.effective_pkg_resistance_mohm * 1e-3;

    // SSN computation
    // Number of simultaneously switching outputs
    int n_switching = static_cast<int>(pkg.signal_pins * pkg.switching_fraction);
    if (n_switching < 1) n_switching = 1;

    // dI/dt per pin: approximate as I_per_pin / t_rise
    // t_rise = V_swing / slew_rate; for CMOS: V_swing ≈ VDD
    double t_rise_ns = (pkg.slew_rate_v_per_ns > 0) ? cfg_.vdd / pkg.slew_rate_v_per_ns : 1.0;
    double di_dt_per_pin = pkg.io_current_ma / t_rise_ns; // mA/ns = A/us

    // Ground bounce: V_gnd = N_switching * L_gnd * dI/dt
    // L in nH, dI/dt in mA/ns → V = L(nH) * dI/dt(mA/ns) = L*1e-9 * I*1e-3 / t*1e-9 = L*I/t * 1e-3 V
    // → mV = L(nH) * dI/dt(mA/ns)
    double l_ground_per_pin = (pkg.ground_pins > 0) ? pkg.lead_inductance_nh / pkg.ground_pins : pkg.lead_inductance_nh;
    result.pkg_ground_bounce_mv = n_switching * l_ground_per_pin * di_dt_per_pin;

    // Total SSN is the sum of VDD droop + ground bounce
    double l_power_per_pin = (pkg.power_pins > 0) ? pkg.lead_inductance_nh / pkg.power_pins : pkg.lead_inductance_nh;
    double vdd_droop_mv = n_switching * l_power_per_pin * di_dt_per_pin;
    result.ssn_mv = vdd_droop_mv + result.pkg_ground_bounce_mv;

    // Package-die resonance frequency
    // f_res = 1 / (2*pi * sqrt(L_loop * C_pkg))
    double l_loop_h = result.total_loop_inductance_nh * 1e-9;
    double c_pkg_f = pkg.package_cap_pf * 1e-12;
    if (l_loop_h > 0 && c_pkg_f > 0) {
        double f_res_hz = 1.0 / (2.0 * M_PI * std::sqrt(l_loop_h * c_pkg_f));
        result.resonance_freq_mhz = f_res_hz * 1e-6;
    }

    // Noise budget check: SSN should be < 10% of VDD for reliable operation
    double noise_budget_mv = cfg_.vdd * 1000.0 * 0.10;
    result.meets_noise_budget = result.ssn_mv < noise_budget_mv;

    // Summary
    result.summary = "Package IR drop: " + std::to_string(result.pkg_ir_drop_mv) + "mV, " +
                     "Ground bounce: " + std::to_string(result.pkg_ground_bounce_mv) + "mV, " +
                     "SSN: " + std::to_string(result.ssn_mv) + "mV (" +
                     (result.meets_noise_budget ? "PASS" : "FAIL") +
                     ", budget=" + std::to_string(noise_budget_mv) + "mV), " +
                     "Resonance: " + std::to_string(result.resonance_freq_mhz) + "MHz, " +
                     "Loop L: " + std::to_string(result.total_loop_inductance_nh) + "nH";
    return result;
}

} // namespace sf
