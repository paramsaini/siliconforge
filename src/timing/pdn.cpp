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

} // namespace sf
