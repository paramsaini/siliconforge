// SiliconForge — Noise / PSIJ Analyzer Implementation
#include "timing/noise.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace sf {

// ── Static methods ───────────────────────────────────────────────────────

SsnResult NoiseAnalyzer::compute_ssn(int num_outputs, double inductance_nh,
                                      double di_dt_ma_per_ns) {
    SsnResult s;
    s.num_outputs = num_outputs;
    s.effective_inductance_nh = inductance_nh;
    // V = N × L × di/dt  (nH × mA/ns = mV)
    s.l_di_dt_mv = inductance_nh * di_dt_ma_per_ns;
    s.peak_ssn_mv = num_outputs * s.l_di_dt_mv;
    return s;
}

double NoiseAnalyzer::timing_derating(double peak_noise_mv, double vdd_mv) {
    if (vdd_mv <= 0) return 0;
    // Derating: percentage penalty = noise / VDD × 100, capped at 25%
    return std::min(25.0, (peak_noise_mv / vdd_mv) * 100.0);
}

// ── Frequency sweep ──────────────────────────────────────────────────────

std::vector<FreqPoint> NoiseAnalyzer::frequency_sweep() const {
    std::vector<FreqPoint> pts;
    if (cfg_.freq_points <= 0) return pts;

    double f_start = cfg_.freq_start_mhz;
    double f_stop = cfg_.freq_stop_mhz;
    double log_start = std::log10(f_start);
    double log_stop = std::log10(f_stop);
    double step = (cfg_.freq_points > 1) ?
        (log_stop - log_start) / (cfg_.freq_points - 1) : 0;

    double C_die = cfg_.on_die_decap_nf * 1e-9;   // F
    double C_pkg = cfg_.pkg_decap_nf * 1e-9;       // F
    double L_pkg = cfg_.pkg_inductance_nh * 1e-9;   // H
    double R_esr = cfg_.board_esr_mohm * 1e-3;      // Ω
    double I_switch = cfg_.num_switching_outputs * cfg_.di_dt_ma_per_ns *
                      cfg_.output_rise_time_ns * 1e-3; // A (peak)

    for (int k = 0; k < cfg_.freq_points; ++k) {
        double f_mhz = std::pow(10.0, log_start + k * step);
        double f_hz = f_mhz * 1e6;
        double omega = 2.0 * M_PI * f_hz;

        // PDN impedance: Z = R_esr + j(ωL - 1/ωC_total)
        double C_total = C_die + C_pkg;
        double X_L = omega * L_pkg;
        double X_C = (C_total > 0 && omega > 0) ? 1.0 / (omega * C_total) : 0;
        double Z = std::sqrt(R_esr * R_esr + (X_L - X_C) * (X_L - X_C));

        FreqPoint fp;
        fp.freq_mhz = f_mhz;
        fp.impedance_mohm = Z * 1000.0; // Ω → mΩ
        fp.noise_mv = I_switch * Z * 1000.0; // V → mV
        pts.push_back(fp);
    }
    return pts;
}

// ── Legacy private methods ───────────────────────────────────────────────

double NoiseAnalyzer::estimate_supply_noise(double x, double y) const {
    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    double dx = std::min(x - pd_.die_area.x0, pd_.die_area.x1 - x);
    double dy = std::min(y - pd_.die_area.y0, pd_.die_area.y1 - y);
    double dist_to_edge = std::min(dx, dy);
    double max_dist = std::min(die_w, die_h) / 2;

    double base_noise = cfg_.vdd * 1000 * 0.03 * (dist_to_edge / max_dist);

    double cell_noise = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        double cx = c.position.x + c.width/2;
        double cy = c.position.y + c.height/2;
        double d = std::sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
        if (d > 0 && d < 20) {
            cell_noise += c.width * c.height * 0.1 / d;
        }
    }

    return base_noise + cell_noise;
}

double NoiseAnalyzer::noise_to_jitter(double noise_mv, double vdd_mv) const {
    double slew = vdd_mv / 100;
    return (slew > 0) ? noise_mv / slew : 0;
}

// ── Main analysis ────────────────────────────────────────────────────────

NoiseResult NoiseAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    NoiseResult r;
    double noise_margin = cfg_.vdd * 1000 * cfg_.noise_margin_pct / 100.0;
    r.noise_margin_mv = noise_margin;

    double total_noise_sq = 0;
    int count = 0;

    for (int ni = 0; ni < (int)pd_.nets.size(); ++ni) {
        auto& net = pd_.nets[ni];
        if (net.cell_ids.empty()) continue;

        double max_noise = 0;
        for (auto cid : net.cell_ids) {
            auto& c = pd_.cells[cid];
            double noise = estimate_supply_noise(
                c.position.x + c.width/2, c.position.y + c.height/2);
            max_noise = std::max(max_noise, noise);
        }

        double jitter = noise_to_jitter(max_noise, cfg_.vdd * 1000);
        bool violates = max_noise > noise_margin;

        r.details.push_back({ni, net.name, max_noise, jitter, violates});
        r.peak_noise_mv = std::max(r.peak_noise_mv, max_noise);
        r.psij_ps = std::max(r.psij_ps, jitter);
        total_noise_sq += max_noise * max_noise;
        if (violates) r.noise_violations++;
        count++;
    }

    r.rms_noise_mv = count > 0 ? std::sqrt(total_noise_sq / count) : 0;

    // SSN analysis
    r.ssn = compute_ssn(cfg_.num_switching_outputs,
                         cfg_.pkg_inductance_nh,
                         cfg_.di_dt_ma_per_ns);

    // Frequency spectrum
    r.freq_spectrum = frequency_sweep();

    // Noise budget: total margin minus all noise sources
    double total_noise = std::max(r.peak_noise_mv, r.ssn.peak_ssn_mv);
    r.noise_budget_mv = noise_margin;
    r.noise_slack_mv = noise_margin - total_noise;

    // Timing derating
    r.timing_derating_pct = timing_derating(total_noise, cfg_.vdd * 1000);

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Peak noise: " + std::to_string((int)r.peak_noise_mv) + "mV, " +
                "SSN: " + std::to_string((int)r.ssn.peak_ssn_mv) + "mV, " +
                "PSIJ: " + std::to_string((int)r.psij_ps) + "ps, " +
                std::to_string(r.noise_violations) + " violations";
    return r;
}

} // namespace sf
