// SiliconForge — Signal Integrity Analyzer Implementation
#include "timing/signal_integrity.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace sf {

// ── Static helper methods ────────────────────────────────────────────────

double SignalIntegrityAnalyzer::effective_coupling(double cc_ff, double miller,
                                                    bool opposite_switching) {
    if (opposite_switching)
        return cc_ff * (1.0 + miller);
    else
        return cc_ff * (1.0 - miller);
}

double SignalIntegrityAnalyzer::timing_overlap(double a_start, double a_end,
                                                double v_start, double v_end) {
    if (a_end <= v_start || v_end <= a_start) return 0.0;
    double overlap = std::min(a_end, v_end) - std::max(a_start, v_start);
    double victim_window = v_end - v_start;
    if (victim_window <= 0) return 0.0;
    return std::min(1.0, overlap / victim_window);
}

double SignalIntegrityAnalyzer::compute_cid(double cc_eff_ff, double victim_cap_ff,
                                             double aggressor_slew_ps) {
    // CID = (Cc_eff / C_victim) × slew_aggressor
    if (victim_cap_ff <= 0) return 0.0;
    return (cc_eff_ff / victim_cap_ff) * aggressor_slew_ps;
}

bool SignalIntegrityAnalyzer::glitch_passes_filter(double glitch_width_ps,
                                                    double gate_prop_ps) {
    // Glitch propagates only if its width exceeds gate propagation delay
    return glitch_width_ps >= gate_prop_ps;
}

std::vector<NicEntry> SignalIntegrityAnalyzer::default_nic_table(double vdd) {
    double vdd_mv = vdd * 1000.0;
    return {
        {"INV",  vdd_mv * 0.35, vdd_mv * 0.50, 25.0},
        {"NAND", vdd_mv * 0.30, vdd_mv * 0.45, 30.0},
        {"NOR",  vdd_mv * 0.28, vdd_mv * 0.42, 30.0},
        {"AND2", vdd_mv * 0.30, vdd_mv * 0.45, 30.0},
        {"BUF",  vdd_mv * 0.40, vdd_mv * 0.55, 20.0},
        {"DFF",  vdd_mv * 0.25, vdd_mv * 0.38, 35.0},
    };
}

// ── Legacy private methods (unchanged logic) ─────────────────────────────

double SignalIntegrityAnalyzer::coupling_cap(int net_a, int net_b) const {
    if (net_a >= (int)pd_.nets.size() || net_b >= (int)pd_.nets.size()) return 0;

    double total_coupling = 0;
    for (auto& wa : pd_.wires) {
        for (auto& wb : pd_.wires) {
            if (wa.layer != wb.layer) continue;
            bool a_horiz = std::abs(wa.start.y - wa.end.y) < 0.01;
            bool b_horiz = std::abs(wb.start.y - wb.end.y) < 0.01;
            if (a_horiz != b_horiz) continue;

            double spacing;
            if (a_horiz) {
                spacing = std::abs((wa.start.y + wa.end.y)/2 - (wb.start.y + wb.end.y)/2);
            } else {
                spacing = std::abs((wa.start.x + wa.end.x)/2 - (wb.start.x + wb.end.x)/2);
            }

            if (spacing > 0 && spacing < cfg_.coupling_distance_um) {
                double overlap_len;
                if (a_horiz) {
                    double a_min = std::min(wa.start.x, wa.end.x);
                    double a_max = std::max(wa.start.x, wa.end.x);
                    double b_min = std::min(wb.start.x, wb.end.x);
                    double b_max = std::max(wb.start.x, wb.end.x);
                    overlap_len = std::max(0.0, std::min(a_max, b_max) - std::max(a_min, b_min));
                } else {
                    double a_min = std::min(wa.start.y, wa.end.y);
                    double a_max = std::max(wa.start.y, wa.end.y);
                    double b_min = std::min(wb.start.y, wb.end.y);
                    double b_max = std::max(wb.start.y, wb.end.y);
                    overlap_len = std::max(0.0, std::min(a_max, b_max) - std::max(a_min, b_min));
                }
                if (spacing > 0)
                    total_coupling += 0.05 * overlap_len / spacing; // fF
            }
        }
    }
    return total_coupling;
}

double SignalIntegrityAnalyzer::noise_voltage(double c_couple, double c_victim, double vdd) const {
    if (c_couple + c_victim == 0) return 0;
    return vdd * c_couple / (c_couple + c_victim);
}

// ── Main analysis ────────────────────────────────────────────────────────

SiResult SignalIntegrityAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    SiResult r;

    double noise_threshold = cfg_.vdd * cfg_.noise_threshold_pct / 100.0;
    r.nic_table = default_nic_table(cfg_.vdd);

    double total_overlap = 0;
    int overlap_count = 0;

    for (int i = 0; i < (int)pd_.nets.size(); ++i) {
        double worst_noise = 0;
        double worst_coupling = 0;
        double worst_cid = 0;
        int worst_agg = -1;

        for (int j = 0; j < (int)pd_.nets.size(); ++j) {
            if (i == j) continue;
            double cc = coupling_cap(i, j);
            if (cc <= 0) continue;

            double c_victim = cfg_.victim_base_cap_ff;
            for (auto cid : pd_.nets[i].cell_ids)
                c_victim += cfg_.pin_cap_ff;

            // Build aggressor-victim pair
            AggressorVictim av;
            av.aggressor_net = j;
            av.victim_net = i;
            av.coupling_cap_ff = cc;
            av.opposite_switching = true; // assume worst-case
            av.effective_cap_ff = effective_coupling(cc, cfg_.miller_factor,
                                                     av.opposite_switching);

            // Timing window: model as net-index-based spread
            double v_start = i * 10.0;
            double v_end = v_start + 50.0;
            double a_start = j * 10.0;
            double a_end = a_start + 50.0;
            av.timing_overlap = timing_overlap(a_start, a_end, v_start, v_end);

            if (av.timing_overlap > 0) {
                total_overlap += av.timing_overlap;
                overlap_count++;
                r.timing_window_overlaps++;
            }

            // CID calculation
            double cid = compute_cid(av.effective_cap_ff, c_victim,
                                      av.aggressor_slew_ps);
            cid *= av.timing_overlap; // weight by overlap
            av.cid_ps = cid;

            // Noise
            double noise = noise_voltage(av.effective_cap_ff, c_victim, cfg_.vdd);
            noise *= (av.timing_overlap > 0 ? av.timing_overlap : 1.0);
            av.noise_mv = noise * 1000.0;

            // Glitch width ~ proportional to coupling ratio × slew
            av.glitch_width_ps = (c_victim > 0) ?
                (av.effective_cap_ff / c_victim) * av.aggressor_slew_ps * 0.5 : 0;
            av.glitch_filtered = !glitch_passes_filter(av.glitch_width_ps,
                                                        cfg_.glitch_filter_ps);
            if (av.glitch_filtered)
                r.filtered_glitches++;

            r.av_details.push_back(av);

            if (noise > worst_noise) {
                worst_noise = noise;
                worst_coupling = cc;
                worst_agg = j;
            }
            if (cid > worst_cid) worst_cid = cid;
        }

        if (worst_coupling > 0) {
            CrosstalkVictim v;
            v.net_id = i;
            v.net_name = pd_.nets[i].name;
            v.coupling_cap_ff = worst_coupling;
            v.noise_mv = worst_noise * 1000;
            v.delay_impact_ps = worst_coupling * 0.5; // legacy simplified
            v.is_glitch_risk = worst_noise > noise_threshold;

            r.details.push_back(v);
            r.victims++;
            if (v.is_glitch_risk) r.glitch_risks++;
            r.worst_noise_mv = std::max(r.worst_noise_mv, v.noise_mv);
            r.worst_delay_ps = std::max(r.worst_delay_ps, v.delay_impact_ps);
        }
        if (worst_cid > r.worst_cid_ps) {
            r.worst_cid_ps = worst_cid;
            r.worst_aggressor_net = worst_agg;
        }
        r.nets_analyzed++;
    }

    r.avg_timing_overlap = overlap_count > 0 ? total_overlap / overlap_count : 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.victims) + " nets with crosstalk, " +
                std::to_string(r.glitch_risks) + " glitch risks, worst noise: " +
                std::to_string((int)r.worst_noise_mv) + "mV";
    return r;
}

} // namespace sf
