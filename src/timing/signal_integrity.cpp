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
            // Same-layer (lateral) coupling
            bool same_layer = (wa.layer == wb.layer);
            // Cross-layer (vertical) coupling: adjacent layers only
            bool cross_layer = (std::abs(wa.layer - wb.layer) == 1);
            if (!same_layer && !cross_layer) continue;

            bool a_horiz = std::abs(wa.start.y - wa.end.y) < 0.01;
            bool b_horiz = std::abs(wb.start.y - wb.end.y) < 0.01;

            double spacing;
            if (same_layer) {
                if (a_horiz != b_horiz) continue;  // orthogonal same-layer: no parallel coupling
                if (a_horiz)
                    spacing = std::abs((wa.start.y + wa.end.y)/2 - (wb.start.y + wb.end.y)/2);
                else
                    spacing = std::abs((wa.start.x + wa.end.x)/2 - (wb.start.x + wb.end.x)/2);
            } else {
                // Cross-layer: vertical spacing is ILD thickness (~0.3 um typical)
                spacing = 0.3;
            }

            if (spacing > 0 && spacing < cfg_.coupling_distance_um) {
                // Compute parallel run length (overlap in dominant direction)
                double overlap_len = 0;
                if (same_layer) {
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
                } else {
                    // Cross-layer overlap: project both onto X then Y, take geometric mean
                    double ax0 = std::min(wa.start.x, wa.end.x), ax1 = std::max(wa.start.x, wa.end.x);
                    double bx0 = std::min(wb.start.x, wb.end.x), bx1 = std::max(wb.start.x, wb.end.x);
                    double ox = std::max(0.0, std::min(ax1, bx1) - std::max(ax0, bx0));
                    double ay0 = std::min(wa.start.y, wa.end.y), ay1 = std::max(wa.start.y, wa.end.y);
                    double by0 = std::min(wb.start.y, wb.end.y), by1 = std::max(wb.start.y, wb.end.y);
                    double oy = std::max(0.0, std::min(ay1, by1) - std::max(ay0, by0));
                    overlap_len = std::max(ox, oy);
                }
                if (overlap_len > 0 && spacing > 0) {
                    // Coupling per-unit-length scaled by 1/spacing, cross-layer attenuated 0.6×
                    double scale = same_layer ? 1.0 : 0.6;
                    total_coupling += scale * 0.05 * overlap_len / spacing; // fF
                }
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
        double noise_sq_sum = 0;   // RSS accumulator for multi-aggressor superposition
        double total_cid = 0;      // sum of CID contributions from all aggressors

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
            total_cid += cid;  // aggregate CID from all aggressors

            // Noise
            double noise = noise_voltage(av.effective_cap_ff, c_victim, cfg_.vdd);
            noise *= (av.timing_overlap > 0 ? av.timing_overlap : 1.0);
            av.noise_mv = noise * 1000.0;

            // RSS noise superposition: σ_total² = Σσ_i²
            noise_sq_sum += (noise * 1000.0) * (noise * 1000.0);

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
            // Use RSS noise (multi-aggressor superposition) instead of single worst
            double rss_noise_mv = std::sqrt(noise_sq_sum);

            CrosstalkVictim v;
            v.net_id = i;
            v.net_name = pd_.nets[i].name;
            v.coupling_cap_ff = worst_coupling;
            v.noise_mv = rss_noise_mv;                 // RSS superposition
            v.delay_impact_ps = total_cid;              // aggregated CID from all aggressors
            v.is_glitch_risk = rss_noise_mv > noise_threshold * 1000.0;

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

// ── Enhanced: Identify All Aggressors ────────────────────────────────

std::vector<AggressorInfo> SignalIntegrityAnalyzer::identify_all_aggressors() {
    std::vector<AggressorInfo> result;
    for (int v = 0; v < (int)pd_.nets.size(); ++v) {
        for (int a = 0; a < (int)pd_.nets.size(); ++a) {
            if (v == a) continue;
            double cc = coupling_cap(v, a);
            if (cc <= 0) continue;

            AggressorInfo info;
            info.victim_net = v;
            info.aggressor_net = a;
            info.coupling_cap = cc;

            // Estimate parallel length from coupling (coupling ~ 0.05 * len / spacing)
            info.parallel_length = cc / 0.05 * cfg_.coupling_distance_um;
            info.shared_layer = 0;

            double v_start = v * 10.0, v_end = v_start + 50.0;
            double a_start = a * 10.0, a_end = a_start + 50.0;
            info.timing_window_overlap = timing_overlap(a_start, a_end, v_start, v_end);

            result.push_back(info);
        }
    }
    return result;
}

// ── Enhanced: Functional Filtering ───────────────────────────────────

std::vector<FunctionalFilter> SignalIntegrityAnalyzer::functional_filtering() {
    std::vector<FunctionalFilter> result;
    auto aggressors = identify_all_aggressors();
    for (auto& ag : aggressors) {
        FunctionalFilter ff;
        ff.victim_net = ag.victim_net;
        ff.aggressor_net = ag.aggressor_net;

        // Heuristic: if timing windows don't overlap, they can't switch simultaneously
        if (ag.timing_window_overlap < 0.01) {
            ff.can_switch_simultaneously = false;
            ff.reason = "no timing window overlap";
        }
        // Different clock domains heuristic (odd vs even net index)
        else if ((ag.victim_net % 2) != (ag.aggressor_net % 2)) {
            ff.can_switch_simultaneously = false;
            ff.reason = "different clock domains";
        } else {
            ff.can_switch_simultaneously = true;
            ff.reason = "same timing domain, overlap=" +
                std::to_string(ag.timing_window_overlap);
        }
        result.push_back(ff);
    }
    return result;
}

// ── Enhanced: Noise Propagation ──────────────────────────────────────

std::vector<NoisePropResult> SignalIntegrityAnalyzer::propagate_noise() {
    std::vector<NoisePropResult> result;
    double vdd = cfg_.vdd;
    double noise_margin = vdd * 1000.0 * 0.3;  // ~30% of VDD as noise margin

    for (int v = 0; v < (int)pd_.nets.size(); ++v) {
        double max_noise_mv = 0;
        double max_width_ps = 0;

        // Find worst aggressor noise for this victim
        for (int a = 0; a < (int)pd_.nets.size(); ++a) {
            if (v == a) continue;
            double cc = coupling_cap(v, a);
            if (cc <= 0) continue;

            double c_victim = cfg_.victim_base_cap_ff;
            for (auto cid : pd_.nets[v].cell_ids) {
                (void)cid;
                c_victim += cfg_.pin_cap_ff;
            }

            double noise = noise_voltage(
                effective_coupling(cc, cfg_.miller_factor, true), c_victim, vdd) * 1000.0;
            double width = (c_victim > 0) ? (cc / c_victim) * 50.0 * 0.5 : 0;
            if (noise > max_noise_mv) {
                max_noise_mv = noise;
                max_width_ps = width;
            }
        }

        if (max_noise_mv <= 0) continue;

        NoisePropResult np;
        np.victim_net = v;
        np.noise_amplitude_mv = max_noise_mv;
        np.noise_width_ps = max_width_ps;

        // Propagate through logic gates - each gate attenuates by gain factor
        int num_cells = (int)pd_.nets[v].cell_ids.size();
        np.gates_traversed = std::max(1, num_cells);
        double gain_per_gate = 0.7;  // typical noise attenuation
        np.attenuated_amplitude_mv = max_noise_mv * std::pow(gain_per_gate, np.gates_traversed);
        np.propagates_to_output = np.attenuated_amplitude_mv > noise_margin;

        result.push_back(np);
    }
    return result;
}

// ── Enhanced: Glitch Energy Check ────────────────────────────────────

std::vector<GlitchResult> SignalIntegrityAnalyzer::check_glitch_energy() {
    std::vector<GlitchResult> result;
    double ff_threshold_fj = 0.5;  // flip-flop capture energy threshold

    for (int v = 0; v < (int)pd_.nets.size(); ++v) {
        double total_charge_fc = 0;  // accumulated charge across all aggressors (fC)
        double total_energy_fj = 0;  // accumulated glitch energy (fJ)

        double c_victim = cfg_.victim_base_cap_ff;
        for (auto cid : pd_.nets[v].cell_ids) {
            (void)cid;
            c_victim += cfg_.pin_cap_ff;
        }

        for (int a = 0; a < (int)pd_.nets.size(); ++a) {
            if (v == a) continue;
            double cc = coupling_cap(v, a);
            if (cc <= 0) continue;

            double cc_eff = effective_coupling(cc, cfg_.miller_factor, true);
            double noise_v = noise_voltage(cc_eff, c_victim, cfg_.vdd);
            double noise_mv = noise_v * 1000.0;
            double width_ps = (c_victim > 0) ? (cc / c_victim) * 50.0 * 0.5 : 0;

            // Charge injection: Q = Cc_eff × ΔV (fF × V = fC)
            double charge = cc_eff * cfg_.vdd;  // worst-case full-swing aggressor
            total_charge_fc += charge;

            // Energy per aggressor: E = noise × width × Cc × 1e-3 → fJ
            double energy = noise_mv * width_ps * cc * 1e-3;
            total_energy_fj += energy;  // superpose across all aggressors
        }

        if (total_energy_fj > 0) {
            GlitchResult gr;
            gr.net_idx = v;
            gr.glitch_energy_fj = total_energy_fj;
            gr.threshold_fj = ff_threshold_fj;
            gr.is_functional_failure = total_energy_fj > ff_threshold_fj;
            result.push_back(gr);
        }
    }
    return result;
}

// ── Enhanced: SI-Aware Hold Check ────────────────────────────────────

std::vector<SiHoldFix> SignalIntegrityAnalyzer::check_si_hold() {
    std::vector<SiHoldFix> result;
    double base_hold_margin_ps = 50.0;

    for (int v = 0; v < (int)pd_.nets.size(); ++v) {
        double worst_cid = 0;
        for (int a = 0; a < (int)pd_.nets.size(); ++a) {
            if (v == a) continue;
            double cc = coupling_cap(v, a);
            if (cc <= 0) continue;

            double c_victim = cfg_.victim_base_cap_ff;
            for (auto cid : pd_.nets[v].cell_ids) {
                (void)cid;
                c_victim += cfg_.pin_cap_ff;
            }
            double cc_eff = effective_coupling(cc, cfg_.miller_factor, false);
            // Same-direction switching speeds up signal → reduces hold slack
            double cid = compute_cid(std::abs(cc_eff), c_victim, 50.0);
            worst_cid = std::max(worst_cid, cid);
        }

        if (worst_cid > 0) {
            SiHoldFix fix;
            fix.endpoint = v;
            fix.original_hold_slack = base_hold_margin_ps;
            fix.si_hold_slack = base_hold_margin_ps - worst_cid;
            fix.needs_fix = fix.si_hold_slack < 0;
            fix.buffer_delay_needed = fix.needs_fix ? -fix.si_hold_slack : 0;
            result.push_back(fix);
        }
    }
    return result;
}

// ── Enhanced: Full Enhanced SI Run ───────────────────────────────────

SiResult SignalIntegrityAnalyzer::run_enhanced() {
    SiResult r = analyze();

    auto aggressors = identify_all_aggressors();
    auto filters = functional_filtering();
    auto noise_prop = propagate_noise();
    auto glitches = check_glitch_energy();
    auto hold_fixes = check_si_hold();

    int functional_failures = 0;
    for (auto& g : glitches)
        if (g.is_functional_failure) functional_failures++;

    int hold_violations = 0;
    for (auto& h : hold_fixes)
        if (h.needs_fix) hold_violations++;

    int propagating = 0;
    for (auto& np : noise_prop)
        if (np.propagates_to_output) propagating++;

    r.message += " | Enhanced: " +
        std::to_string(aggressors.size()) + " aggressor pairs, " +
        std::to_string(functional_failures) + " glitch failures, " +
        std::to_string(hold_violations) + " SI-hold violations, " +
        std::to_string(propagating) + " propagating";
    return r;
}

} // namespace sf
