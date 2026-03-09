// SiliconForge — Signal Integrity Analyzer Implementation
#include "timing/signal_integrity.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace sf {

double SignalIntegrityAnalyzer::coupling_cap(int net_a, int net_b) const {
    if (net_a >= (int)pd_.nets.size() || net_b >= (int)pd_.nets.size()) return 0;
    auto& na = pd_.nets[net_a];
    auto& nb = pd_.nets[net_b];

    // Estimate coupling from wire proximity
    double total_coupling = 0;
    for (auto& wa : pd_.wires) {
        for (auto& wb : pd_.wires) {
            if (wa.layer != wb.layer) continue;
            // Parallel segments on same layer → coupling
            bool a_horiz = std::abs(wa.start.y - wa.end.y) < 0.01;
            bool b_horiz = std::abs(wb.start.y - wb.end.y) < 0.01;
            if (a_horiz != b_horiz) continue; // must be parallel

            double spacing;
            if (a_horiz) {
                spacing = std::abs((wa.start.y + wa.end.y)/2 - (wb.start.y + wb.end.y)/2);
            } else {
                spacing = std::abs((wa.start.x + wa.end.x)/2 - (wb.start.x + wb.end.x)/2);
            }

            if (spacing > 0 && spacing < 5.0) { // within coupling range
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
                // Coupling cap ∝ overlap_length / spacing
                if (spacing > 0)
                    total_coupling += 0.05 * overlap_len / spacing; // fF
            }
        }
    }
    return total_coupling;
}

double SignalIntegrityAnalyzer::noise_voltage(double c_couple, double c_victim, double vdd) const {
    // Simplified noise model: V_noise = V_dd × C_couple / (C_couple + C_victim)
    if (c_couple + c_victim == 0) return 0;
    return vdd * c_couple / (c_couple + c_victim);
}

SiResult SignalIntegrityAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    SiResult r;

    double noise_threshold = vdd_ * 0.15; // 15% of VDD = glitch risk

    // Analyze each net pair for coupling
    for (int i = 0; i < (int)pd_.nets.size(); ++i) {
        double worst_noise = 0;
        double worst_coupling = 0;

        for (int j = 0; j < (int)pd_.nets.size(); ++j) {
            if (i == j) continue;
            double cc = coupling_cap(i, j);
            if (cc > 0) {
                // Estimate victim capacitance
                double c_victim = 0.5; // fF base
                for (auto cid : pd_.nets[i].cell_ids)
                    c_victim += 0.002; // pin cap

                double noise = noise_voltage(cc, c_victim, vdd_);
                worst_noise = std::max(worst_noise, noise);
                worst_coupling = std::max(worst_coupling, cc);
            }
        }

        if (worst_coupling > 0) {
            CrosstalkVictim v;
            v.net_id = i;
            v.net_name = pd_.nets[i].name;
            v.coupling_cap_ff = worst_coupling;
            v.noise_mv = worst_noise * 1000; // V → mV
            v.delay_impact_ps = worst_coupling * 0.5; // simplified delay impact
            v.is_glitch_risk = worst_noise > noise_threshold;

            r.details.push_back(v);
            r.victims++;
            if (v.is_glitch_risk) r.glitch_risks++;
            r.worst_noise_mv = std::max(r.worst_noise_mv, v.noise_mv);
            r.worst_delay_ps = std::max(r.worst_delay_ps, v.delay_impact_ps);
        }
        r.nets_analyzed++;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.victims) + " nets with crosstalk, " +
                std::to_string(r.glitch_risks) + " glitch risks, worst noise: " +
                std::to_string((int)r.worst_noise_mv) + "mV";
    return r;
}

} // namespace sf
