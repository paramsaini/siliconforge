// SiliconForge — Parasitic Extraction Implementation
#include "timing/parasitics.hpp"
#include <sstream>
#include <chrono>
#include <cmath>

namespace sf {

ParasiticNet ParasiticExtractor::extract_net(int net_idx) {
    ParasiticNet pn;
    pn.net_id = net_idx;

    auto& net = pd_.nets[net_idx];
    if (net.cell_ids.size() < 2) return pn;

    // For each pair of connected cells, compute wire length and RC
    for (size_t i = 0; i + 1 < net.cell_ids.size(); ++i) {
        auto& c0 = pd_.cells[net.cell_ids[i]];
        auto& c1 = pd_.cells[net.cell_ids[i + 1]];

        // Manhattan distance between pin positions
        double px0 = c0.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c0.width/2);
        double py0 = c0.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c0.height/2);
        double px1 = c1.position.x + (i+1 < net.pin_offsets.size() ? net.pin_offsets[i+1].x : c1.width/2);
        double py1 = c1.position.y + (i+1 < net.pin_offsets.size() ? net.pin_offsets[i+1].y : c1.height/2);

        double length = std::abs(px1 - px0) + std::abs(py1 - py0);
        double r = length * params_.wire_res_per_um;
        double c = length * params_.wire_cap_per_um;

        // Add coupling capacitance estimate
        c += c * params_.coupling_cap_factor;

        pn.segments.push_back({length, r, c});
        pn.total_res_ohm += r;
        pn.total_cap_ff += c;
    }

    // Add via parasitics (one via per layer transition)
    size_t num_vias = 0;
    for (auto& w : pd_.wires) {
        // Count vias adjacent to this net (simplified)
    }
    pn.total_res_ohm += num_vias * params_.via_res;
    pn.total_cap_ff += num_vias * params_.via_cap;

    // Elmore delay: τ = R_total × C_total (simplified)
    pn.elmore_delay_ps = pn.total_res_ohm * pn.total_cap_ff;

    return pn;
}

ParasiticResult ParasiticExtractor::extract() {
    auto t0 = std::chrono::high_resolution_clock::now();
    ParasiticResult result;

    for (size_t i = 0; i < pd_.nets.size(); ++i) {
        result.nets.push_back(extract_net(i));
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

std::string ParasiticResult::to_spef() const {
    // SPEF-like output (Standard Parasitic Exchange Format)
    std::ostringstream ss;
    ss << "*SPEF \"IEEE 1481-2009\"\n"
       << "*DESIGN \"SiliconForge\"\n"
       << "*DATE \"2025-01-01\"\n"
       << "*VENDOR \"SiliconForge\"\n"
       << "*DIVIDER /\n"
       << "*DELIMITER :\n"
       << "*T_UNIT 1 PS\n"
       << "*C_UNIT 1 FF\n"
       << "*R_UNIT 1 OHM\n\n";

    for (auto& pn : nets) {
        if (pn.segments.empty()) continue;
        ss << "*D_NET n" << pn.net_id << " " << pn.total_cap_ff << "\n";
        ss << "*CONN\n";
        ss << "*CAP\n";
        int seg_id = 1;
        for (auto& seg : pn.segments) {
            ss << seg_id << " n" << pn.net_id << ":" << seg_id
               << " " << seg.capacitance << "\n";
            seg_id++;
        }
        ss << "*RES\n";
        seg_id = 1;
        for (auto& seg : pn.segments) {
            ss << seg_id << " n" << pn.net_id << ":" << seg_id
               << " n" << pn.net_id << ":" << (seg_id + 1)
               << " " << seg.resistance << "\n";
            seg_id++;
        }
        ss << "*END\n\n";
    }
    return ss.str();
}

} // namespace sf
