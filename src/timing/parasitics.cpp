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

    // Build net→wire mapping for layer-aware extraction
    // Use wire segments that belong to this net (by net_id tag)
    std::vector<const WireSegment*> net_wires;
    for (auto& w : pd_.wires) {
        if (w.net_id == net_idx) net_wires.push_back(&w);
    }

    // If we have actual routed wire segments, use layer-aware extraction
    if (!net_wires.empty()) {
        for (auto* w : net_wires) {
            double dx = std::abs(w->end.x - w->start.x);
            double dy = std::abs(w->end.y - w->start.y);
            double length = dx + dy;
            if (length < 1e-6) continue;

            // Per-layer R/C parameters
            double r_per_um = params_.wire_res_per_um;
            double c_per_um = params_.wire_cap_per_um;
            if (w->layer >= 0 && w->layer < (int)params_.layers.size()) {
                r_per_um = params_.layers[w->layer].res_per_um;
                c_per_um = params_.layers[w->layer].cap_per_um;
            }

            double r = length * r_per_um;
            double c = length * c_per_um;
            // Fringe capacitance (proportional to wire perimeter)
            c += length * params_.fringe_cap_per_um;

            pn.segments.push_back({length, r, c, w->layer});
            pn.total_res_ohm += r;
            pn.total_cap_ff += c;
        }
    } else {
        // Fallback: Manhattan distance between pin pairs (unrouted nets)
        for (size_t i = 0; i + 1 < net.cell_ids.size(); ++i) {
            auto& c0 = pd_.cells[net.cell_ids[i]];
            auto& c1 = pd_.cells[net.cell_ids[i + 1]];

            double px0 = c0.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c0.width/2);
            double py0 = c0.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c0.height/2);
            double px1 = c1.position.x + (i+1 < net.pin_offsets.size() ? net.pin_offsets[i+1].x : c1.width/2);
            double py1 = c1.position.y + (i+1 < net.pin_offsets.size() ? net.pin_offsets[i+1].y : c1.height/2);

            double length = std::abs(px1 - px0) + std::abs(py1 - py0);
            double r = length * params_.wire_res_per_um;
            double c = length * params_.wire_cap_per_um;
            c += c * params_.coupling_cap_factor;

            pn.segments.push_back({length, r, c, -1});
            pn.total_res_ohm += r;
            pn.total_cap_ff += c;
        }
    }

    // Via parasitics: count vias that connect to wires in this net
    int num_vias = 0;
    for (auto& v : pd_.vias) {
        // Check if via position is on any wire of this net
        for (auto* w : net_wires) {
            double wx0 = std::min(w->start.x, w->end.x) - w->width/2;
            double wy0 = std::min(w->start.y, w->end.y) - w->width/2;
            double wx1 = std::max(w->start.x, w->end.x) + w->width/2;
            double wy1 = std::max(w->start.y, w->end.y) + w->width/2;
            if (v.position.x >= wx0 && v.position.x <= wx1 &&
                v.position.y >= wy0 && v.position.y <= wy1 &&
                (v.lower_layer == w->layer || v.upper_layer == w->layer)) {
                num_vias++;
                break;
            }
        }
    }
    // Also estimate vias from layer transitions in wire segments
    if (num_vias == 0 && pn.segments.size() > 1) {
        int prev_layer = pn.segments[0].layer;
        for (size_t i = 1; i < pn.segments.size(); ++i) {
            if (pn.segments[i].layer != prev_layer && pn.segments[i].layer >= 0) {
                num_vias++;
                prev_layer = pn.segments[i].layer;
            }
        }
    }
    pn.via_count = num_vias;
    pn.total_res_ohm += num_vias * params_.via_res;
    pn.total_cap_ff += num_vias * params_.via_cap;

    // Elmore delay: proper RC-ladder model
    // τ = Σ(R_i × C_downstream_i) for each segment in the RC chain
    // C_downstream_i = sum of all capacitances from segment i to the sink
    if (!pn.segments.empty()) {
        pn.elmore_delay_ps = 0;
        double cumulative_r = 0;
        for (auto& seg : pn.segments) {
            // Distributed RC: each segment contributes R_seg × (C_seg/2 + C_downstream)
            double c_downstream = 0;
            bool found = false;
            for (auto& s2 : pn.segments) {
                if (found) c_downstream += s2.capacitance;
                if (&s2 == &seg) found = true;
            }
            pn.elmore_delay_ps += seg.resistance * (seg.capacitance / 2.0 + c_downstream);
        }
        // Add via contribution
        pn.elmore_delay_ps += num_vias * params_.via_res * (pn.total_cap_ff / pn.segments.size());
    }

    return pn;
}

ParasiticResult ParasiticExtractor::extract() {
    auto t0 = std::chrono::high_resolution_clock::now();
    ParasiticResult result;

    for (size_t i = 0; i < pd_.nets.size(); ++i) {
        result.nets.push_back(extract_net(i));
    }

    // Extract coupling capacitances between adjacent parallel wires
    result_cache_ = result.nets;
    extract_coupling();
    result.nets = result_cache_;
    
    // Add fringe cap to total
    for (auto& pn : result.nets) {
        for (auto& cc : pn.coupling)
            pn.total_coupling_ff += cc.coupling_ff;
        pn.total_cap_ff += pn.total_coupling_ff + 
            pn.segments.size() * params_.fringe_cap_per_um * 0.5;
        pn.elmore_delay_ps = pn.total_res_ohm * pn.total_cap_ff;
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
               << " " << seg.capacitance;
            if (seg.layer >= 0) ss << " ; layer=" << seg.layer;
            ss << "\n";
            seg_id++;
        }
        if (pn.total_coupling_ff > 0) {
            ss << "*CC\n";
            for (auto& cc : pn.coupling) {
                ss << "n" << pn.net_id << " n" << cc.aggressor_net 
                   << " " << cc.coupling_ff << "\n";
            }
        }
        ss << "*RES\n";
        seg_id = 1;
        for (auto& seg : pn.segments) {
            ss << seg_id << " n" << pn.net_id << ":" << seg_id
               << " n" << pn.net_id << ":" << (seg_id + 1)
               << " " << seg.resistance << "\n";
            seg_id++;
        }
        if (pn.via_count > 0) {
            ss << "*VIA " << pn.via_count << "\n";
        }
        ss << "*END\n\n";
    }
    return ss.str();
}


void ParasiticExtractor::extract_coupling() {
    // Compute coupling capacitance between parallel wire segments
    // For each pair of nets, find overlapping parallel wire segments
    // Cc = coupling_coeff * overlap_length / spacing
    
    struct WireSeg {
        int net_id;
        int layer;
        double x0, y0, x1, y1;
    };
    
    // Collect all wire segments per net
    std::vector<WireSeg> all_segs;
    for (auto& w : pd_.wires) {
        // Find which net this wire belongs to (by proximity to net cells)
        int best_net = -1;
        double best_dist = 1e18;
        for (size_t ni = 0; ni < pd_.nets.size(); ni++) {
            for (auto cid : pd_.nets[ni].cell_ids) {
                double cx = pd_.cells[cid].position.x + pd_.cells[cid].width/2;
                double cy = pd_.cells[cid].position.y + pd_.cells[cid].height/2;
                double d = std::abs(w.start.x - cx) + std::abs(w.start.y - cy);
                if (d < best_dist) { best_dist = d; best_net = (int)ni; }
            }
        }
        if (best_net >= 0)
            all_segs.push_back({best_net, w.layer, w.start.x, w.start.y, w.end.x, w.end.y});
    }
    
    // For each pair of segments on the same layer, compute coupling
    for (size_t i = 0; i < all_segs.size(); i++) {
        for (size_t j = i + 1; j < all_segs.size(); j++) {
            auto& a = all_segs[i];
            auto& b = all_segs[j];
            if (a.net_id == b.net_id || a.layer != b.layer) continue;
            
            // Check if parallel (both horizontal or both vertical)
            bool a_horiz = (std::abs(a.y1 - a.y0) < 0.01);
            bool b_horiz = (std::abs(b.y1 - b.y0) < 0.01);
            bool a_vert = (std::abs(a.x1 - a.x0) < 0.01);
            bool b_vert = (std::abs(b.x1 - b.x0) < 0.01);
            
            double overlap = 0;
            double spacing = 0;
            
            if (a_horiz && b_horiz) {
                spacing = std::abs(a.y0 - b.y0);
                double lo = std::max(std::min(a.x0, a.x1), std::min(b.x0, b.x1));
                double hi = std::min(std::max(a.x0, a.x1), std::max(b.x0, b.x1));
                overlap = std::max(0.0, hi - lo);
            } else if (a_vert && b_vert) {
                spacing = std::abs(a.x0 - b.x0);
                double lo = std::max(std::min(a.y0, a.y1), std::min(b.y0, b.y1));
                double hi = std::min(std::max(a.y0, a.y1), std::max(b.y0, b.y1));
                overlap = std::max(0.0, hi - lo);
            }
            
            if (overlap > 0 && spacing > 0 && spacing < 5.0) {
                // Cc = coeff * overlap / spacing (simplified field model)
                double cc = params_.coupling_coeff * overlap / spacing;
                if (a.net_id < (int)result_cache_.size())
                    result_cache_[a.net_id].coupling.push_back({b.net_id, cc, overlap});
                if (b.net_id < (int)result_cache_.size())
                    result_cache_[b.net_id].coupling.push_back({a.net_id, cc, overlap});
            }
        }
    }
}

} // namespace sf
