// SiliconForge — Parasitic Extraction Implementation
#include "timing/parasitics.hpp"
#include <sstream>
#include <chrono>
#include <cmath>
#include <unordered_set>

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
            // Self-inductance estimate
            double l = length * params_.wire_ind_per_um;

            pn.segments.push_back({length, r, c, l, w->layer});
            pn.total_res_ohm += r;
            pn.total_cap_ff += c;
            pn.total_ind_nh += l;
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
            double l = length * params_.wire_ind_per_um;

            pn.segments.push_back({length, r, c, l, -1});
            pn.total_res_ohm += r;
            pn.total_cap_ff += c;
            pn.total_ind_nh += l;
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
       << "*R_UNIT 1 OHM\n"
       << "*L_UNIT 1 NH\n\n";

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
        // Inductance section
        if (pn.total_ind_nh > 0) {
            ss << "*INDUC\n";
            seg_id = 1;
            for (auto& seg : pn.segments) {
                if (seg.inductance > 0) {
                    ss << seg_id << " n" << pn.net_id << ":" << seg_id
                       << " n" << pn.net_id << ":" << (seg_id + 1)
                       << " " << seg.inductance << "\n";
                }
                seg_id++;
            }
        }
        if (pn.via_count > 0) {
            ss << "*VIA " << pn.via_count << "\n";
        }
        ss << "*END\n\n";
    }
    return ss.str();
}


void ParasiticResult::scale_parasitics(double temp_factor, double voltage_factor,
                                       double tcr, double vcc) {
    double r_scale = 1.0 + tcr * (temp_factor - 1.0);
    double c_scale = 1.0 + vcc * (voltage_factor - 1.0);
    double l_scale = 1.0 + 0.0005 * (temp_factor - 1.0);

    for (auto& pn : nets) {
        pn.total_res_ohm *= r_scale;
        pn.total_cap_ff *= c_scale;
        pn.total_ind_nh *= l_scale;
        pn.total_coupling_ff *= c_scale;
        for (auto& seg : pn.segments) {
            seg.resistance *= r_scale;
            seg.capacitance *= c_scale;
            seg.inductance *= l_scale;
        }
        for (auto& cc : pn.coupling)
            cc.coupling_ff *= c_scale;
        // Recompute Elmore delay with scaled values
        pn.elmore_delay_ps = pn.total_res_ohm * pn.total_cap_ff;
    }
}


// ============================================================================
// Phase 98: Spatial-grid coupling extraction — O(W·K) instead of O(W^2)
// ============================================================================

void ParasiticExtractor::build_spatial_index() {
    wire_entries_.clear();

    // Collect all wires with proper net_id mapping
    for (auto& w : pd_.wires) {
        WireEntry we;
        we.net_id = w.net_id;
        we.layer = w.layer;
        we.x0 = std::min(w.start.x, w.end.x);
        we.y0 = std::min(w.start.y, w.end.y);
        we.x1 = std::max(w.start.x, w.end.x);
        we.y1 = std::max(w.start.y, w.end.y);
        we.width = w.width;
        we.horizontal = (std::abs(w.end.y - w.start.y) < 0.01);

        // If net_id not tagged on wire, find by proximity
        if (we.net_id < 0) {
            double best_dist = 1e18;
            for (size_t ni = 0; ni < pd_.nets.size(); ni++) {
                for (auto cid : pd_.nets[ni].cell_ids) {
                    if (cid >= (int)pd_.cells.size()) continue;
                    double cx = pd_.cells[cid].position.x + pd_.cells[cid].width / 2;
                    double cy = pd_.cells[cid].position.y + pd_.cells[cid].height / 2;
                    double d = std::abs(w.start.x - cx) + std::abs(w.start.y - cy);
                    if (d < best_dist) { best_dist = d; we.net_id = (int)ni; }
                }
            }
        }
        if (we.net_id >= 0)
            wire_entries_.push_back(we);
    }

    if (wire_entries_.empty()) return;

    // Compute bounding box
    double xmin = 1e18, ymin = 1e18, xmax = -1e18, ymax = -1e18;
    int max_layer = 0;
    for (auto& we : wire_entries_) {
        xmin = std::min(xmin, we.x0);
        ymin = std::min(ymin, we.y0);
        xmax = std::max(xmax, we.x1);
        ymax = std::max(ymax, we.y1);
        max_layer = std::max(max_layer, we.layer);
    }

    grid_nx_ = coupling_cfg_.spatial_grid_bins;
    grid_ny_ = coupling_cfg_.spatial_grid_bins;
    grid_x0_ = xmin;
    grid_y0_ = ymin;
    grid_dx_ = std::max(0.01, (xmax - xmin) / grid_nx_);
    grid_dy_ = std::max(0.01, (ymax - ymin) / grid_ny_);

    // Build per-layer spatial grid
    int num_layers = max_layer + 2;  // +1 for layer 0, +1 for inter-layer
    spatial_grid_.resize(num_layers);
    for (auto& lg : spatial_grid_)
        lg.resize(grid_nx_ * grid_ny_);

    for (int wi = 0; wi < (int)wire_entries_.size(); ++wi) {
        auto& we = wire_entries_[wi];
        int lyr = std::max(0, std::min(we.layer, num_layers - 1));

        // Insert into all bins the wire overlaps
        int bx0 = std::max(0, (int)((we.x0 - grid_x0_) / grid_dx_));
        int by0 = std::max(0, (int)((we.y0 - grid_y0_) / grid_dy_));
        int bx1 = std::min(grid_nx_ - 1, (int)((we.x1 - grid_x0_) / grid_dx_));
        int by1 = std::min(grid_ny_ - 1, (int)((we.y1 - grid_y0_) / grid_dy_));

        for (int by = by0; by <= by1; ++by)
            for (int bx = bx0; bx <= bx1; ++bx)
                spatial_grid_[lyr][by * grid_nx_ + bx].wire_indices.push_back(wi);
    }
}

std::vector<int> ParasiticExtractor::query_nearby(int layer, double x0, double y0,
                                                    double x1, double y1) const {
    std::vector<int> result;
    if (layer < 0 || layer >= (int)spatial_grid_.size()) return result;

    double margin = coupling_cfg_.max_coupling_distance_um;
    int bx0 = std::max(0, (int)((x0 - margin - grid_x0_) / grid_dx_));
    int by0 = std::max(0, (int)((y0 - margin - grid_y0_) / grid_dy_));
    int bx1 = std::min(grid_nx_ - 1, (int)((x1 + margin - grid_x0_) / grid_dx_));
    int by1 = std::min(grid_ny_ - 1, (int)((y1 + margin - grid_y0_) / grid_dy_));

    std::unordered_set<int> seen;
    for (int by = by0; by <= by1; ++by)
        for (int bx = bx0; bx <= bx1; ++bx)
            for (int wi : spatial_grid_[layer][by * grid_nx_ + bx].wire_indices)
                if (seen.insert(wi).second)
                    result.push_back(wi);
    return result;
}

double ParasiticExtractor::compute_coupling_cap(const WireEntry& a,
                                                  const WireEntry& b) const {
    // Same-direction parallel segments: Cc = coeff * overlap / spacing
    // Perpendicular crossing: Cc = coeff * crossing_area / layer_height
    double overlap = 0;
    double spacing = 0;

    bool same_layer = (a.layer == b.layer);

    if (a.horizontal == b.horizontal) {
        // Parallel segments
        if (a.horizontal) {
            spacing = std::abs((a.y0 + a.y1) / 2.0 - (b.y0 + b.y1) / 2.0);
            double lo = std::max(a.x0, b.x0);
            double hi = std::min(a.x1, b.x1);
            overlap = std::max(0.0, hi - lo);
        } else {
            spacing = std::abs((a.x0 + a.x1) / 2.0 - (b.x0 + b.x1) / 2.0);
            double lo = std::max(a.y0, b.y0);
            double hi = std::min(a.y1, b.y1);
            overlap = std::max(0.0, hi - lo);
        }
    } else {
        // Perpendicular crossing — small area coupling
        double ax_lo = a.x0, ax_hi = a.x1, ay_lo = a.y0, ay_hi = a.y1;
        double bx_lo = b.x0, bx_hi = b.x1, by_lo = b.y0, by_hi = b.y1;
        double ox = std::max(0.0, std::min(ax_hi, bx_hi) - std::max(ax_lo, bx_lo));
        double oy = std::max(0.0, std::min(ay_hi, by_hi) - std::max(ay_lo, by_lo));
        if (ox > 0 && oy > 0) {
            double cross_area = std::max(ox, a.width) * std::max(oy, b.width);
            double layer_height = same_layer ? 0.1 : 0.3;  // ILD thickness estimate
            return params_.coupling_coeff * cross_area / layer_height;
        }
        return 0;
    }

    if (overlap <= 0 || spacing <= 0) return 0;
    if (spacing > coupling_cfg_.max_coupling_distance_um) return 0;

    // Wong's parallel plate model with fringe correction:
    // Cc_pp = eps * overlap * wire_height / spacing
    // Cc_fringe = eps * (2 * overlap) / (pi * log(1 + spacing/wire_height))
    // Simplified: Cc = coeff * overlap / spacing^alpha (alpha=0.8 for empirical fit)
    double coeff = same_layer ? params_.coupling_coeff
                              : params_.coupling_coeff * coupling_cfg_.inter_layer_coupling_factor;
    double alpha = 0.8;  // empirical exponent for sub-linear spacing dependence
    double cc = coeff * overlap / std::pow(spacing, alpha);

    // Miller effect doubling for switching aggressors
    if (coupling_cfg_.enable_miller_effect) {
        cc *= coupling_cfg_.miller_factor;
    }

    return cc;
}

void ParasiticExtractor::extract_coupling_spatial() {
    build_spatial_index();
    if (wire_entries_.empty()) return;

    int num_layers = (int)spatial_grid_.size();

    // For each wire, query same-layer and adjacent-layer neighbors
    for (int wi = 0; wi < (int)wire_entries_.size(); ++wi) {
        auto& a = wire_entries_[wi];

        // Same-layer neighbors
        auto nearby = query_nearby(a.layer, a.x0, a.y0, a.x1, a.y1);
        for (int wj : nearby) {
            if (wj <= wi) continue;  // avoid double counting
            auto& b = wire_entries_[wj];
            if (a.net_id == b.net_id) continue;

            double cc = compute_coupling_cap(a, b);
            if (cc > 0) {
                if (a.net_id < (int)result_cache_.size()) {
                    double overlap = std::max(a.x1 - a.x0, a.y1 - a.y0);
                    result_cache_[a.net_id].coupling.push_back(
                        {b.net_id, cc, overlap});
                }
                if (b.net_id < (int)result_cache_.size()) {
                    double overlap = std::max(b.x1 - b.x0, b.y1 - b.y0);
                    result_cache_[b.net_id].coupling.push_back(
                        {a.net_id, cc, overlap});
                }
            }
        }

        // Inter-layer coupling (layer above and below)
        for (int dl = -1; dl <= 1; dl += 2) {
            int adj_layer = a.layer + dl;
            if (adj_layer < 0 || adj_layer >= num_layers) continue;

            auto adj_nearby = query_nearby(adj_layer, a.x0, a.y0, a.x1, a.y1);
            for (int wj : adj_nearby) {
                auto& b = wire_entries_[wj];
                if (a.net_id == b.net_id) continue;
                if (wi < wj || a.layer != b.layer) {
                    // Inter-layer: only count once
                    double cc = compute_coupling_cap(a, b);
                    if (cc > 0) {
                        if (a.net_id < (int)result_cache_.size())
                            result_cache_[a.net_id].coupling.push_back(
                                {b.net_id, cc, 0});
                        if (b.net_id < (int)result_cache_.size())
                            result_cache_[b.net_id].coupling.push_back(
                                {a.net_id, cc, 0});
                    }
                }
            }
        }
    }
}

// AWE (Asymptotic Waveform Evaluation) delay computation
ParasiticExtractor::AweResult ParasiticExtractor::compute_awe(int net_idx,
                                                                int order) const {
    AweResult awe;
    if (net_idx < 0 || net_idx >= (int)result_cache_.size()) return awe;
    auto& pn = result_cache_[net_idx];
    if (pn.segments.empty()) return awe;

    // Build moments m0, m1, ..., m_{2q-1} of the transfer function
    // For an RC-ladder with N segments:
    //   m0 = 1 (DC gain)
    //   m1 = -sum(Ri * Ci_downstream) = -Elmore delay
    //   m2 = sum over pairs (Ri*Rj * path_cap_product)
    // AWE matches q moments to q poles.

    int n = (int)pn.segments.size();
    double total_c = pn.total_cap_ff + pn.total_coupling_ff;

    // Moment 1: Elmore delay (already computed)
    double m1 = 0;
    for (int i = 0; i < n; ++i) {
        double c_downstream = 0;
        for (int j = i; j < n; ++j)
            c_downstream += pn.segments[j].capacitance;
        // Add coupling cap contribution
        c_downstream += pn.total_coupling_ff / std::max(1, n);
        m1 += pn.segments[i].resistance * c_downstream;
    }

    // Moment 2: second-order RC moment
    double m2 = 0;
    for (int i = 0; i < n; ++i) {
        double c_down_i = 0;
        for (int j = i; j < n; ++j)
            c_down_i += pn.segments[j].capacitance;
        for (int k = i; k < n; ++k) {
            double c_down_k = 0;
            for (int j = k; j < n; ++j)
                c_down_k += pn.segments[j].capacitance;
            m2 += pn.segments[i].resistance * c_down_i *
                  pn.segments[k].resistance * c_down_k;
        }
    }

    if (order >= 2 && m1 > 0) {
        // Two-pole AWE approximation
        // p1 = -m1 / m2,  p2 adjusted for stability
        double p1 = -m1 * m1 / std::max(m2, m1 * m1 * 0.01);
        double p2 = p1 * 0.3;  // second pole typically faster

        awe.poles.push_back(p1);
        awe.poles.push_back(p2);
        awe.residues.push_back(0.7);
        awe.residues.push_back(0.3);

        // 50% delay: ln(2) / |dominant_pole|
        awe.delay_ps = 0.693 / std::abs(p1);
        // 20-80% slew: ln(4) / |dominant_pole|
        awe.slew_ps = 1.386 / std::abs(p1);
    } else {
        // Single-pole (Elmore)
        awe.delay_ps = m1;
        awe.slew_ps = 2.2 * m1;  // RC time constant approximation
        awe.poles.push_back(-1.0 / std::max(m1, 1e-6));
        awe.residues.push_back(1.0);
    }

    return awe;
}

// Legacy coupling extraction (keep as fallback)
void ParasiticExtractor::extract_coupling() {
    // Use spatial-grid-accelerated version
    extract_coupling_spatial();
}

} // namespace sf
