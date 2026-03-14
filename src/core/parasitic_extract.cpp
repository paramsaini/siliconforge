// SiliconForge — SPEF Parasitic Extraction Engine Implementation
// Extracts RC parasitics from routed physical layout using geometric models
// and produces SpefData for STA back-annotation.

#include "core/parasitic_extract.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <unordered_set>
#include <unordered_map>

namespace sf {

// ── Construction ─────────────────────────────────────────────────────────

SpefExtractor::SpefExtractor(const PhysicalDesign& pd) : pd_(pd) {}

// ── Geometry helpers ─────────────────────────────────────────────────────

double SpefExtractor::wire_length_um(const WireSegment& seg) {
    double dx = std::abs(seg.end.x - seg.start.x);
    double dy = std::abs(seg.end.y - seg.start.y);
    return dx + dy; // Manhattan length
}

double SpefExtractor::metal_thickness(int layer) const {
    for (auto& e : cfg_.layer_stack)
        if (e.layer_id == layer) return e.thickness_um;
    return 0.3; // default metal thickness (μm)
}

double SpefExtractor::ild_height(int layer) const {
    return cfg_.ild_for_layer(layer);
}

double SpefExtractor::layer_spacing(int layer) const {
    for (auto& e : cfg_.layer_stack)
        if (e.layer_id == layer) return e.spacing_um;
    return 0.14; // default minimum spacing (μm)
}

// Compute overlap length of two wires running in the same direction.
// Returns 0 if wires are not parallel or have no spatial overlap.
double SpefExtractor::parallel_overlap_um(const WireSegment& a, const WireSegment& b) {
    bool a_horiz = (std::abs(a.end.y - a.start.y) < 1e-6);
    bool b_horiz = (std::abs(b.end.y - b.start.y) < 1e-6);

    if (a_horiz != b_horiz) return 0.0; // not parallel

    if (a_horiz) {
        double a_lo = std::min(a.start.x, a.end.x);
        double a_hi = std::max(a.start.x, a.end.x);
        double b_lo = std::min(b.start.x, b.end.x);
        double b_hi = std::max(b.start.x, b.end.x);
        double overlap = std::min(a_hi, b_hi) - std::max(a_lo, b_lo);
        return std::max(0.0, overlap);
    } else {
        double a_lo = std::min(a.start.y, a.end.y);
        double a_hi = std::max(a.start.y, a.end.y);
        double b_lo = std::min(b.start.y, b.end.y);
        double b_hi = std::max(b.start.y, b.end.y);
        double overlap = std::min(a_hi, b_hi) - std::max(a_lo, b_lo);
        return std::max(0.0, overlap);
    }
}

// Perpendicular distance between two parallel wire center-lines.
double SpefExtractor::perpendicular_distance(const WireSegment& a, const WireSegment& b) {
    bool a_horiz = (std::abs(a.end.y - a.start.y) < 1e-6);
    if (a_horiz) {
        double ay = (a.start.y + a.end.y) * 0.5;
        double by = (b.start.y + b.end.y) * 0.5;
        return std::abs(ay - by);
    } else {
        double ax = (a.start.x + a.end.x) * 0.5;
        double bx = (b.start.x + b.end.x) * 0.5;
        return std::abs(ax - bx);
    }
}

// ── Per-wire extraction ──────────────────────────────────────────────────

double SpefExtractor::compute_wire_resistance(const WireSegment& seg) const {
    double L_um = wire_length_um(seg);
    if (L_um < 1e-9) return 0.0;

    double W_um = seg.width;
    if (W_um < 1e-9) W_um = 0.1; // default wire width

    double t_um = metal_thickness(seg.layer);

    // R = ρ·L / (W·t)   — all in μm, ρ in Ω·m → convert lengths to m
    double L_m = L_um * 1e-6;
    double W_m = W_um * 1e-6;
    double t_m = t_um * 1e-6;

    return cfg_.metal_resistivity * L_m / (W_m * t_m); // Ω
}

double SpefExtractor::compute_wire_cap_to_ground(const WireSegment& seg) const {
    double L_um = wire_length_um(seg);
    if (L_um < 1e-9) return 0.0;

    double W_um = seg.width;
    if (W_um < 1e-9) W_um = 0.1;

    double h_um = ild_height(seg.layer); // dielectric height

    // Absolute permittivity (F/m)
    double eps = EPSILON_0 * cfg_.epsilon_r;

    // Parallel-plate cap: C_pp = ε·W·L / h  (all in metres)
    double L_m = L_um * 1e-6;
    double W_m = W_um * 1e-6;
    double h_m = h_um * 1e-6;

    double C_pp = eps * W_m * L_m / h_m; // Farads

    // Fringing capacitance: 2 × fringe_per_um × L
    double C_fringe = 2.0 * cfg_.fringe_cap_per_um * L_um * 1e-15; // fF → F

    return (C_pp + C_fringe) * 1e12; // convert to pF
}

double SpefExtractor::compute_coupling_cap(const WireSegment& a,
                                           const WireSegment& b) const {
    if (a.layer != b.layer) return 0.0; // only same-layer coupling

    double overlap = parallel_overlap_um(a, b);
    if (overlap < 1e-9) return 0.0;

    double dist = perpendicular_distance(a, b);
    double half_w_a = (a.width > 0 ? a.width : 0.1) * 0.5;
    double half_w_b = (b.width > 0 ? b.width : 0.1) * 0.5;
    double spacing = dist - half_w_a - half_w_b; // edge-to-edge
    if (spacing < 1e-3) spacing = 1e-3; // clamp to avoid singularity

    double t_um = metal_thickness(a.layer);
    double eps = EPSILON_0 * cfg_.epsilon_r;

    // C_couple = ε·L_overlap·t / spacing   (all in metres)
    double L_m = overlap * 1e-6;
    double t_m = t_um * 1e-6;
    double s_m = spacing * 1e-6;

    double C = eps * L_m * t_m / s_m; // Farads
    return C * 1e12; // pF
}

// ── Wire inductance ──────────────────────────────────────────────────────

double SpefExtractor::compute_wire_inductance(const WireSegment& seg) const {
    double L_um = wire_length_um(seg);
    if (L_um < 1e-9) return 0.0;

    double W_um = seg.width;
    if (W_um < 1e-9) W_um = 0.1;

    double t_um = metal_thickness(seg.layer);

    // Approximate partial self-inductance using Grover's formula for
    // a rectangular conductor: L ≈ (μ₀/(2π)) * l * [ln(2l/(w+t)) + 0.5 + (w+t)/(3l)]
    // all in SI units (metres, Henries)
    constexpr double MU_0 = 4.0 * 3.14159265358979323846 * 1e-7; // H/m
    double l_m = L_um * 1e-6;
    double w_m = W_um * 1e-6;
    double t_m = t_um * 1e-6;

    double wt = w_m + t_m;
    if (wt < 1e-12) wt = 1e-12;

    double ratio = 2.0 * l_m / wt;
    if (ratio < 1.0) ratio = 1.0;

    double L_henry = (MU_0 / (2.0 * 3.14159265358979323846)) * l_m *
                     (std::log(ratio) + 0.5 + wt / (3.0 * l_m));

    return L_henry; // Henries
}

// ── Wire index ───────────────────────────────────────────────────────────

SpefExtractor::WireIndex SpefExtractor::build_wire_index() const {
    int num_nets = static_cast<int>(pd_.nets.size());
    WireIndex idx(num_nets);
    for (int i = 0; i < static_cast<int>(pd_.wires.size()); ++i) {
        int nid = pd_.wires[i].net_id;
        if (nid >= 0 && nid < num_nets)
            idx[nid].push_back(i);
    }
    return idx;
}

// ── Lumped extraction ────────────────────────────────────────────────────

SpefNet SpefExtractor::extract_lumped(int net_id,
                                      const std::vector<int>& wire_ids) const {
    SpefNet sn;
    const auto& net = pd_.nets[net_id];
    sn.name = net.name;

    double total_R = 0.0;
    double total_C = 0.0;
    double total_L = 0.0;

    for (int wi : wire_ids) {
        const auto& seg = pd_.wires[wi];
        total_R += compute_wire_resistance(seg);
        total_C += compute_wire_cap_to_ground(seg);
        total_L += compute_wire_inductance(seg);
    }

    // Add via parasitics — estimate one via per layer transition
    int via_count = 0;
    if (wire_ids.size() > 1) {
        std::unordered_set<int> layers_used;
        for (int wi : wire_ids)
            layers_used.insert(pd_.wires[wi].layer);
        via_count = std::max(0, static_cast<int>(layers_used.size()) - 1);
    }
    total_R += via_count * cfg_.via_resistance_ohm;
    total_C += via_count * cfg_.via_cap_ff * 1e-3; // fF → pF

    sn.total_cap = total_C;

    // Pin parasitics (one entry per connected cell)
    for (size_t p = 0; p < net.cell_ids.size(); ++p) {
        SpefPinParasitic pp;
        int cid = net.cell_ids[p];
        if (cid >= 0 && cid < static_cast<int>(pd_.cells.size())) {
            pp.pin_name = pd_.cells[cid].name + ":Z";
            pp.x = pd_.cells[cid].position.x;
            pp.y = pd_.cells[cid].position.y;
        } else {
            pp.pin_name = sn.name + ":P" + std::to_string(p);
        }
        pp.cap = total_C / static_cast<double>(net.cell_ids.size());
        sn.pins.push_back(pp);
    }

    // Single lumped resistor between first and last pin
    if (sn.pins.size() >= 2) {
        SpefResistor r;
        r.id = 1;
        r.node1 = sn.pins.front().pin_name;
        r.node2 = sn.pins.back().pin_name;
        r.value = total_R;
        sn.resistors.push_back(r);
    }

    // Single ground cap on the net
    SpefCapacitor gc;
    gc.id = 1;
    gc.node1 = sn.name;
    gc.node2 = ""; // ground
    gc.value = total_C;
    sn.caps.push_back(gc);

    // Lumped inductance
    if (total_L > 0.0 && sn.pins.size() >= 2) {
        SpefInductor ind;
        ind.id = 1;
        ind.node1 = sn.pins.front().pin_name;
        ind.node2 = sn.pins.back().pin_name;
        ind.value = total_L;
        sn.inductors.push_back(ind);
        sn.total_ind = total_L;
    }

    return sn;
}

// ── Distributed extraction (RC-π segments) ───────────────────────────────

SpefNet SpefExtractor::extract_distributed(int net_id,
                                           const std::vector<int>& wire_ids) const {
    SpefNet sn;
    const auto& net = pd_.nets[net_id];
    sn.name = net.name;

    int N = std::max(1, cfg_.pi_segments);
    int rid = 0; // resistor counter
    int cid_counter = 0; // capacitor counter
    int lid = 0; // inductor counter
    double total_cap = 0.0;
    double total_ind = 0.0;

    // Pin parasitics
    for (size_t p = 0; p < net.cell_ids.size(); ++p) {
        SpefPinParasitic pp;
        int cid = net.cell_ids[p];
        if (cid >= 0 && cid < static_cast<int>(pd_.cells.size())) {
            pp.pin_name = pd_.cells[cid].name + ":Z";
            pp.x = pd_.cells[cid].position.x;
            pp.y = pd_.cells[cid].position.y;
        } else {
            pp.pin_name = sn.name + ":P" + std::to_string(p);
        }
        sn.pins.push_back(pp);
    }

    // For each wire segment, split into N π-sections
    for (size_t wi_idx = 0; wi_idx < wire_ids.size(); ++wi_idx) {
        const auto& seg = pd_.wires[wire_ids[wi_idx]];
        double seg_R = compute_wire_resistance(seg);
        double seg_C = compute_wire_cap_to_ground(seg);
        double seg_L = compute_wire_inductance(seg);

        if (seg_R < 1e-15 && seg_C < 1e-15 && seg_L < 1e-20) continue;

        double r_per_pi = seg_R / N;
        double c_per_pi = seg_C / N;
        double l_per_pi = seg_L / N;

        std::string base = sn.name + ":w" + std::to_string(wi_idx);

        for (int k = 0; k < N; ++k) {
            std::string node_a = base + "_" + std::to_string(k);
            std::string node_b = base + "_" + std::to_string(k + 1);

            // π-model: C/2 — R — C/2
            // Left cap (C/2)
            {
                SpefCapacitor c;
                c.id = ++cid_counter;
                c.node1 = node_a;
                c.node2 = ""; // ground
                c.value = c_per_pi * 0.5;
                total_cap += c.value;
                sn.caps.push_back(c);
            }

            // Resistor
            {
                SpefResistor r;
                r.id = ++rid;
                r.node1 = node_a;
                r.node2 = node_b;
                r.value = r_per_pi;
                sn.resistors.push_back(r);
            }

            // Inductor (series with resistor)
            if (l_per_pi > 1e-20) {
                SpefInductor ind;
                ind.id = ++lid;
                ind.node1 = node_a;
                ind.node2 = node_b;
                ind.value = l_per_pi;
                total_ind += ind.value;
                sn.inductors.push_back(ind);
            }

            // Right cap (C/2)
            {
                SpefCapacitor c;
                c.id = ++cid_counter;
                c.node1 = node_b;
                c.node2 = ""; // ground
                c.value = c_per_pi * 0.5;
                total_cap += c.value;
                sn.caps.push_back(c);
            }
        }
    }

    // Via parasitics between layer transitions
    {
        std::unordered_set<int> layers_used;
        for (int wi : wire_ids)
            layers_used.insert(pd_.wires[wi].layer);
        int via_count = std::max(0, static_cast<int>(layers_used.size()) - 1);
        for (int v = 0; v < via_count; ++v) {
            SpefResistor r;
            r.id = ++rid;
            r.node1 = sn.name + ":via" + std::to_string(v);
            r.node2 = sn.name + ":via" + std::to_string(v) + "_top";
            r.value = cfg_.via_resistance_ohm;
            sn.resistors.push_back(r);

            double vc = cfg_.via_cap_ff * 1e-3; // fF → pF
            SpefCapacitor c;
            c.id = ++cid_counter;
            c.node1 = r.node1;
            c.node2 = ""; // ground
            c.value = vc;
            total_cap += vc;
            sn.caps.push_back(c);
        }
    }

    sn.total_cap = total_cap;
    sn.total_ind = total_ind;

    // Distribute pin cap equally
    if (!sn.pins.empty()) {
        double pin_share = total_cap / static_cast<double>(sn.pins.size());
        for (auto& pp : sn.pins) pp.cap = pin_share;
    }

    return sn;
}

// ── Coupling capacitor extraction ────────────────────────────────────────

void SpefExtractor::extract_coupling_caps(const WireIndex& idx,
                                          std::vector<SpefNet>& nets) const {
    int num_nets = static_cast<int>(idx.size());

    for (int i = 0; i < num_nets; ++i) {
        for (int j = i + 1; j < num_nets; ++j) {
            if (idx[i].empty() || idx[j].empty()) continue;

            double total_cc = 0.0;

            // Check all wire pairs between nets i and j
            for (int wi : idx[i]) {
                for (int wj : idx[j]) {
                    double cc = compute_coupling_cap(pd_.wires[wi], pd_.wires[wj]);
                    total_cc += cc;
                }
            }

            if (total_cc < cfg_.coupling_threshold) continue;

            // Add coupling cap to both nets
            auto add_cc = [&](int net_idx, int other_net_idx, double val) {
                if (net_idx >= static_cast<int>(nets.size())) return;
                SpefCapacitor c;
                c.id = static_cast<int>(nets[net_idx].caps.size()) + 1;
                c.node1 = nets[net_idx].name;
                c.node2 = nets[other_net_idx].name;
                c.value = val;
                nets[net_idx].caps.push_back(c);
                nets[net_idx].total_cap += val;
            };

            add_cc(i, j, total_cc);
            add_cc(j, i, total_cc);
        }
    }
}

// ── Main extraction ──────────────────────────────────────────────────────

SpefData SpefExtractor::extract() {
    SpefData spef;
    spef.design_name = "siliconforge_extracted";

    // SPEF units: resistance in Ω, capacitance in pF, time in ns
    spef.units.res_scale = 1.0;
    spef.units.cap_scale = 1e-12;
    spef.units.time_scale = 1e-9;

    WireIndex wire_idx = build_wire_index();
    int num_nets = static_cast<int>(pd_.nets.size());

    spef.nets.reserve(num_nets);

    for (int i = 0; i < num_nets; ++i) {
        SpefNet sn;

        if (wire_idx[i].empty()) {
            // Net has no routed wires — emit stub with zero parasitics
            sn.name = pd_.nets[i].name;
            sn.total_cap = 0.0;
        } else {
            switch (cfg_.mode) {
            case ExtractionMode::LUMPED:
                sn = extract_lumped(i, wire_idx[i]);
                break;
            case ExtractionMode::DISTRIBUTED:
            case ExtractionMode::COUPLED:
                sn = extract_distributed(i, wire_idx[i]);
                break;
            }
        }

        spef.net_map[sn.name] = static_cast<int>(spef.nets.size());
        spef.nets.push_back(std::move(sn));
    }

    // Coupling pass (only in COUPLED mode)
    if (cfg_.mode == ExtractionMode::COUPLED) {
        extract_coupling_caps(wire_idx, spef.nets);
    }

    return spef;
}

SpefData SpefExtractor::extract_with_corner(const ExtractionCorner& corner) {
    SpefData spef = extract();
    spef.apply_corner(corner);
    return spef;
}

// ── Helper: resolve layer id to name ─────────────────────────────────────

std::string SpefExtractor::layer_name(int layer_id) const {
    for (const auto& l : pd_.layers)
        if (l.id == layer_id) return l.name;
    return "M" + std::to_string(layer_id);
}

// ── Cross-layer coupling capacitance ─────────────────────────────────────
// Models overlap-area-based coupling between wires on adjacent metal layers.
// C_cross = ε · overlap_area / ILD_thickness

double SpefExtractor::compute_coupling_cap_cross_layer(const WireSegment& a,
                                                       const WireSegment& b) const {
    int layer_diff = std::abs(a.layer - b.layer);
    if (layer_diff != 1) return 0.0; // only adjacent layers

    // Compute bounding-box overlap area
    double ax0 = std::min(a.start.x, a.end.x);
    double ax1 = std::max(a.start.x, a.end.x);
    double ay0 = std::min(a.start.y, a.end.y);
    double ay1 = std::max(a.start.y, a.end.y);

    double bx0 = std::min(b.start.x, b.end.x);
    double bx1 = std::max(b.start.x, b.end.x);
    double by0 = std::min(b.start.y, b.end.y);
    double by1 = std::max(b.start.y, b.end.y);

    // Add half-widths to create wire area envelopes
    double wa = (a.width > 0 ? a.width : 0.1) * 0.5;
    double wb = (b.width > 0 ? b.width : 0.1) * 0.5;

    double ox = std::max(0.0, std::min(ax1 + wa, bx1 + wb) - std::max(ax0 - wa, bx0 - wb));
    double oy = std::max(0.0, std::min(ay1 + wa, by1 + wb) - std::max(ay0 - wa, by0 - wb));
    double area_um2 = ox * oy;
    if (area_um2 < 1e-9) return 0.0;

    int upper_layer = std::max(a.layer, b.layer);
    double ild_um = ild_height(upper_layer);
    if (ild_um < 1e-6) ild_um = 0.5;

    double eps = EPSILON_0 * cfg_.epsilon_r;
    double area_m2 = area_um2 * 1e-12;
    double ild_m = ild_um * 1e-6;

    double C = eps * area_m2 / ild_m; // Farads
    return C * 1e15; // fF
}

// ── Multi-layer crosstalk extraction ─────────────────────────────────────

CrosstalkExtractionResult SpefExtractor::extract_crosstalk(const std::string& victim_net) {
    CrosstalkExtractionResult result;

    // Find victim net index
    int victim_idx = -1;
    for (int i = 0; i < static_cast<int>(pd_.nets.size()); ++i) {
        if (pd_.nets[i].name == victim_net) { victim_idx = i; break; }
    }
    if (victim_idx < 0) return result;

    WireIndex wire_idx = build_wire_index();
    const auto& victim_wires = wire_idx[victim_idx];
    if (victim_wires.empty()) return result;

    std::unordered_set<std::string> aggressor_names;

    for (int ni = 0; ni < static_cast<int>(pd_.nets.size()); ++ni) {
        if (ni == victim_idx) continue;
        const auto& agg_wires = wire_idx[ni];
        if (agg_wires.empty()) continue;

        // Per-layer accumulation
        std::unordered_map<int, double> layer_cc;       // layer -> coupling fF
        std::unordered_map<int, double> layer_overlap;   // layer -> parallel length um

        for (int vi : victim_wires) {
            for (int ai : agg_wires) {
                const auto& vw = pd_.wires[vi];
                const auto& aw = pd_.wires[ai];

                // Same-layer lateral coupling
                if (vw.layer == aw.layer) {
                    double cc = compute_coupling_cap(vw, aw);
                    if (cc > 0.0) {
                        double cc_ff = cc * 1e3; // pF → fF
                        layer_cc[vw.layer] += cc_ff;
                        layer_overlap[vw.layer] += parallel_overlap_um(vw, aw);
                    }
                }

                // Cross-layer vertical coupling
                if (std::abs(vw.layer - aw.layer) == 1) {
                    double cc_ff = compute_coupling_cap_cross_layer(vw, aw);
                    if (cc_ff > 0.0) {
                        int report_layer = std::max(vw.layer, aw.layer);
                        layer_cc[report_layer] += cc_ff;
                        // No meaningful parallel_length for cross-layer
                    }
                }
            }
        }

        // Emit one CouplingCap per layer with non-trivial coupling
        for (auto& [lid, cc_ff] : layer_cc) {
            double thresh_ff = cfg_.coupling_threshold * 1e3; // pF → fF
            if (cc_ff < thresh_ff) continue;

            CouplingCap cap;
            cap.aggressor_net = pd_.nets[ni].name;
            cap.victim_net = victim_net;
            cap.layer = layer_name(lid);
            cap.coupling_cap = cc_ff;
            cap.parallel_length = layer_overlap.count(lid) ? layer_overlap[lid] : 0.0;
            result.coupling_caps.push_back(cap);
            result.total_coupling_cap += cc_ff;
            aggressor_names.insert(pd_.nets[ni].name);
        }
    }

    result.num_aggressors = static_cast<int>(aggressor_names.size());
    return result;
}

// ── Frequency-dependent coupling capacitance ─────────────────────────────
// C(f) = C_dc * (1 + alpha * sqrt(f / f_ref))
// alpha ≈ 0.1, f_ref = 1 GHz

double SpefExtractor::frequency_dependent_cap(double coupling_cap_dc,
                                              double freq_ghz,
                                              double skin_depth_um) {
    if (freq_ghz <= 0.0) return coupling_cap_dc;

    constexpr double alpha = 0.1;
    constexpr double f_ref_ghz = 1.0;

    // Skin-depth scaling: thinner skin depth → stronger effect
    double skin_factor = 0.5 / std::max(skin_depth_um, 0.01);

    double correction = alpha * skin_factor * std::sqrt(freq_ghz / f_ref_ghz);
    return coupling_cap_dc * (1.0 + correction);
}

} // namespace sf
