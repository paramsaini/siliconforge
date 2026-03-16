// SiliconForge — Power Grid Planning Implementation
#include "pnr/power_plan.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace sf {

// net_id conventions: -3 = VDD power, -4 = VSS ground
static constexpr int NET_VDD = -3;
static constexpr int NET_VSS = -4;

PowerPlanResult PowerPlanner::plan(const PowerPlanConfig& cfg) {
    PowerPlanResult res;
    create_rings(cfg, res);
    create_stripes(cfg, res);
    create_rails(cfg, res);
    create_vias(cfg, res);

    res.message = "Power plan: " + std::to_string(res.rings) + " rings, " +
                  std::to_string(res.stripes) + " stripes, " +
                  std::to_string(res.rails) + " rails, " +
                  std::to_string(res.vias) + " vias, wire=" +
                  std::to_string(res.total_wire_length) + " um";
    return res;
}

void PowerPlanner::create_rings(const PowerPlanConfig& cfg, PowerPlanResult& res) {
    double x0 = pd_.die_area.x0 + cfg.ring_offset;
    double y0 = pd_.die_area.y0 + cfg.ring_offset;
    double x1 = pd_.die_area.x1 - cfg.ring_offset;
    double y1 = pd_.die_area.y1 - cfg.ring_offset;

    auto add_ring = [&](double inset, int net_id) {
        double rx0 = x0 + inset;
        double ry0 = y0 + inset;
        double rx1 = x1 - inset;
        double ry1 = y1 - inset;

        // Bottom horizontal
        pd_.wires.push_back({cfg.ring_layer, {rx0, ry0}, {rx1, ry0}, cfg.ring_width, net_id});
        // Top horizontal
        pd_.wires.push_back({cfg.ring_layer, {rx0, ry1}, {rx1, ry1}, cfg.ring_width, net_id});
        // Left vertical
        pd_.wires.push_back({cfg.ring_layer, {rx0, ry0}, {rx0, ry1}, cfg.ring_width, net_id});
        // Right vertical
        pd_.wires.push_back({cfg.ring_layer, {rx1, ry0}, {rx1, ry1}, cfg.ring_width, net_id});

        double perim = 2.0 * ((rx1 - rx0) + (ry1 - ry0));
        res.total_wire_length += perim;
        res.rings++;
    };

    // VDD ring (outer)
    add_ring(0.0, NET_VDD);
    // VSS ring (inner, offset by ring_width + spacing)
    add_ring(cfg.ring_width + 0.5, NET_VSS);
}

void PowerPlanner::create_stripes(const PowerPlanConfig& cfg, PowerPlanResult& res) {
    double core_x0 = pd_.die_area.x0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
    double core_x1 = pd_.die_area.x1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;
    double core_y0 = pd_.die_area.y0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
    double core_y1 = pd_.die_area.y1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;

    if (core_x0 >= core_x1 || core_y0 >= core_y1) return;

    bool is_vdd = true;
    for (double x = core_x0; x <= core_x1; x += cfg.stripe_pitch) {
        int net_id = is_vdd ? NET_VDD : NET_VSS;
        pd_.wires.push_back({cfg.stripe_layer, {x, core_y0}, {x, core_y1},
                             cfg.stripe_width, net_id});
        res.total_wire_length += (core_y1 - core_y0);
        res.stripes++;
        is_vdd = !is_vdd;
    }
}

void PowerPlanner::create_rails(const PowerPlanConfig& cfg, PowerPlanResult& res) {
    double core_x0 = pd_.die_area.x0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
    double core_x1 = pd_.die_area.x1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;
    double core_y0 = pd_.die_area.y0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
    double core_y1 = pd_.die_area.y1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;

    if (core_x0 >= core_x1 || core_y0 >= core_y1) return;

    double row_h = pd_.row_height;
    if (row_h <= 0) row_h = 10.0;

    bool is_vdd = true;
    for (double y = core_y0; y <= core_y1; y += row_h) {
        int net_id = is_vdd ? NET_VDD : NET_VSS;
        pd_.wires.push_back({cfg.rail_layer, {core_x0, y}, {core_x1, y},
                             cfg.rail_width, net_id});
        res.total_wire_length += (core_x1 - core_x0);
        res.rails++;
        is_vdd = !is_vdd;
    }
}

void PowerPlanner::create_vias(const PowerPlanConfig& cfg, PowerPlanResult& res) {
    // Collect stripe X positions
    double core_x0 = pd_.die_area.x0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
    double core_x1 = pd_.die_area.x1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;
    double core_y0 = pd_.die_area.y0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
    double core_y1 = pd_.die_area.y1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;

    if (core_x0 >= core_x1 || core_y0 >= core_y1) return;

    double row_h = pd_.row_height;
    if (row_h <= 0) row_h = 10.0;

    // Via stacks at stripe-ring intersections (ring_layer to stripe_layer)
    double ring_y_bot = pd_.die_area.y0 + cfg.ring_offset;
    double ring_y_top = pd_.die_area.y1 - cfg.ring_offset;

    for (double x = core_x0; x <= core_x1; x += cfg.stripe_pitch) {
        // Bottom ring intersection
        for (int l = cfg.ring_layer; l < cfg.stripe_layer; ++l) {
            pd_.vias.push_back({{x, ring_y_bot}, l, l + 1});
            res.vias++;
        }
        // Top ring intersection
        for (int l = cfg.ring_layer; l < cfg.stripe_layer; ++l) {
            pd_.vias.push_back({{x, ring_y_top}, l, l + 1});
            res.vias++;
        }
    }

    // Via stacks at stripe-rail intersections (rail_layer to stripe_layer)
    for (double x = core_x0; x <= core_x1; x += cfg.stripe_pitch) {
        for (double y = core_y0; y <= core_y1; y += row_h) {
            for (int l = cfg.rail_layer; l < cfg.stripe_layer; ++l) {
                pd_.vias.push_back({{x, y}, l, l + 1});
                res.vias++;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Phase 96: IR Drop Feedback Loop, EM-aware Sizing, Adaptive Pitch
// ═══════════════════════════════════════════════════════════════════════════

void PowerPlanner::add_stripes_at_positions(const std::vector<double>& x_positions,
                                             const PowerPlanConfig& cfg,
                                             PowerPlanResult& res) {
    double y0 = pd_.die_area.y0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
    double y1 = pd_.die_area.y1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;

    for (double x : x_positions) {
        // VDD stripe
        WireSegment vdd_stripe;
        vdd_stripe.layer = cfg.stripe_layer;
        vdd_stripe.start = {x, y0};
        vdd_stripe.end   = {x, y1};
        vdd_stripe.width = cfg.stripe_width;
        vdd_stripe.net_id = NET_VDD;
        pd_.wires.push_back(vdd_stripe);

        // VSS stripe offset by one stripe width + spacing
        WireSegment vss_stripe;
        vss_stripe.layer = cfg.stripe_layer;
        vss_stripe.start = {x + cfg.stripe_width + 0.5, y0};
        vss_stripe.end   = {x + cfg.stripe_width + 0.5, y1};
        vss_stripe.width = cfg.stripe_width;
        vss_stripe.net_id = NET_VSS;
        pd_.wires.push_back(vss_stripe);

        res.stripes += 2;
        res.total_wire_length += 2 * (y1 - y0);
    }
}

double PowerPlanner::compute_em_width(double peak_current_ma, double j_max_em, double min_width) {
    if (j_max_em <= 0) return min_width;
    double required_width = peak_current_ma / j_max_em;
    return std::max(min_width, required_width);
}

std::vector<double> PowerPlanner::compute_adaptive_pitches(
    int num_regions, const std::vector<double>& region_currents,
    double min_pitch, double max_pitch) {
    if (num_regions <= 0 || region_currents.empty()) return {};

    double avg_current = 0;
    for (double c : region_currents) avg_current += c;
    avg_current /= (double)region_currents.size();
    if (avg_current <= 0) avg_current = 1.0;

    std::vector<double> pitches;
    for (int i = 0; i < num_regions && i < (int)region_currents.size(); ++i) {
        double ratio = region_currents[i] / avg_current;
        // High current -> small pitch (dense stripes), low current -> large pitch
        double t = std::max(0.0, std::min(1.0, (ratio - 0.5) / 1.0));
        double pitch = max_pitch - t * (max_pitch - min_pitch);
        pitch = std::max(min_pitch, std::min(max_pitch, pitch));
        pitches.push_back(pitch);
    }
    return pitches;
}

PowerPlanResult PowerPlanner::plan_with_ir_feedback(const PowerPlanConfig& cfg) {
    // 1. Run base power plan
    PowerPlanResult result = plan(cfg);

    double die_w = pd_.die_area.x1 - pd_.die_area.x0;
    double die_h = pd_.die_area.y1 - pd_.die_area.y0;
    if (die_w <= 0 || die_h <= 0) {
        result.ir_feedback.message = "Invalid die area — skipping IR feedback.";
        return result;
    }

    double total_current = cfg.total_current_ma * 1e-3; // Amperes
    IrFeedbackResult& fb = result.ir_feedback;

    // Resistive grid model: R_sheet ~0.03 ohm/sq for upper metals (SKY130 M5)
    double r_sheet = 0.03;
    double stripe_count = std::max(1.0, (double)result.stripes);
    double current_per_stripe = total_current / stripe_count;
    double stripe_length = die_h;
    double stripe_r = r_sheet * stripe_length / std::max(cfg.stripe_width, 0.01);
    double worst_ir_v = current_per_stripe * stripe_r;
    double worst_ir_mv = worst_ir_v * 1000.0;

    fb.initial_worst_ir_mv = worst_ir_mv;
    fb.ir_per_iteration.push_back(worst_ir_mv);

    // 2. Iterative feedback loop — add stripes at midpoints to reduce IR
    for (int iter = 0; iter < cfg.ir_max_iterations; ++iter) {
        fb.iterations = iter + 1;

        if (worst_ir_mv <= cfg.ir_target_mv) {
            fb.converged = true;
            break;
        }

        // Compute midpoints between existing stripes for densification
        double core_x0 = pd_.die_area.x0 + cfg.ring_offset + 2 * cfg.ring_width + 1.0;
        double core_x1 = pd_.die_area.x1 - cfg.ring_offset - 2 * cfg.ring_width - 1.0;

        // Collect existing stripe X positions on the stripe layer
        std::vector<double> existing_x;
        for (const auto& w : pd_.wires) {
            if (w.layer == cfg.stripe_layer &&
                (w.net_id == NET_VDD || w.net_id == NET_VSS)) {
                existing_x.push_back(w.start.x);
            }
        }
        std::sort(existing_x.begin(), existing_x.end());

        // Deduplicate
        existing_x.erase(std::unique(existing_x.begin(), existing_x.end()),
                         existing_x.end());

        // Find midpoints between consecutive stripes where spacing is large
        std::vector<double> hotspot_x;
        for (size_t i = 0; i + 1 < existing_x.size(); ++i) {
            double gap = existing_x[i + 1] - existing_x[i];
            if (gap > cfg.stripe_width * 4) {
                double mid = (existing_x[i] + existing_x[i + 1]) / 2.0;
                if (mid > core_x0 && mid < core_x1) {
                    hotspot_x.push_back(mid);
                }
            }
        }

        if (hotspot_x.empty()) break;

        add_stripes_at_positions(hotspot_x, cfg, result);
        fb.extra_stripes_added += (int)hotspot_x.size() * 2; // VDD + VSS each

        // Recompute IR with updated stripe count
        stripe_count += hotspot_x.size();
        current_per_stripe = total_current / stripe_count;
        stripe_r = r_sheet * stripe_length / std::max(cfg.stripe_width, 0.01);
        worst_ir_v = current_per_stripe * stripe_r;
        double new_ir_mv = worst_ir_v * 1000.0;

        double improvement = worst_ir_mv - new_ir_mv;
        worst_ir_mv = new_ir_mv;
        fb.ir_per_iteration.push_back(worst_ir_mv);

        if (improvement < cfg.ir_convergence_threshold) break;
    }

    fb.final_worst_ir_mv = worst_ir_mv;
    if (worst_ir_mv <= cfg.ir_target_mv) fb.converged = true;

    // 3. EM-aware stripe width sizing
    if (cfg.enable_em_sizing && cfg.j_max_em_ma_per_um > 0) {
        double i_per_stripe_ma = cfg.total_current_ma / std::max(1.0, stripe_count);
        double em_width = compute_em_width(i_per_stripe_ma, cfg.j_max_em_ma_per_um,
                                            cfg.stripe_width);
        if (em_width > cfg.stripe_width) {
            for (auto& w : pd_.wires) {
                if (w.layer == cfg.stripe_layer &&
                    (w.net_id == NET_VDD || w.net_id == NET_VSS)) {
                    w.width = em_width;
                    result.em_widened_stripes++;
                }
            }
        }
    }

    // Build summary message
    std::ostringstream ss;
    ss << "IR feedback: " << fb.iterations << " iterations, "
       << "IR " << fb.initial_worst_ir_mv << " -> " << fb.final_worst_ir_mv << " mV"
       << (fb.converged ? " (converged)" : " (not converged)")
       << ", " << fb.extra_stripes_added << " extra stripes";
    fb.message = ss.str();

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Tier 3: Multi-Vdd Power Grid
// ═══════════════════════════════════════════════════════════════════════════

void PowerPlanner::create_domain_rings(const DomainGrid& dg,
                                        const PowerPlanConfig& cfg,
                                        PowerPlanResult& res) {
    // Create power/ground rings around the domain region
    double rx0 = dg.region.x0, ry0 = dg.region.y0;
    double rx1 = dg.region.x1, ry1 = dg.region.y1;
    double rw = cfg.ring_width;

    // Top, bottom, left, right rings for VDD and VSS
    auto add_wire = [&](Point s, Point e) {
        WireSegment w;
        w.layer = cfg.ring_layer;
        w.start = s;
        w.end = e;
        w.width = rw;
        pd_.wires.push_back(w);
    };
    add_wire({rx0, ry1 - rw}, {rx1, ry1});
    add_wire({rx0, ry0}, {rx1, ry0 + rw});
    add_wire({rx0, ry0}, {rx0 + rw, ry1});
    add_wire({rx1 - rw, ry0}, {rx1, ry1});
    res.rings += 4;
}

void PowerPlanner::create_domain_stripes(const DomainGrid& dg,
                                          const PowerPlanConfig& cfg,
                                          PowerPlanResult& res) {
    double sx0 = dg.region.x0, sy0 = dg.region.y0;
    double sx1 = dg.region.x1, sy1 = dg.region.y1;

    for (double x = sx0 + cfg.stripe_pitch / 2; x < sx1; x += cfg.stripe_pitch) {
        WireSegment w;
        w.layer = cfg.stripe_layer;
        w.start = {x - cfg.stripe_width/2, sy0};
        w.end = {x + cfg.stripe_width/2, sy1};
        w.width = cfg.stripe_width;
        pd_.wires.push_back(w);
        res.stripes++;
        res.total_wire_length += (sy1 - sy0);

        // Via stack from stripe down to rails
        double row_h = pd_.row_height > 0 ? pd_.row_height : 1.4;
        for (double y = sy0; y <= sy1; y += row_h) {
            for (int l = cfg.rail_layer; l < cfg.stripe_layer; ++l) {
                Via v;
                v.position = {x, y};
                v.lower_layer = l;
                v.upper_layer = l + 1;
                pd_.vias.push_back(v);
                res.vias++;
            }
        }
    }
}

PowerPlanResult PowerPlanner::plan_multi_vdd(const PowerPlanConfig& cfg) {
    // First create the default global grid
    PowerPlanResult res = plan(cfg);

    // Then overlay per-domain grids
    for (auto& dg : domain_grids_) {
        if (dg.region.width() <= 0 || dg.region.height() <= 0) continue;
        create_domain_rings(dg, cfg, res);
        create_domain_stripes(dg, cfg, res);
    }

    res.message += " | Multi-Vdd: " + std::to_string(domain_grids_.size()) + " domains";
    return res;
}

} // namespace sf
