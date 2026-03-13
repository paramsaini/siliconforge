// SiliconForge — Power Grid Planning Implementation
#include "pnr/power_plan.hpp"
#include <cmath>

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
