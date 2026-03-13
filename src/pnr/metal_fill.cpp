// SiliconForge — Metal Fill Insertion Implementation
// SI/timing-aware fill with pattern support and critical net avoidance.
#include "pnr/metal_fill.hpp"
#include <cmath>
#include <algorithm>

namespace sf {

// net_id = -2 means "fill" (non-signal)
static constexpr int NET_FILL = -2;

double MetalFillEngine::compute_layer_density(int layer) const {
    double die_area = pd_.die_area.area();
    if (die_area <= 0) return 0.0;

    double wire_area = 0;
    for (auto& w : pd_.wires) {
        if (w.layer != layer) continue;
        double dx = std::abs(w.end.x - w.start.x);
        double dy = std::abs(w.end.y - w.start.y);
        double length = dx + dy;
        wire_area += length * w.width;
    }
    return wire_area / die_area;
}

bool MetalFillEngine::near_critical_wire(double x, double y, double w, double h,
                                          int layer, double spacing) const {
    Rect fill_rect(x - spacing, y - spacing, x + w + spacing, y + h + spacing);
    for (auto& wire : pd_.wires) {
        if (wire.layer != layer) continue;
        if (critical_nets_.count(wire.net_id) == 0) continue;
        double wx0 = std::min(wire.start.x, wire.end.x) - wire.width / 2;
        double wy0 = std::min(wire.start.y, wire.end.y) - wire.width / 2;
        double wx1 = std::max(wire.start.x, wire.end.x) + wire.width / 2;
        double wy1 = std::max(wire.start.y, wire.end.y) + wire.width / 2;
        if (fill_rect.overlaps(Rect(wx0, wy0, wx1, wy1))) return true;
    }
    return false;
}

bool MetalFillEngine::near_cross_layer_wire(double x, double y, double w, double h,
                                              int layer, double spacing) const {
    Rect fill_rect(x - spacing, y - spacing, x + w + spacing, y + h + spacing);
    // Check adjacent layers (layer-1 and layer+1)
    for (auto& wire : pd_.wires) {
        int wl = wire.layer;
        if (wl != layer - 1 && wl != layer + 1) continue;
        if (wire.net_id == NET_FILL) continue; // skip other fills
        double wx0 = std::min(wire.start.x, wire.end.x) - wire.width / 2;
        double wy0 = std::min(wire.start.y, wire.end.y) - wire.width / 2;
        double wx1 = std::max(wire.start.x, wire.end.x) + wire.width / 2;
        double wy1 = std::max(wire.start.y, wire.end.y) + wire.width / 2;
        if (fill_rect.overlaps(Rect(wx0, wy0, wx1, wy1))) return true;
    }
    return false;
}

int MetalFillEngine::fill_layer(int layer, const MetalFillConfig& cfg, MetalFillResult& res) {
    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    if (die_w <= 0 || die_h <= 0) return 0;

    double current_density = compute_layer_density(layer);
    if (current_density >= cfg.min_density) return 0;

    double die_area = die_w * die_h;
    double target_area = cfg.min_density * die_area;
    double current_area = current_density * die_area;
    double needed_area = target_area - current_area;

    double fill_cell_area = cfg.fill_width * cfg.fill_height;
    if (fill_cell_area <= 0) return 0;

    double step_x = cfg.fill_width + cfg.fill_spacing;
    double step_y = cfg.fill_height + cfg.fill_spacing;

    int fills_inserted = 0;
    double area_added = 0;
    int row = 0;

    for (double y = pd_.die_area.y0 + cfg.fill_spacing;
         y + cfg.fill_height <= pd_.die_area.y1 - cfg.fill_spacing;
         y += step_y, ++row) {

        // Staggered pattern: offset every other row by half step
        double x_offset = 0;
        if (cfg.pattern == FillPattern::STAGGERED && (row % 2 == 1))
            x_offset = step_x / 2.0;

        int col = 0;
        for (double x = pd_.die_area.x0 + cfg.fill_spacing + x_offset;
             x + cfg.fill_width <= pd_.die_area.x1 - cfg.fill_spacing;
             x += step_x, ++col) {

            if (area_added >= needed_area) return fills_inserted;

            // Checkerboard: skip alternating positions
            if (cfg.pattern == FillPattern::CHECKERBOARD && ((row + col) % 2 == 1))
                continue;

            // Check overlap with existing wires on this layer
            Rect fill_rect(x - cfg.fill_spacing, y - cfg.fill_spacing,
                           x + cfg.fill_width + cfg.fill_spacing,
                           y + cfg.fill_height + cfg.fill_spacing);

            bool conflict = false;
            for (auto& w : pd_.wires) {
                if (w.layer != layer) continue;
                double wx0 = std::min(w.start.x, w.end.x) - w.width / 2;
                double wy0 = std::min(w.start.y, w.end.y) - w.width / 2;
                double wx1 = std::max(w.start.x, w.end.x) + w.width / 2;
                double wy1 = std::max(w.start.y, w.end.y) + w.width / 2;
                Rect wire_rect(wx0, wy0, wx1, wy1);
                if (fill_rect.overlaps(wire_rect)) { conflict = true; break; }
            }
            if (conflict) continue;

            // SI-aware: enlarged keepout near critical nets
            if (cfg.si_aware && !critical_nets_.empty()) {
                if (near_critical_wire(x, y, cfg.fill_width, cfg.fill_height,
                                       layer, cfg.critical_net_spacing)) {
                    res.fills_skipped_critical++;
                    continue;
                }
            }

            // Cross-layer awareness: avoid coupling to adjacent layer wires
            if (cfg.cross_layer_aware) {
                if (near_cross_layer_wire(x, y, cfg.fill_width, cfg.fill_height,
                                           layer, cfg.cross_layer_spacing)) {
                    res.fills_skipped_cross_layer++;
                    continue;
                }
            }

            // Insert fill
            if (cfg.pattern == FillPattern::SLOTTED) {
                // Slotted fill: two narrow rectangles with a gap
                double slot_h = cfg.fill_height * 0.4;
                double gap = cfg.fill_height * 0.2;
                pd_.wires.push_back({layer,
                    {x, y + slot_h / 2},
                    {x + cfg.fill_width, y + slot_h / 2},
                    slot_h, NET_FILL});
                pd_.wires.push_back({layer,
                    {x, y + slot_h + gap + slot_h / 2},
                    {x + cfg.fill_width, y + slot_h + gap + slot_h / 2},
                    slot_h, NET_FILL});
                fills_inserted += 2;
                area_added += cfg.fill_width * slot_h * 2;
            } else {
                pd_.wires.push_back({layer,
                    {x, y + cfg.fill_height / 2},
                    {x + cfg.fill_width, y + cfg.fill_height / 2},
                    cfg.fill_height, NET_FILL});
                fills_inserted++;
                area_added += fill_cell_area;
            }
        }
    }
    return fills_inserted;
}

MetalFillResult MetalFillEngine::fill(const MetalFillConfig& cfg) {
    MetalFillResult res;
    res.density_before.resize(cfg.num_layers, 0.0);
    res.density_after.resize(cfg.num_layers, 0.0);

    for (int l = 0; l < cfg.num_layers; ++l) {
        res.density_before[l] = compute_layer_density(l);
        int n = fill_layer(l, cfg, res);
        res.total_fills += n;
        res.density_after[l] = compute_layer_density(l);
    }

    res.message = "Metal fill: " + std::to_string(res.total_fills) +
                  " fills across " + std::to_string(cfg.num_layers) + " layers";
    if (res.fills_skipped_critical > 0)
        res.message += ", " + std::to_string(res.fills_skipped_critical) + " skipped (critical nets)";
    if (res.fills_skipped_cross_layer > 0)
        res.message += ", " + std::to_string(res.fills_skipped_cross_layer) + " skipped (cross-layer)";
    return res;
}

} // namespace sf
