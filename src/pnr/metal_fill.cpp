// SiliconForge — Metal Fill Insertion Implementation
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

int MetalFillEngine::fill_layer(int layer, const MetalFillConfig& cfg) {
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

    // Grid-based fill: tile the die with fill candidates
    double step_x = cfg.fill_width + cfg.fill_spacing;
    double step_y = cfg.fill_height + cfg.fill_spacing;

    int fills_inserted = 0;
    double area_added = 0;

    for (double y = pd_.die_area.y0 + cfg.fill_spacing;
         y + cfg.fill_height <= pd_.die_area.y1 - cfg.fill_spacing;
         y += step_y) {
        for (double x = pd_.die_area.x0 + cfg.fill_spacing;
             x + cfg.fill_width <= pd_.die_area.x1 - cfg.fill_spacing;
             x += step_x) {

            if (area_added >= needed_area) return fills_inserted;

            // Check for overlap with existing wires on this layer
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

            // Insert fill as a horizontal wire segment
            pd_.wires.push_back({layer,
                                 {x, y + cfg.fill_height / 2},
                                 {x + cfg.fill_width, y + cfg.fill_height / 2},
                                 cfg.fill_height, NET_FILL});
            fills_inserted++;
            area_added += fill_cell_area;
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
        int n = fill_layer(l, cfg);
        res.total_fills += n;
        res.density_after[l] = compute_layer_density(l);
    }

    res.message = "Metal fill: " + std::to_string(res.total_fills) +
                  " fills across " + std::to_string(cfg.num_layers) + " layers";
    return res;
}

} // namespace sf
