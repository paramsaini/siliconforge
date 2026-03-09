// SiliconForge — IR Drop Analyzer Implementation
#include "timing/ir_drop.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace sf {

IrDropResult IrDropAnalyzer::analyze(int grid_res) {
    auto t0 = std::chrono::high_resolution_clock::now();
    IrDropResult r;
    r.vdd = vdd_;
    r.grid_x = grid_res;
    r.grid_y = grid_res;
    r.threshold_pct = 5.0;

    double cell_w = pd_.die_area.width() / grid_res;
    double cell_h = pd_.die_area.height() / grid_res;

    // Build current density map: how much current each grid cell draws
    std::vector<std::vector<double>> current_map(grid_res, std::vector<double>(grid_res, 0));
    double total_cell_area = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        total_cell_area += c.width * c.height;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, grid_res-1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, grid_res-1);
        // Current proportional to cell area
        current_map[gy][gx] += c.width * c.height;
    }

    // Normalize current map to total current
    if (total_cell_area > 0) {
        for (auto& row : current_map)
            for (auto& val : row)
                val = (val / total_cell_area) * total_current_ma_;
    }

    // Compute IR drop using resistive grid model
    // Simple model: IR drop increases with distance from the closest power pad
    // Assume power pads at corners
    double grid_resistance = 0.1; // Ohm per grid unit

    r.drop_map.resize(grid_res, std::vector<double>(grid_res, 0));
    double worst_drop = 0, total_drop = 0;
    int total_cells = 0;

    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            // Distance from nearest corner (power pad)
            double d_corner = std::min({
                std::sqrt((double)(x*x + y*y)),
                std::sqrt((double)((grid_res-1-x)*(grid_res-1-x) + y*y)),
                std::sqrt((double)(x*x + (grid_res-1-y)*(grid_res-1-y))),
                std::sqrt((double)((grid_res-1-x)*(grid_res-1-x) + (grid_res-1-y)*(grid_res-1-y)))
            });

            // IR drop = I × R × distance
            double local_current = current_map[y][x];
            double drop_v = local_current * grid_resistance * d_corner * 0.001;
            double drop_mv = drop_v * 1000;

            r.drop_map[y][x] = drop_mv;
            worst_drop = std::max(worst_drop, drop_mv);
            total_drop += drop_mv;
            total_cells++;

            // Check if hotspot
            if (drop_mv > vdd_ * 1000 * r.threshold_pct / 100.0) {
                Rect region(pd_.die_area.x0 + x * cell_w, pd_.die_area.y0 + y * cell_h,
                           pd_.die_area.x0 + (x+1) * cell_w, pd_.die_area.y0 + (y+1) * cell_h);
                r.hotspots.push_back({region, drop_mv, local_current});
                r.num_hotspots++;
            }
        }
    }

    r.worst_drop_mv = worst_drop;
    r.avg_drop_mv = total_cells > 0 ? total_drop / total_cells : 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Worst IR drop: " + std::to_string((int)r.worst_drop_mv) + "mV (" +
                std::to_string(r.worst_drop_mv / (vdd_ * 10)) + "%), " +
                std::to_string(r.num_hotspots) + " hotspot(s)";
    return r;
}

} // namespace sf
