// SiliconForge — ML Congestion Predictor
#include "ml/congestion_model.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace sf {

void MlCongestionPredictor::add_net_prob(int net_id, std::vector<std::vector<double>>& grid,
                                        double cell_w, double cell_h) const {
    auto& net = pd_.nets[net_id];
    if (net.cell_ids.size() < 2) return;

    double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
    for (auto cid : net.cell_ids) {
        auto& c = pd_.cells[cid];
        xmin = std::min(xmin, c.position.x);
        xmax = std::max(xmax, c.position.x);
        ymin = std::min(ymin, c.position.y);
        ymax = std::max(ymax, c.position.y);
    }

    // Grid coordinates
    int gx_min = std::max(0, (int)(xmin / cell_w));
    int gx_max = std::min(grid_x_ - 1, (int)(xmax / cell_w));
    int gy_min = std::max(0, (int)(ymin / cell_h));
    int gy_max = std::min(grid_y_ - 1, (int)(ymax / cell_h));

    // Area of bounding box
    int dx = gx_max - gx_min + 1;
    int dy = gy_max - gy_min + 1;
    
    // Feature extraction: estimated prob of routing crossing a bin
    // Simplification of RUDY (Rectangular Uniform Wire Density) model
    double prob = (double)(net.cell_ids.size()) / (dx * dy);

    // Spread probability to grid
    for (int y = gy_min; y <= gy_max; ++y) {
        for (int x = gx_min; x <= gx_max; ++x) {
            grid[y][x] += prob;
        }
    }
}

CongestionResult MlCongestionPredictor::predict() const {
    auto t0 = std::chrono::high_resolution_clock::now();
    CongestionResult r;
    r.heatmap_grid.resize(grid_y_, std::vector<double>(grid_x_, 0.0));

    double cell_w = pd_.die_area.width() / grid_x_;
    double cell_h = pd_.die_area.height() / grid_y_;

    // 1. Gather probabilistic routing density (RUDY)
    for (size_t ni = 0; ni < pd_.nets.size(); ++ni) {
        add_net_prob(ni, r.heatmap_grid, cell_w, cell_h);
    }

    // 2. Add local cell density weight to formulation
    // Congestion is worse where cells are densely packed
    for (const auto& c : pd_.cells) {
        int gx = std::clamp((int)(c.position.x / cell_w), 0, grid_x_ - 1);
        int gy = std::clamp((int)(c.position.y / cell_h), 0, grid_y_ - 1);
        r.heatmap_grid[gy][gx] += 0.5; // cell pin density factor
    }

    // 3. Compute metrics
    // Assume capacity threshold per bin (arbitrary normalization for now)
    double capacity = 4.0; 
    
    double sum = 0;
    for (int y = 0; y < grid_y_; ++y) {
        for (int x = 0; x < grid_x_; ++x) {
            double v = r.heatmap_grid[y][x];
            r.peak_congestion = std::max(r.peak_congestion, v / capacity);
            if (v > capacity) r.overflow_bins++;
            sum += (v / capacity);
        }
    }
    
    r.average_congestion = sum / (grid_x_ * grid_y_);

    // Normalize heatmap [0, 1] based on capacity for visualizer
    for (int y = 0; y < grid_y_; ++y) {
        for (int x = 0; x < grid_x_; ++x) {
            r.heatmap_grid[y][x] = std::min(r.heatmap_grid[y][x] / capacity, 1.0);
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return r;
}

} // namespace sf
