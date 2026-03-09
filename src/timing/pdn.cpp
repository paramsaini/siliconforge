// SiliconForge — PDN Analyzer Implementation
#include "timing/pdn.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace sf {

void PdnAnalyzer::auto_config(double vdd, double current_ma) {
    cfg_.vdd = vdd;
    cfg_.total_current_ma = current_ma;
    cfg_.pad_count = 4;
    cfg_.stripes.clear();

    // Generate default PDN grid: horizontal + vertical stripes
    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    int h_stripes = std::max(2, (int)(die_h / 20.0));
    int v_stripes = std::max(2, (int)(die_w / 20.0));

    for (int i = 0; i < h_stripes; ++i) {
        double offset = pd_.die_area.y0 + (i + 0.5) * die_h / h_stripes;
        cfg_.stripes.push_back({PdnStripe::HORIZONTAL, offset, 1.0, 3, 0.05});
    }
    for (int i = 0; i < v_stripes; ++i) {
        double offset = pd_.die_area.x0 + (i + 0.5) * die_w / v_stripes;
        cfg_.stripes.push_back({PdnStripe::VERTICAL, offset, 1.0, 4, 0.05});
    }
}

double PdnAnalyzer::nearest_stripe_resistance(double x, double y) const {
    double min_dist = 1e18;
    double min_res = 1.0; // default high resistance

    for (auto& s : cfg_.stripes) {
        double dist;
        if (s.direction == PdnStripe::HORIZONTAL)
            dist = std::abs(y - s.offset);
        else
            dist = std::abs(x - s.offset);

        if (dist < min_dist) {
            min_dist = dist;
            min_res = dist * s.resistance_per_um + cfg_.pad_resistance;
        }
    }
    return min_res;
}

PdnResult PdnAnalyzer::analyze(int grid_res) {
    auto t0 = std::chrono::high_resolution_clock::now();
    PdnResult r;

    if (cfg_.stripes.empty()) auto_config();

    double cell_w = pd_.die_area.width() / grid_res;
    double cell_h = pd_.die_area.height() / grid_res;

    // Build current demand map
    std::vector<std::vector<double>> current_map(grid_res, std::vector<double>(grid_res, 0));
    double total_cell_area = 0;
    for (auto& c : pd_.cells) {
        if (!c.placed) continue;
        total_cell_area += c.width * c.height;
        int gx = std::clamp((int)((c.position.x - pd_.die_area.x0) / cell_w), 0, grid_res-1);
        int gy = std::clamp((int)((c.position.y - pd_.die_area.y0) / cell_h), 0, grid_res-1);
        current_map[gy][gx] += c.width * c.height;
    }

    // Normalize to total current
    if (total_cell_area > 0) {
        for (auto& row : current_map)
            for (auto& val : row)
                val = (val / total_cell_area) * cfg_.total_current_ma;
    }

    // Compute voltage drop at each grid point: ΔV = I × R_path
    double worst_drop = 0, total_drop = 0;
    int total_nodes = 0;

    for (int y = 0; y < grid_res; ++y) {
        for (int x = 0; x < grid_res; ++x) {
            double px = pd_.die_area.x0 + (x + 0.5) * cell_w;
            double py = pd_.die_area.y0 + (y + 0.5) * cell_h;
            double r_path = nearest_stripe_resistance(px, py);
            double i_local = current_map[y][x];
            double drop_v = i_local * r_path * 0.001; // mA × Ohm = mV
            double drop_mv = drop_v * 1000;
            double voltage = cfg_.vdd * 1000 - drop_mv;

            r.nodes.push_back({px, py, voltage / 1000.0, i_local});

            worst_drop = std::max(worst_drop, drop_mv);
            total_drop += drop_mv;
            total_nodes++;

            // Check EM limits
            for (auto& s : cfg_.stripes) {
                double dist;
                if (s.direction == PdnStripe::HORIZONTAL)
                    dist = std::abs(py - s.offset);
                else
                    dist = std::abs(px - s.offset);
                if (dist < cell_w) {
                    double current_density = i_local / s.width;
                    r.worst_current_density = std::max(r.worst_current_density, current_density);
                    if (current_density > cfg_.em_limit_ma_per_um)
                        r.em_violations++;
                }
            }
        }
    }

    r.worst_drop_mv = worst_drop;
    r.worst_drop_pct = (cfg_.vdd > 0) ? worst_drop / (cfg_.vdd * 10) : 0;
    r.avg_drop_mv = total_nodes > 0 ? total_drop / total_nodes : 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Worst drop: " + std::to_string((int)r.worst_drop_mv) + "mV (" +
                std::to_string((int)r.worst_drop_pct) + "%), " +
                std::to_string(r.em_violations) + " EM violations";
    return r;
}

} // namespace sf
