// SiliconForge — Analytical Placer Implementation
#include "pnr/placer.hpp"
#include <algorithm>
#include <numeric>
#include <chrono>
#include <iostream>
#include <cmath>
#include <cassert>

namespace sf {

double AnalyticalPlacer::compute_hpwl() const {
    return pd_.total_wirelength();
}

void AnalyticalPlacer::solve_quadratic() {
    // Simplified analytical placement: for each cell, compute the weighted
    // average position based on connected cells' positions (Gauss-Seidel iteration)
    //
    // For net n connecting cells i,j: w_ij = 1/|n|
    // x_i = (∑ w_ij * x_j + w_pad * pad_x) / (∑ w_ij + w_pad)

    int n = (int)pd_.cells.size();
    if (n == 0) return;

    // Initial placement: random-ish within die
    for (int i = 0; i < n; ++i) {
        if (!pd_.cells[i].placed) {
            double fx = (double)(i % 10) / 10.0;
            double fy = (double)(i / 10) / 10.0;
            pd_.cells[i].position.x = pd_.die_area.x0 + fx * pd_.die_area.width() * 0.8 + pd_.die_area.width() * 0.1;
            pd_.cells[i].position.y = pd_.die_area.y0 + fy * pd_.die_area.height() * 0.8 + pd_.die_area.height() * 0.1;
            pd_.cells[i].placed = true;
        }
    }

    // Build adjacency weights per cell
    std::vector<std::vector<std::pair<int, double>>> adj(n); // adj[i] = {(j, weight)}
    for (auto& net : pd_.nets) {
        if (net.cell_ids.size() < 2) continue;
        double w = 1.0 / (net.cell_ids.size() - 1);
        for (size_t a = 0; a < net.cell_ids.size(); ++a) {
            for (size_t b = a + 1; b < net.cell_ids.size(); ++b) {
                int ci = net.cell_ids[a], cj = net.cell_ids[b];
                adj[ci].push_back({cj, w});
                adj[cj].push_back({ci, w});
            }
        }
    }

    // Pad connections to anchors (keep cells inside die)
    double pad_weight = 0.01;

    // Gauss-Seidel iterations
    for (int iter = 0; iter < 50; ++iter) {
        double max_move = 0;
        for (int i = 0; i < n; ++i) {
            double sum_wx = 0, sum_wy = 0, sum_w = 0;

            for (auto& [j, w] : adj[i]) {
                sum_wx += w * pd_.cells[j].position.x;
                sum_wy += w * pd_.cells[j].position.y;
                sum_w += w;
            }

            // Anchor to die center
            double cx = pd_.die_area.center().x;
            double cy = pd_.die_area.center().y;
            sum_wx += pad_weight * cx;
            sum_wy += pad_weight * cy;
            sum_w += pad_weight;

            if (sum_w > 0) {
                double new_x = sum_wx / sum_w;
                double new_y = sum_wy / sum_w;
                max_move = std::max(max_move, std::abs(new_x - pd_.cells[i].position.x));
                max_move = std::max(max_move, std::abs(new_y - pd_.cells[i].position.y));
                pd_.cells[i].position.x = new_x;
                pd_.cells[i].position.y = new_y;
            }
        }
        if (max_move < 0.01) break;
    }
}

void AnalyticalPlacer::spread() {
    // Simple cell spreading: sort by x, distribute evenly across rows
    int n = (int)pd_.cells.size();
    if (n == 0) return;

    // Sort cells by x position
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return pd_.cells[a].position.x < pd_.cells[b].position.x;
    });

    // Compute row layout
    int num_rows = std::max(1, (int)(pd_.die_area.height() / pd_.row_height));
    int cells_per_row = (n + num_rows - 1) / num_rows;

    for (int i = 0; i < n; ++i) {
        int row = i / cells_per_row;
        int col = i % cells_per_row;
        auto& c = pd_.cells[order[i]];

        double row_y = pd_.die_area.y0 + row * pd_.row_height;
        double col_x = pd_.die_area.x0 + col * (pd_.die_area.width() / cells_per_row);

        // Blend: 70% quadratic solution, 30% spread position
        c.position.x = c.position.x * 0.7 + col_x * 0.3;
        c.position.y = c.position.y * 0.7 + row_y * 0.3;
    }
}

void AnalyticalPlacer::legalize() {
    // Snap cells to row grid and resolve overlaps with left-edge greedy
    int num_rows = std::max(1, (int)(pd_.die_area.height() / pd_.row_height));

    // Assign each cell to nearest row
    for (auto& c : pd_.cells) {
        double best_y = pd_.die_area.y0;
        double best_dist = 1e18;
        for (int r = 0; r < num_rows; ++r) {
            double ry = pd_.die_area.y0 + r * pd_.row_height;
            double d = std::abs(c.position.y - ry);
            if (d < best_dist) { best_dist = d; best_y = ry; }
        }
        c.position.y = best_y;
    }

    // For each row, sort by x and pack left-to-right
    for (int r = 0; r < num_rows; ++r) {
        double ry = pd_.die_area.y0 + r * pd_.row_height;
        std::vector<int> row_cells;
        for (size_t i = 0; i < pd_.cells.size(); ++i) {
            if (std::abs(pd_.cells[i].position.y - ry) < 0.01)
                row_cells.push_back(i);
        }
        std::sort(row_cells.begin(), row_cells.end(), [&](int a, int b) {
            return pd_.cells[a].position.x < pd_.cells[b].position.x;
        });

        double next_x = pd_.die_area.x0;
        for (int ci : row_cells) {
            pd_.cells[ci].position.x = std::max(pd_.cells[ci].position.x, next_x);
            next_x = pd_.cells[ci].position.x + pd_.cells[ci].width;
        }
    }
}

PlaceResult AnalyticalPlacer::place() {
    auto t0 = std::chrono::high_resolution_clock::now();

    solve_quadratic();
    spread();
    legalize();

    auto t1 = std::chrono::high_resolution_clock::now();
    PlaceResult r;
    r.hpwl = compute_hpwl();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.legal = !pd_.has_overlaps();
    r.message = r.legal ? "Placement legal — no overlaps" : "Placement has overlaps";
    return r;
}

} // namespace sf
