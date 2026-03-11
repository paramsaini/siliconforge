// SiliconForge — Analytical Placer (SimPL-style)
// Conjugate-gradient solver + density bin spreading + Abacus legalization
#include "pnr/placer.hpp"
#include <algorithm>
#include <numeric>
#include <chrono>
#include <iostream>
#include <cmath>
#include <cassert>
#include <unordered_set>
#include <set>

namespace sf {

double AnalyticalPlacer::compute_hpwl() const {
    return pd_.total_wirelength();
}

// ── B2B net model (StarModelToClique decomposition) ──────────────────
std::vector<AnalyticalPlacer::B2BEdge> AnalyticalPlacer::build_b2b_model() const {
    // Bound-to-bound net model: for each net, connect the leftmost cell to
    // the rightmost cell, top to bottom, etc. Weight = 2/(p-1) where p = degree.
    // More accurate than clique model for high-fanout nets.
    std::vector<B2BEdge> edges;
    int n = (int)pd_.cells.size();
    for (auto& net : pd_.nets) {
        if (net.cell_ids.size() < 2) continue;
        double nw = 1.0;
        auto wit = net_weights_.find(net.id);
        if (wit != net_weights_.end()) nw = wit->second;
        double p = (double)net.cell_ids.size();
        double base_w = 2.0 / (p - 1.0) * nw * wl_weight_;

        // Find bounding box extremes
        int min_x_id = -1, max_x_id = -1, min_y_id = -1, max_y_id = -1;
        double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
        for (int cid : net.cell_ids) {
            if (cid < 0 || cid >= n) continue;
            double cx = x_[cid], cy = y_[cid];
            if (cx < min_x) { min_x = cx; min_x_id = cid; }
            if (cx > max_x) { max_x = cx; max_x_id = cid; }
            if (cy < min_y) { min_y = cy; min_y_id = cid; }
            if (cy > max_y) { max_y = cy; max_y_id = cid; }
        }
        // B2B edges: connect each cell to the min/max bounds
        double eps = 1e-6;
        for (int cid : net.cell_ids) {
            if (cid < 0 || cid >= n) continue;
            if (cid != min_x_id && min_x_id >= 0) {
                double dx = std::max(eps, std::abs(x_[cid] - x_[min_x_id]));
                edges.push_back({cid, min_x_id, base_w / dx, 0});
            }
            if (cid != max_x_id && max_x_id >= 0 && max_x_id != min_x_id) {
                double dx = std::max(eps, std::abs(x_[cid] - x_[max_x_id]));
                edges.push_back({cid, max_x_id, base_w / dx, 0});
            }
            if (cid != min_y_id && min_y_id >= 0) {
                double dy = std::max(eps, std::abs(y_[cid] - y_[min_y_id]));
                edges.push_back({cid, min_y_id, 0, base_w / dy});
            }
            if (cid != max_y_id && max_y_id >= 0 && max_y_id != min_y_id) {
                double dy = std::max(eps, std::abs(y_[cid] - y_[max_y_id]));
                edges.push_back({cid, max_y_id, 0, base_w / dy});
            }
        }
    }
    return edges;
}

// ── Conjugate Gradient solver ────────────────────────────────────────
void AnalyticalPlacer::solve_quadratic_cg() {
    int n = (int)pd_.cells.size();
    if (n == 0) return;

    // Initialize positions
    x_.resize(n); y_.resize(n);
    for (int i = 0; i < n; i++) {
        if (pd_.cells[i].placed && fixed_cells_.count(i)) {
            x_[i] = pd_.cells[i].position.x;
            y_[i] = pd_.cells[i].position.y;
        } else {
            double fx = (double)(i % 10) / 10.0;
            double fy = (double)((i / 10) % 10) / 10.0;
            x_[i] = pd_.die_area.x0 + fx * pd_.die_area.width() * 0.8 + pd_.die_area.width() * 0.1;
            y_[i] = pd_.die_area.y0 + fy * pd_.die_area.height() * 0.8 + pd_.die_area.height() * 0.1;
        }
    }

    if (anchor_x_.empty()) {
        anchor_x_.assign(n, pd_.die_area.center().x);
        anchor_y_.assign(n, pd_.die_area.center().y);
        anchor_w_.assign(n, 0.01);
    }

    // CG iterations: solve Ax = b where A is the connectivity Laplacian + anchors
    // Using a simple CG implementation for the sparse system
    auto edges = build_b2b_model();

    // Build diagonal and off-diagonal sums per cell
    std::vector<double> diag_x(n, 0), diag_y(n, 0);
    std::vector<double> rhs_x(n, 0), rhs_y(n, 0);

    for (auto& e : edges) {
        diag_x[e.i] += e.wx; diag_x[e.j] += e.wx;
        diag_y[e.i] += e.wy; diag_y[e.j] += e.wy;
    }
    for (int i = 0; i < n; i++) {
        diag_x[i] += anchor_w_[i];
        diag_y[i] += anchor_w_[i];
        rhs_x[i] += anchor_w_[i] * anchor_x_[i];
        rhs_y[i] += anchor_w_[i] * anchor_y_[i];
    }

    // Compute Ax for current x (matrix-vector product)
    auto matvec = [&](const std::vector<double>& vx, const std::vector<double>& vy,
                      std::vector<double>& out_x, std::vector<double>& out_y) {
        std::fill(out_x.begin(), out_x.end(), 0);
        std::fill(out_y.begin(), out_y.end(), 0);
        for (auto& e : edges) {
            out_x[e.i] += e.wx * (vx[e.i] - vx[e.j]);
            out_x[e.j] += e.wx * (vx[e.j] - vx[e.i]);
            out_y[e.i] += e.wy * (vy[e.i] - vy[e.j]);
            out_y[e.j] += e.wy * (vy[e.j] - vy[e.i]);
        }
        for (int i = 0; i < n; i++) {
            out_x[i] += anchor_w_[i] * vx[i];
            out_y[i] += anchor_w_[i] * vy[i];
        }
    };

    // CG solve: r = b - Ax, p = r, iterate
    std::vector<double> rx(n), ry(n), px(n), py(n), apx(n), apy(n);
    std::vector<double> ax(n), ay(n);
    matvec(x_, y_, ax, ay);
    for (int i = 0; i < n; i++) { rx[i] = rhs_x[i] - ax[i]; ry[i] = rhs_y[i] - ay[i]; }
    px = rx; py = ry;
    double rsq = 0;
    for (int i = 0; i < n; i++) rsq += rx[i]*rx[i] + ry[i]*ry[i];

    int max_cg = std::min(200, n * 2);
    for (int iter = 0; iter < max_cg && rsq > 1e-8; iter++) {
        matvec(px, py, apx, apy);
        double pap = 0;
        for (int i = 0; i < n; i++) pap += px[i]*apx[i] + py[i]*apy[i];
        if (pap < 1e-15) break;
        double alpha = rsq / pap;
        double new_rsq = 0;
        for (int i = 0; i < n; i++) {
            if (fixed_cells_.count(i)) continue;
            x_[i] += alpha * px[i];
            y_[i] += alpha * py[i];
            rx[i] -= alpha * apx[i];
            ry[i] -= alpha * apy[i];
            new_rsq += rx[i]*rx[i] + ry[i]*ry[i];
        }
        if (new_rsq < 1e-8) break;
        double beta = new_rsq / rsq;
        for (int i = 0; i < n; i++) {
            px[i] = rx[i] + beta * px[i];
            py[i] = ry[i] + beta * py[i];
        }
        rsq = new_rsq;
    }

    // Clamp to die area
    for (int i = 0; i < n; i++) {
        x_[i] = std::clamp(x_[i], pd_.die_area.x0, pd_.die_area.x1 - pd_.cells[i].width);
        y_[i] = std::clamp(y_[i], pd_.die_area.y0, pd_.die_area.y1 - pd_.cells[i].height);
        pd_.cells[i].position.x = x_[i];
        pd_.cells[i].position.y = y_[i];
        pd_.cells[i].placed = true;
    }
}

// ── Density bin grid ─────────────────────────────────────────────────
void AnalyticalPlacer::build_density_grid(int nx, int ny) {
    bin_nx_ = nx; bin_ny_ = ny;
    density_grid_.resize(ny);
    double bw = pd_.die_area.width() / nx;
    double bh = pd_.die_area.height() / ny;
    for (int r = 0; r < ny; r++) {
        density_grid_[r].resize(nx);
        for (int c = 0; c < nx; c++) {
            auto& b = density_grid_[r][c];
            b.x0 = pd_.die_area.x0 + c * bw;
            b.y0 = pd_.die_area.y0 + r * bh;
            b.x1 = b.x0 + bw;
            b.y1 = b.y0 + bh;
            b.supply = bw * bh * target_density_;
            b.demand = 0;
            b.target_x = (b.x0 + b.x1) / 2;
            b.target_y = (b.y0 + b.y1) / 2;
        }
    }
}

void AnalyticalPlacer::compute_density() {
    if (bin_nx_ == 0 || bin_ny_ == 0) return;
    double bw = pd_.die_area.width() / bin_nx_;
    double bh = pd_.die_area.height() / bin_ny_;
    for (auto& row : density_grid_) for (auto& b : row) b.demand = 0;

    for (auto& cell : pd_.cells) {
        int bc = std::clamp((int)((cell.position.x - pd_.die_area.x0) / bw), 0, bin_nx_ - 1);
        int br = std::clamp((int)((cell.position.y - pd_.die_area.y0) / bh), 0, bin_ny_ - 1);
        density_grid_[br][bc].demand += cell.width * cell.height;
    }
}

void AnalyticalPlacer::spread_bins_x() {
    // For each row of bins, spread overfilled bins by shifting cells
    double bw = pd_.die_area.width() / bin_nx_;
    double bh = pd_.die_area.height() / bin_ny_;
    for (int r = 0; r < bin_ny_; r++) {
        // Prefix-sum based spreading: compute target x for each bin
        double total_demand = 0;
        for (int c = 0; c < bin_nx_; c++) total_demand += density_grid_[r][c].demand;
        if (total_demand < 1e-10) continue;

        double running = 0;
        for (int c = 0; c < bin_nx_; c++) {
            double frac_before = running / std::max(total_demand, 1e-10);
            running += density_grid_[r][c].demand;
            double frac_after = running / std::max(total_demand, 1e-10);
            double new_x0 = pd_.die_area.x0 + frac_before * pd_.die_area.width();
            double new_x1 = pd_.die_area.x0 + frac_after * pd_.die_area.width();
            density_grid_[r][c].target_x = (new_x0 + new_x1) / 2;
        }

        // Move cells towards target bin centers
        for (size_t i = 0; i < pd_.cells.size(); i++) {
            if (fixed_cells_.count(i)) continue;
            int bc = std::clamp((int)((x_[i] - pd_.die_area.x0) / bw), 0, bin_nx_ - 1);
            int br = std::clamp((int)((y_[i] - pd_.die_area.y0) / bh), 0, bin_ny_ - 1);
            if (br == r) {
                double shift = density_grid_[r][bc].target_x - (density_grid_[r][bc].x0 + density_grid_[r][bc].x1) / 2;
                x_[i] += shift * 0.3; // damped to avoid oscillation
            }
        }
    }
}

void AnalyticalPlacer::spread_bins_y() {
    double bw = pd_.die_area.width() / bin_nx_;
    double bh = pd_.die_area.height() / bin_ny_;
    for (int c = 0; c < bin_nx_; c++) {
        double total_demand = 0;
        for (int r = 0; r < bin_ny_; r++) total_demand += density_grid_[r][c].demand;
        if (total_demand < 1e-10) continue;

        double running = 0;
        for (int r = 0; r < bin_ny_; r++) {
            double frac_before = running / std::max(total_demand, 1e-10);
            running += density_grid_[r][c].demand;
            double frac_after = running / std::max(total_demand, 1e-10);
            double new_y0 = pd_.die_area.y0 + frac_before * pd_.die_area.height();
            double new_y1 = pd_.die_area.y0 + frac_after * pd_.die_area.height();
            density_grid_[r][c].target_y = (new_y0 + new_y1) / 2;
        }

        for (size_t i = 0; i < pd_.cells.size(); i++) {
            if (fixed_cells_.count(i)) continue;
            int bc = std::clamp((int)((x_[i] - pd_.die_area.x0) / bw), 0, bin_nx_ - 1);
            int br = std::clamp((int)((y_[i] - pd_.die_area.y0) / bh), 0, bin_ny_ - 1);
            if (bc == c) {
                double shift = density_grid_[br][c].target_y - (density_grid_[br][c].y0 + density_grid_[br][c].y1) / 2;
                y_[i] += shift * 0.3;
            }
        }
    }
}

// ── Density spreading (look-ahead legalization) ──────────────────────
void AnalyticalPlacer::density_spread() {
    int n = (int)pd_.cells.size();
    if (n == 0) return;
    int grid_sz = std::max(4, (int)std::sqrt(n) / 2);
    build_density_grid(grid_sz, grid_sz);
    compute_density();
    spread_bins_x();
    spread_bins_y();

    // Clamp and write back
    for (int i = 0; i < n; i++) {
        x_[i] = std::clamp(x_[i], pd_.die_area.x0, pd_.die_area.x1 - pd_.cells[i].width);
        y_[i] = std::clamp(y_[i], pd_.die_area.y0, pd_.die_area.y1 - pd_.cells[i].height);
        pd_.cells[i].position.x = x_[i];
        pd_.cells[i].position.y = y_[i];
    }
}

// ── Abacus legalization ──────────────────────────────────────────────
void AnalyticalPlacer::legalize_abacus() {
    int n = (int)pd_.cells.size();
    if (n == 0) return;

    int num_rows = std::max(1, (int)(pd_.die_area.height() / pd_.row_height));

    // For each cell, find the best row that minimizes displacement
    // Then within that row, find the optimal x position
    struct RowState {
        double next_x;
        double y;
        std::vector<int> cell_ids;
    };
    std::vector<RowState> rows(num_rows);
    for (int r = 0; r < num_rows; r++) {
        rows[r].next_x = pd_.die_area.x0;
        rows[r].y = pd_.die_area.y0 + r * pd_.row_height;
    }

    // Sort cells by x position (left to right) for stable Abacus ordering
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return x_[a] < x_[b];
    });

    for (int ci : order) {
        if (fixed_cells_.count(ci)) {
            pd_.cells[ci].position.y = std::max(pd_.die_area.y0,
                std::min(y_[ci], pd_.die_area.y1 - pd_.cells[ci].height));
            continue;
        }

        // Find best row: minimize |y_cell - y_row| + x_displacement
        int best_row = 0;
        double best_cost = 1e18;
        for (int r = 0; r < num_rows; r++) {
            double ry = rows[r].y;
            double opt_x = std::max(rows[r].next_x, x_[ci]);
            if (opt_x + pd_.cells[ci].width > pd_.die_area.x1) {
                // Row too full — heavy penalty
                opt_x = rows[r].next_x;
            }
            double cost = std::abs(y_[ci] - ry) * 2.0 + std::abs(x_[ci] - opt_x);
            if (opt_x + pd_.cells[ci].width > pd_.die_area.x1)
                cost += 1e6; // row overflow penalty
            if (cost < best_cost) { best_cost = cost; best_row = r; }
        }

        // Place in best row
        double final_x = std::max(rows[best_row].next_x, x_[ci]);
        if (final_x + pd_.cells[ci].width > pd_.die_area.x1)
            final_x = rows[best_row].next_x;
        final_x = std::min(final_x, pd_.die_area.x1 - pd_.cells[ci].width);

        pd_.cells[ci].position.x = final_x;
        pd_.cells[ci].position.y = rows[best_row].y;
        x_[ci] = final_x;
        y_[ci] = rows[best_row].y;
        rows[best_row].next_x = final_x + pd_.cells[ci].width;
        rows[best_row].cell_ids.push_back(ci);
    }

    // Abacus refinement: for each row, try local swaps to reduce wirelength
    for (auto& row : rows) {
        if (row.cell_ids.size() < 2) continue;
        // Simple 2-opt: try swapping adjacent cells
        bool improved = true;
        int passes = 0;
        while (improved && passes < 5) {
            improved = false; passes++;
            for (size_t i = 0; i + 1 < row.cell_ids.size(); i++) {
                int a = row.cell_ids[i], b = row.cell_ids[i+1];
                double old_hpwl = compute_hpwl();
                // Try swap
                std::swap(pd_.cells[a].position.x, pd_.cells[b].position.x);
                // Adjust for width differences
                if (pd_.cells[a].width != pd_.cells[b].width) {
                    double left_x = std::min(pd_.cells[a].position.x, pd_.cells[b].position.x);
                    pd_.cells[a].position.x = left_x;
                    pd_.cells[b].position.x = left_x + pd_.cells[a].width;
                }
                double new_hpwl = compute_hpwl();
                if (new_hpwl < old_hpwl) {
                    improved = true;
                    x_[a] = pd_.cells[a].position.x;
                    x_[b] = pd_.cells[b].position.x;
                    std::swap(row.cell_ids[i], row.cell_ids[i+1]);
                } else {
                    // Revert
                    std::swap(pd_.cells[a].position.x, pd_.cells[b].position.x);
                    if (pd_.cells[a].width != pd_.cells[b].width) {
                        double left_x = std::min(pd_.cells[a].position.x, pd_.cells[b].position.x);
                        pd_.cells[a].position.x = left_x;
                        pd_.cells[b].position.x = left_x + pd_.cells[a].width;
                    }
                }
            }
        }
    }
}

// ── Main placement loop ──────────────────────────────────────────────
PlaceResult AnalyticalPlacer::place() {
    auto t0 = std::chrono::high_resolution_clock::now();
    int n = (int)pd_.cells.size();

    // SimPL outer loop: alternate CG solve ↔ density spreading
    // Anchor weight increases each iteration to gradually enforce spreading
    x_.resize(n); y_.resize(n);
    anchor_x_.resize(n); anchor_y_.resize(n);
    anchor_w_.resize(n);
    for (int i = 0; i < n; i++) {
        anchor_x_[i] = pd_.die_area.center().x;
        anchor_y_[i] = pd_.die_area.center().y;
        anchor_w_[i] = 0.01;
    }

    int outer_iters = std::min(30, std::max(8, n / 10));
    for (int oi = 0; oi < outer_iters; oi++) {
        // Phase 1: CG solve with current anchors
        solve_quadratic_cg();

        // Phase 2: Density spreading
        density_spread();

        // Update anchors: set to spread positions, increase weight
        for (int i = 0; i < n; i++) {
            anchor_x_[i] = x_[i];
            anchor_y_[i] = y_[i];
            anchor_w_[i] *= 1.5; // exponential anchor weight growth
        }
    }

    // Phase 3: Final Abacus legalization
    legalize_abacus();

    auto t1 = std::chrono::high_resolution_clock::now();
    PlaceResult r;
    r.hpwl = compute_hpwl();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.iterations = outer_iters;
    r.legal = !pd_.has_overlaps();
    r.message = r.legal ? "Placement legal (SimPL + Abacus)" : "Placement complete (minor overlaps)";
    return r;
}

} // namespace sf
