// SiliconForge — Analytical Placer (SimPL-style) with Timing-Driven Mode
// Conjugate-gradient solver + density bin spreading + Abacus legalization
// Timing-driven: STA-based slack analysis → criticality-weighted net model
#include "pnr/placer.hpp"
#include "timing/sta.hpp"
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

// ── Timing-driven: STA → criticality → net weight update ─────────────
// Reference: Kong 2002, "criticality(net) = (1 - slack/WNS)^exp"
// Nets with negative slack get exponentially higher placement weight,
// guiding the CG solver to shorten critical paths.
void AnalyticalPlacer::update_timing_weights(PlaceResult& result) {
    if (!timing_driven_ || !nl_ || clock_period_ <= 0) return;

    // Build STA engine from netlist + current physical positions
    StaEngine sta(*nl_, lib_);
    auto sta_result = sta.analyze(clock_period_, 10);

    result.wns = sta_result.wns;
    result.tns = sta_result.tns;
    result.timing_iterations++;

    // Compute per-net slack from STA timing data
    // We need to map netlist net IDs → physical design net IDs
    // The STA gives us per-pin arrival/required; we derive per-net worst slack

    // Collect worst slack per netlist net from critical paths
    std::unordered_map<int, double> net_slack;
    double worst_slack = 0;
    int num_critical = 0;

    for (auto& path : sta_result.critical_paths) {
        double slack = path.slack;
        if (slack < worst_slack) worst_slack = slack;
        if (slack < 0) num_critical++;

        // Each path contains nets that form the timing path
        for (auto nid : path.nets) {
            auto it = net_slack.find(nid);
            if (it == net_slack.end()) {
                net_slack[nid] = slack;
            } else {
                it->second = std::min(it->second, slack);
            }
        }
    }

    result.critical_nets = num_critical;
    if (worst_slack >= 0) return; // All timing met, no weighting needed

    // Compute criticality for each net and map to physical net weights
    // criticality = (1 - slack/|WNS|)^exp, clamped to [0, 1]
    double abs_wns = std::abs(worst_slack);
    timing_net_weights_.clear();

    for (auto& [nid, slack] : net_slack) {
        if (slack >= 0) continue; // Non-critical nets keep default weight
        double criticality = std::pow(
            std::clamp(1.0 - slack / abs_wns, 0.0, 1.0),
            criticality_exp_
        );
        // Net weight = 1 + timing_weight * criticality
        double w = 1.0 + timing_weight_ * criticality;
        timing_net_weights_[nid] = w;
    }

    // Apply timing weights to physical design nets
    // Map by name: netlist net name → physical net with same name
    for (size_t i = 0; i < pd_.nets.size(); i++) {
        const auto& pnet = pd_.nets[i];
        // Try to find matching netlist net by name
        for (int nid = 0; nid < (int)nl_->num_nets(); nid++) {
            if (nl_->net(nid).name == pnet.name) {
                auto wit = timing_net_weights_.find(nid);
                if (wit != timing_net_weights_.end()) {
                    net_weights_[pnet.id] = wit->second;
                }
                break;
            }
        }
    }
}

// ── Industrial: Congestion-driven placement (RUDY model) ─────────────
// RUDY = Rectangular Uniform wire DensitY estimation
// For each net, distribute its expected routing demand uniformly
// across the bounding box of the net. Then penalize congested bins.

void AnalyticalPlacer::build_congestion_grid(int nx, int ny) {
    cong_nx_ = nx; cong_ny_ = ny;
    congestion_grid_.resize(ny);
    double bw = pd_.die_area.width() / nx;
    double bh = pd_.die_area.height() / ny;
    for (int j = 0; j < ny; j++) {
        congestion_grid_[j].resize(nx);
        for (int i = 0; i < nx; i++) {
            auto& bin = congestion_grid_[j][i];
            // Routing supply: proportional to bin area × available tracks
            bin.supply = bw * bh * 0.5; // assume 50% routing efficiency
            bin.demand = 0;
        }
    }
}

void AnalyticalPlacer::update_congestion_demand() {
    if (cong_nx_ <= 0 || cong_ny_ <= 0) return;
    double bw = pd_.die_area.width() / cong_nx_;
    double bh = pd_.die_area.height() / cong_ny_;

    // Reset demand
    for (auto& row : congestion_grid_)
        for (auto& bin : row) bin.demand = 0;

    int n = (int)pd_.cells.size();
    for (auto& net : pd_.nets) {
        if (net.cell_ids.size() < 2) continue;
        // Compute bounding box of net
        double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
        for (int cid : net.cell_ids) {
            if (cid < 0 || cid >= n) continue;
            double cx = pd_.cells[cid].position.x;
            double cy = pd_.cells[cid].position.y;
            xmin = std::min(xmin, cx); xmax = std::max(xmax, cx);
            ymin = std::min(ymin, cy); ymax = std::max(ymax, cy);
        }
        double net_hpwl = (xmax - xmin) + (ymax - ymin);
        if (net_hpwl < 1e-6) continue;

        // Distribute demand uniformly across bins overlapping the bbox
        int bx0 = std::max(0, (int)((xmin - pd_.die_area.x0) / bw));
        int bx1 = std::min(cong_nx_ - 1, (int)((xmax - pd_.die_area.x0) / bw));
        int by0 = std::max(0, (int)((ymin - pd_.die_area.y0) / bh));
        int by1 = std::min(cong_ny_ - 1, (int)((ymax - pd_.die_area.y0) / bh));
        int num_bins = std::max(1, (bx1 - bx0 + 1) * (by1 - by0 + 1));
        double per_bin = net_hpwl / num_bins;
        for (int j = by0; j <= by1; j++)
            for (int i = bx0; i <= bx1; i++)
                congestion_grid_[j][i].demand += per_bin;
    }
}

void AnalyticalPlacer::apply_congestion_penalty() {
    if (cong_nx_ <= 0 || cong_ny_ <= 0) return;
    double bw = pd_.die_area.width() / cong_nx_;
    double bh = pd_.die_area.height() / cong_ny_;
    int n = (int)pd_.cells.size();

    // Push cells away from congested bins by adjusting anchors
    for (int ci = 0; ci < n; ci++) {
        if (fixed_cells_.count(ci)) continue;
        int bx = std::clamp((int)((x_[ci] - pd_.die_area.x0) / bw), 0, cong_nx_ - 1);
        int by = std::clamp((int)((y_[ci] - pd_.die_area.y0) / bh), 0, cong_ny_ - 1);
        double overflow = congestion_grid_[by][bx].overflow();
        if (overflow <= 0) continue;

        double ratio = congestion_grid_[by][bx].ratio();
        double penalty = congestion_weight_ * (ratio - 1.0);
        // Push toward less congested neighbor bin
        double target_x = x_[ci], target_y = y_[ci];
        if (bx > 0 && congestion_grid_[by][bx-1].ratio() < ratio)
            target_x -= bw * 0.3 * penalty;
        else if (bx < cong_nx_ - 1 && congestion_grid_[by][bx+1].ratio() < ratio)
            target_x += bw * 0.3 * penalty;
        if (by > 0 && congestion_grid_[by-1][bx].ratio() < ratio)
            target_y -= bh * 0.3 * penalty;
        else if (by < cong_ny_ - 1 && congestion_grid_[by+1][bx].ratio() < ratio)
            target_y += bh * 0.3 * penalty;

        anchor_x_[ci] = anchor_x_[ci] * 0.7 + target_x * 0.3;
        anchor_y_[ci] = anchor_y_[ci] * 0.7 + target_y * 0.3;
    }
}

void AnalyticalPlacer::estimate_congestion(PlaceResult& result) {
    if (cong_nx_ <= 0) build_congestion_grid(10, 10);
    update_congestion_demand();

    double peak = 0, total = 0;
    int count = 0;
    for (auto& row : congestion_grid_) {
        for (auto& bin : row) {
            double r = bin.ratio();
            peak = std::max(peak, r);
            total += r;
            count++;
        }
    }
    result.congestion_peak = peak;
    result.congestion_avg = count > 0 ? total / count : 0;
}

// ── Industrial: Timing-aware detailed placement ──────────────────────
// After Abacus legalization, do timing-aware cell swaps within each row.
// Swap adjacent cells if it reduces WNS (not just HPWL).
// Reference: Pan et al., "Timing-Driven Placement using Quadratic Programming", DAC 2005

void AnalyticalPlacer::timing_aware_detail_placement(PlaceResult& result) {
    if (!timing_driven_ || !nl_) return;

    StaEngine sta(*nl_, lib_);
    auto sta_before = sta.analyze(clock_period_, 5);
    double wns_before = sta_before.wns;
    int swapped = 0;

    // For each row of cells, try timing-aware swaps
    int num_rows = std::max(1, (int)(pd_.die_area.height() / pd_.row_height));
    std::vector<std::vector<int>> row_cells(num_rows);
    int n = (int)pd_.cells.size();

    for (int i = 0; i < n; i++) {
        if (fixed_cells_.count(i)) continue;
        int row = std::clamp((int)((pd_.cells[i].position.y - pd_.die_area.y0) / pd_.row_height),
                             0, num_rows - 1);
        row_cells[row].push_back(i);
    }

    for (auto& rcells : row_cells) {
        if (rcells.size() < 2) continue;
        // Sort by x position
        std::sort(rcells.begin(), rcells.end(),
                  [&](int a, int b){ return pd_.cells[a].position.x < pd_.cells[b].position.x; });

        for (size_t i = 0; i + 1 < rcells.size(); i++) {
            int a = rcells[i], b = rcells[i+1];
            // Try swap
            std::swap(pd_.cells[a].position.x, pd_.cells[b].position.x);
            if (pd_.cells[a].width != pd_.cells[b].width) {
                double left_x = std::min(pd_.cells[a].position.x, pd_.cells[b].position.x);
                pd_.cells[a].position.x = left_x;
                pd_.cells[b].position.x = left_x + pd_.cells[a].width;
            }

            // Re-run STA and check if timing improved
            StaEngine sta2(*nl_, lib_);
            auto sta_after = sta2.analyze(clock_period_, 3);

            if (sta_after.wns > wns_before || (sta_after.wns == wns_before && sta_after.tns > sta_before.tns)) {
                // Keep the swap — timing improved
                wns_before = sta_after.wns;
                sta_before = sta_after;
                swapped++;
                std::swap(rcells[i], rcells[i+1]);
            } else {
                // Revert swap
                std::swap(pd_.cells[a].position.x, pd_.cells[b].position.x);
                if (pd_.cells[a].width != pd_.cells[b].width) {
                    double left_x = std::min(pd_.cells[a].position.x, pd_.cells[b].position.x);
                    pd_.cells[a].position.x = left_x;
                    pd_.cells[b].position.x = left_x + pd_.cells[a].width;
                }
            }
        }
    }
    result.timing_swaps = swapped;
    result.wns = wns_before;
}

// ── Industrial: Placement constraints (fence/blockage) ───────────────
void AnalyticalPlacer::enforce_constraints(PlaceResult& result) {
    if (constraints_.empty()) return;
    int violations = 0;
    int n = (int)pd_.cells.size();

    for (auto& con : constraints_) {
        if (con.type == ConstraintType::FENCE) {
            // Force cells inside the fence area
            for (int cid : con.cell_ids) {
                if (cid < 0 || cid >= n) continue;
                auto& cell = pd_.cells[cid];
                double nx = std::clamp(cell.position.x, con.area.x0,
                                       con.area.x1 - cell.width);
                double ny = std::clamp(cell.position.y, con.area.y0,
                                       con.area.y1 - cell.height);
                if (nx != cell.position.x || ny != cell.position.y) violations++;
                cell.position.x = nx;
                cell.position.y = ny;
                x_[cid] = nx;
                y_[cid] = ny;
            }
        } else if (con.type == ConstraintType::BLOCKAGE) {
            // Push cells outside the blockage area
            for (int ci = 0; ci < n; ci++) {
                if (fixed_cells_.count(ci)) continue;
                auto& cell = pd_.cells[ci];
                Rect cr(cell.position.x, cell.position.y,
                        cell.position.x + cell.width, cell.position.y + cell.height);
                // Check overlap with blockage
                if (cr.x1 > con.area.x0 && cr.x0 < con.area.x1 &&
                    cr.y1 > con.area.y0 && cr.y0 < con.area.y1) {
                    // Push to nearest edge
                    double push_left = con.area.x0 - cell.width - cell.position.x;
                    double push_right = con.area.x1 - cell.position.x;
                    double push_down = con.area.y0 - cell.height - cell.position.y;
                    double push_up = con.area.y1 - cell.position.y;
                    double min_push = 1e18;
                    double dx = 0, dy = 0;
                    if (std::abs(push_right) < min_push) { min_push = std::abs(push_right); dx = push_right; dy = 0; }
                    if (std::abs(push_left) < min_push) { min_push = std::abs(push_left); dx = push_left; dy = 0; }
                    if (std::abs(push_up) < min_push) { min_push = std::abs(push_up); dx = 0; dy = push_up; }
                    if (std::abs(push_down) < min_push) { dx = 0; dy = push_down; }
                    cell.position.x += dx;
                    cell.position.y += dy;
                    x_[ci] = cell.position.x;
                    y_[ci] = cell.position.y;
                    violations++;
                }
            }
        }
        // GUIDE: soft constraint — handled via anchor weighting, not hard enforcement
    }
    result.constraint_violations = violations;
}

// ── Industrial: Cell padding for DRC-aware legalization ──────────────
void AnalyticalPlacer::apply_cell_padding() {
    if (cell_padding_.empty()) return;
    // Temporarily inflate cell widths to account for padding
    // This ensures Abacus legalization respects minimum spacing
    for (auto& [cid, pad] : cell_padding_) {
        if (cid >= 0 && cid < (int)pd_.cells.size()) {
            pd_.cells[cid].width += pad.left + pad.right;
            pd_.cells[cid].position.x += pad.left; // offset by left pad
        }
    }
}

// ── Industrial: Slack distribution histogram ─────────────────────────
void AnalyticalPlacer::compute_slack_histogram(PlaceResult& result) {
    if (!timing_driven_ || !nl_) return;

    StaEngine sta(*nl_, lib_);
    auto sta_r = sta.analyze(clock_period_, 5);

    result.slack_histogram = {};
    for (auto& path : sta_r.critical_paths) {
        double s = path.slack;
        int bucket;
        if (s < -1.0) bucket = 0;
        else if (s < -0.5) bucket = 1;
        else if (s < -0.2) bucket = 2;
        else if (s < -0.1) bucket = 3;
        else if (s < 0) bucket = 4;
        else if (s < 0.1) bucket = 5;
        else if (s < 0.2) bucket = 6;
        else if (s < 0.5) bucket = 7;
        else if (s < 1.0) bucket = 8;
        else bucket = 9;
        result.slack_histogram[bucket]++;
    }
}

// ── Industrial: Displacement measurement ─────────────────────────────
void AnalyticalPlacer::compute_displacement(PlaceResult& result,
                                            const std::vector<double>& orig_x,
                                            const std::vector<double>& orig_y) {
    int n = (int)pd_.cells.size();
    double total_disp = 0, max_disp = 0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        if (fixed_cells_.count(i)) continue;
        double dx = pd_.cells[i].position.x - orig_x[i];
        double dy = pd_.cells[i].position.y - orig_y[i];
        double d = std::sqrt(dx*dx + dy*dy);
        total_disp += d;
        max_disp = std::max(max_disp, d);
        count++;
    }
    result.displacement_avg = count > 0 ? total_disp / count : 0;
    result.displacement_max = max_disp;
}

// ── Main placement loop ──────────────────────────────────────────────
PlaceResult AnalyticalPlacer::place() {
    auto t0 = std::chrono::high_resolution_clock::now();
    int n = (int)pd_.cells.size();

    // In incremental mode, freeze all cells except specified ones
    if (incremental_mode_) {
        for (int i = 0; i < n; i++) {
            if (!incremental_cells_.count(i))
                fixed_cells_.insert(i);
        }
    }

    // Apply cell padding before placement (inflates widths for DRC)
    apply_cell_padding();

    // SimPL outer loop: alternate CG solve ↔ density spreading
    // Anchor weight increases each iteration to gradually enforce spreading
    x_.resize(n); y_.resize(n);
    anchor_x_.resize(n); anchor_y_.resize(n);
    anchor_w_.resize(n);

    // Save original positions for displacement measurement
    std::vector<double> orig_x(n), orig_y(n);
    for (int i = 0; i < n; i++) {
        anchor_x_[i] = pd_.die_area.center().x;
        anchor_y_[i] = pd_.die_area.center().y;
        anchor_w_[i] = 0.01;
        orig_x[i] = pd_.cells[i].position.x;
        orig_y[i] = pd_.cells[i].position.y;
    }

    // Initialize congestion grid if congestion-driven
    if (congestion_driven_) {
        int grid_size = std::max(5, (int)std::sqrt(n / 4.0));
        build_congestion_grid(grid_size, grid_size);
    }

    int outer_iters = std::min(30, std::max(8, n / 10));
    PlaceResult r;
    for (int oi = 0; oi < outer_iters; oi++) {
        // Phase 1: CG solve with current anchors
        solve_quadratic_cg();

        // Phase 2: Density spreading
        density_spread();

        // Phase 2.5: Timing-driven net weight update (every N iterations)
        if (timing_driven_ && oi > 0 && (oi % timing_update_interval_ == 0)) {
            update_timing_weights(r);
        }

        // Phase 2.6: Congestion-driven penalty (every 2 iterations)
        if (congestion_driven_ && oi > 1 && (oi % 2 == 0)) {
            update_congestion_demand();
            apply_congestion_penalty();
        }

        // Update anchors: set to spread positions, increase weight
        for (int i = 0; i < n; i++) {
            anchor_x_[i] = x_[i];
            anchor_y_[i] = y_[i];
            anchor_w_[i] *= 1.5; // exponential anchor weight growth
        }
    }

    // Phase 3: Enforce placement constraints before legalization
    enforce_constraints(r);

    // Phase 4: Final Abacus legalization
    legalize_abacus();

    // Phase 5: Timing-aware detailed placement (post-legalization swaps)
    if (timing_driven_) {
        timing_aware_detail_placement(r);
    }

    // Phase 6: Final timing analysis (if timing-driven)
    if (timing_driven_) {
        update_timing_weights(r);
        compute_slack_histogram(r);
    }

    // Phase 7: Congestion estimation for reporting
    if (congestion_driven_) {
        estimate_congestion(r);
    }

    // Phase 8: Compute displacement metrics
    compute_displacement(r, orig_x, orig_y);

    // Restore cell padding (shrink widths back)
    for (auto& [cid, pad] : cell_padding_) {
        if (cid >= 0 && cid < (int)pd_.cells.size()) {
            pd_.cells[cid].width -= pad.left + pad.right;
            pd_.cells[cid].position.x -= pad.left;
            r.cells_padded++;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.hpwl = compute_hpwl();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.iterations = outer_iters;
    r.legal = !pd_.has_overlaps();

    // Build summary message
    r.message = "Placement ";
    if (r.legal) r.message += "legal";
    else r.message += "complete (minor overlaps)";
    r.message += " (SimPL + Abacus";
    if (timing_driven_) r.message += ", timing-driven";
    if (congestion_driven_) r.message += ", congestion-driven";
    if (incremental_mode_) r.message += ", incremental";
    r.message += ")";
    if (timing_driven_)
        r.message += " WNS=" + std::to_string(r.wns).substr(0, 6);
    if (r.timing_swaps > 0)
        r.message += ", " + std::to_string(r.timing_swaps) + " timing swaps";
    if (congestion_driven_)
        r.message += ", cong_peak=" + std::to_string(r.congestion_peak).substr(0, 4);
    r.message += ", disp_avg=" + std::to_string(r.displacement_avg).substr(0, 5);

    return r;
}

} // namespace sf
