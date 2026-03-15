// SiliconForge — Analytical Placer (SimPL-style) with Timing-Driven Mode
// Conjugate-gradient solver + density bin spreading + Abacus legalization
// Timing-driven: STA-based slack analysis → criticality-weighted net model
#include "pnr/placer.hpp"
#include "pnr/spectral_density.hpp"
#include "pnr/gpu_density.hpp"
#include "timing/sta.hpp"
#include "ml/congestion_model.hpp"
#include <algorithm>
#include <numeric>
#include <chrono>
#include <iostream>
#include <cmath>
#include <cassert>
#include <unordered_set>
#include <set>
#ifdef SF_HAS_OPENMP
#include <omp.h>
#endif

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

// Phase 2: Build per-cell adjacency list from edge list
// Converts edge-centric representation to cell-centric for parallel matvec.
// Each cell gets a list of {neighbor, wx, wy} so the matvec inner loop
// has no write conflicts and is embarrassingly parallel.
void AnalyticalPlacer::build_cell_adjacency(const std::vector<B2BEdge>& edges, int n) {
    cell_adj_.assign(n, {});
    for (auto& e : edges) {
        if (e.i >= 0 && e.i < n && e.j >= 0 && e.j < n) {
            cell_adj_[e.i].push_back({e.j, e.wx, e.wy});
            cell_adj_[e.j].push_back({e.i, e.wx, e.wy});
        }
    }
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

    // Phase 2: Build per-cell adjacency list for parallel matvec
    build_cell_adjacency(edges, n);

    // Build diagonal and off-diagonal sums per cell
    std::vector<double> diag_x(n, 0), diag_y(n, 0);
    std::vector<double> rhs_x(n, 0), rhs_y(n, 0);

    for (auto& e : edges) {
        diag_x[e.i] += e.wx; diag_x[e.j] += e.wx;
        diag_y[e.i] += e.wy; diag_y[e.j] += e.wy;
    }
#ifdef SF_HAS_OPENMP
    #pragma omp parallel for schedule(static) if(n > 500)
#endif
    for (int i = 0; i < n; i++) {
        diag_x[i] += anchor_w_[i];
        diag_y[i] += anchor_w_[i];
        rhs_x[i] += anchor_w_[i] * anchor_x_[i];
        rhs_y[i] += anchor_w_[i] * anchor_y_[i];
    }

    // Phase 2: Parallel matvec using per-cell adjacency list
    // Each cell independently sums its neighbor contributions — no write conflicts
    auto matvec = [&](const std::vector<double>& vx, const std::vector<double>& vy,
                      std::vector<double>& out_x, std::vector<double>& out_y) {
#ifdef SF_HAS_OPENMP
        #pragma omp parallel for schedule(static) if(n > 500)
#endif
        for (int i = 0; i < n; i++) {
            double sx = 0, sy = 0;
            for (auto& adj : cell_adj_[i]) {
                sx += adj.wx * (vx[i] - vx[adj.nbr]);
                sy += adj.wy * (vy[i] - vy[adj.nbr]);
            }
            // Anchor term
            out_x[i] = sx + anchor_w_[i] * vx[i];
            out_y[i] = sy + anchor_w_[i] * vy[i];
        }
    };

    // CG solve: r = b - Ax, p = r, iterate
    std::vector<double> rx(n), ry(n), px(n), py(n), apx(n), apy(n);
    std::vector<double> ax(n), ay(n);
    matvec(x_, y_, ax, ay);
#ifdef SF_HAS_OPENMP
    #pragma omp parallel for schedule(static) if(n > 500)
#endif
    for (int i = 0; i < n; i++) { rx[i] = rhs_x[i] - ax[i]; ry[i] = rhs_y[i] - ay[i]; }
    px = rx; py = ry;
    double rsq = 0;
#ifdef SF_HAS_OPENMP
    #pragma omp parallel for schedule(static) reduction(+:rsq) if(n > 500)
#endif
    for (int i = 0; i < n; i++) rsq += rx[i]*rx[i] + ry[i]*ry[i];

    int max_cg = std::min(200, n * 2);
    for (int iter = 0; iter < max_cg && rsq > 1e-8; iter++) {
        matvec(px, py, apx, apy);
        double pap = 0;
#ifdef SF_HAS_OPENMP
        #pragma omp parallel for schedule(static) reduction(+:pap) if(n > 500)
#endif
        for (int i = 0; i < n; i++) pap += px[i]*apx[i] + py[i]*apy[i];
        if (pap < 1e-15) break;
        double alpha = rsq / pap;
        double new_rsq = 0;
#ifdef SF_HAS_OPENMP
        #pragma omp parallel for schedule(static) reduction(+:new_rsq) if(n > 500)
#endif
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
#ifdef SF_HAS_OPENMP
        #pragma omp parallel for schedule(static) if(n > 500)
#endif
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

    // Cell-to-bin accumulation — sequential due to bin write conflicts
    // (parallel would require atomic adds or thread-local grids)
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

    // === Macro placement: place macros at die boundaries first ===
    {
        std::vector<int> macro_ids;
        for (int i = 0; i < n; i++) {
            if (pd_.cells[i].is_macro && !pd_.cells[i].placed) {
                macro_ids.push_back(i);
            }
        }
        if (!macro_ids.empty()) {
            // Sort macros by area (largest first)
            std::sort(macro_ids.begin(), macro_ids.end(), [&](int a, int b) {
                return pd_.cells[a].width * pd_.cells[a].height >
                       pd_.cells[b].width * pd_.cells[b].height;
            });
            // Place macros at corners and edges
            double dx = pd_.die_area.x0;
            double dy = pd_.die_area.y0;
            double die_w = pd_.die_area.width();
            double die_h = pd_.die_area.height();
            // Corner positions: TL, TR, BL, BR, then edges
            struct Slot { double x, y; bool used; };
            std::vector<Slot> slots = {
                {dx, dy, false},
                {dx + die_w, dy, false},     // will adjust
                {dx, dy + die_h, false},      // will adjust
                {dx + die_w, dy + die_h, false} // will adjust
            };
            for (size_t mi = 0; mi < macro_ids.size(); mi++) {
                int cid = macro_ids[mi];
                auto& cell = pd_.cells[cid];
                double mx, my;
                if (mi == 0)      { mx = dx; my = dy; }
                else if (mi == 1) { mx = dx + die_w - cell.width; my = dy; }
                else if (mi == 2) { mx = dx; my = dy + die_h - cell.height; }
                else if (mi == 3) { mx = dx + die_w - cell.width; my = dy + die_h - cell.height; }
                else {
                    // Place along top/bottom edges
                    mx = dx + ((mi - 4) % 2 == 0 ? (mi - 4) / 2 * 30.0 : die_w - cell.width - (mi - 4) / 2 * 30.0);
                    my = (mi % 2 == 0) ? dy : dy + die_h - cell.height;
                    mx = std::max(dx, std::min(mx, dx + die_w - cell.width));
                    my = std::max(dy, std::min(my, dy + die_h - cell.height));
                }
                cell.position = {mx, my};
                cell.placed = true;
                fixed_cells_.insert(cid);
                // Add blockage for macro halo
                if (cell.halo > 0) {
                    Rect halo_rect(mx - cell.halo, my - cell.halo,
                                   mx + cell.width + cell.halo,
                                   my + cell.height + cell.halo);
                    constraints_.push_back({ConstraintType::BLOCKAGE,
                                           "macro_halo_" + cell.name, halo_rect, {}});
                }
            }
        }
    }

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

// ── Congestion-driven refinement using ML predictor ──────────────────
void AnalyticalPlacer::congestion_driven_refine(const Netlist& nl) {
    int n = (int)pd_.cells.size();
    if (n == 0) return;

    // Ensure position vectors are in sync
    x_.resize(n);
    y_.resize(n);
    for (int i = 0; i < n; i++) {
        x_[i] = pd_.cells[i].position.x;
        y_[i] = pd_.cells[i].position.y;
    }

    constexpr int MAX_REFINE_ITERS = 3;
    constexpr double HOTSPOT_THRESHOLD = 0.80;

    for (int iter = 0; iter < MAX_REFINE_ITERS; iter++) {
        // Run ML congestion predictor
        int grid_sz = std::max(5, (int)std::sqrt(n / 4.0));
        MlCongestionPredictor predictor(pd_, grid_sz, grid_sz);
        auto cong = predictor.predict();

        if (cong.peak_congestion < HOTSPOT_THRESHOLD) break; // No hotspots

        // Identify hotspot tiles (>80% congestion)
        double bw = pd_.die_area.width() / grid_sz;
        double bh = pd_.die_area.height() / grid_sz;

        for (int gy = 0; gy < grid_sz && gy < (int)cong.heatmap_grid.size(); gy++) {
            for (int gx = 0; gx < grid_sz && gx < (int)cong.heatmap_grid[gy].size(); gx++) {
                double c = cong.heatmap_grid[gy][gx];
                if (c <= HOTSPOT_THRESHOLD) continue;

                // Spread cells in this hotspot region toward less congested neighbors
                double bin_x0 = pd_.die_area.x0 + gx * bw;
                double bin_y0 = pd_.die_area.y0 + gy * bh;
                double bin_x1 = bin_x0 + bw;
                double bin_y1 = bin_y0 + bh;
                double bin_cx = (bin_x0 + bin_x1) / 2.0;
                double bin_cy = (bin_y0 + bin_y1) / 2.0;

                // Find least congested neighbor
                double min_cong = c;
                double target_x = bin_cx, target_y = bin_cy;
                auto get_cong = [&](int r, int cc) -> double {
                    if (r < 0 || r >= (int)cong.heatmap_grid.size()) return 1.0;
                    if (cc < 0 || cc >= (int)cong.heatmap_grid[r].size()) return 1.0;
                    return cong.heatmap_grid[r][cc];
                };

                // Check 4 neighbors
                struct Dir { int dr, dc; };
                Dir dirs[] = {{0,-1},{0,1},{-1,0},{1,0}};
                for (auto& d : dirs) {
                    int nr = gy + d.dr, nc = gx + d.dc;
                    double nc_val = get_cong(nr, nc);
                    if (nc_val < min_cong) {
                        min_cong = nc_val;
                        target_x = pd_.die_area.x0 + (nc + 0.5) * bw;
                        target_y = pd_.die_area.y0 + (nr + 0.5) * bh;
                    }
                }

                if (min_cong >= c) continue; // No better neighbor

                // Move cells in this bin toward the target
                double spread_factor = 0.3 * (c - HOTSPOT_THRESHOLD);
                for (int ci = 0; ci < n; ci++) {
                    if (fixed_cells_.count(ci)) continue;
                    double cx = x_[ci], cy = y_[ci];
                    if (cx >= bin_x0 && cx < bin_x1 && cy >= bin_y0 && cy < bin_y1) {
                        x_[ci] += (target_x - cx) * spread_factor;
                        y_[ci] += (target_y - cy) * spread_factor;
                        // Clamp to die
                        x_[ci] = std::clamp(x_[ci], pd_.die_area.x0,
                                            pd_.die_area.x1 - pd_.cells[ci].width);
                        y_[ci] = std::clamp(y_[ci], pd_.die_area.y0,
                                            pd_.die_area.y1 - pd_.cells[ci].height);
                        pd_.cells[ci].position.x = x_[ci];
                        pd_.cells[ci].position.y = y_[ci];
                    }
                }
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════════
// ePlace-style enhancements: density smoothing, WL gradient, Nesterov
// ══════════════════════════════════════════════════════════════════════

// ── A) compute_density_map: 2D grid of cell density ──────────────────
std::vector<std::vector<double>> AnalyticalPlacer::compute_density_map() {
    int nx = density_cfg_.bin_count_x;
    int ny = density_cfg_.bin_count_y;
    std::vector<std::vector<double>> grid(ny, std::vector<double>(nx, 0.0));

    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    if (die_w < 1e-9 || die_h < 1e-9) return grid;

    double bw = die_w / nx;
    double bh = die_h / ny;
    double bin_area = bw * bh;
    if (bin_area < 1e-12) return grid;

    int n = static_cast<int>(pd_.cells.size());
    for (int ci = 0; ci < n; ci++) {
        auto& cell = pd_.cells[ci];
        double cx0 = cell.position.x;
        double cy0 = cell.position.y;
        double cx1 = cx0 + cell.width;
        double cy1 = cy0 + cell.height;

        // Determine overlapping bin range
        int bx0 = std::max(0, static_cast<int>((cx0 - pd_.die_area.x0) / bw));
        int bx1 = std::min(nx - 1, static_cast<int>((cx1 - pd_.die_area.x0) / bw));
        int by0 = std::max(0, static_cast<int>((cy0 - pd_.die_area.y0) / bh));
        int by1 = std::min(ny - 1, static_cast<int>((cy1 - pd_.die_area.y0) / bh));

        for (int r = by0; r <= by1; r++) {
            double row_y0 = pd_.die_area.y0 + r * bh;
            double row_y1 = row_y0 + bh;
            double oy = std::max(0.0, std::min(cy1, row_y1) - std::max(cy0, row_y0));
            for (int c = bx0; c <= bx1; c++) {
                double col_x0 = pd_.die_area.x0 + c * bw;
                double col_x1 = col_x0 + bw;
                double ox = std::max(0.0, std::min(cx1, col_x1) - std::max(cx0, col_x0));
                grid[r][c] += (ox * oy) / bin_area;
            }
        }
    }
    return grid;
}

// ── B) smooth_density: Gaussian smoothing (approximation of DCT) ─────
void AnalyticalPlacer::smooth_density(std::vector<std::vector<double>>& density) {
    int ny = static_cast<int>(density.size());
    if (ny == 0) return;
    int nx = static_cast<int>(density[0].size());
    if (nx == 0) return;

    // 3×3 Gaussian kernel (sigma ≈ 0.85)
    static constexpr double k[3][3] = {
        {1.0/16, 2.0/16, 1.0/16},
        {2.0/16, 4.0/16, 2.0/16},
        {1.0/16, 2.0/16, 1.0/16}
    };

    for (int iter = 0; iter < density_cfg_.fft_iterations; iter++) {
        std::vector<std::vector<double>> tmp(ny, std::vector<double>(nx, 0.0));
        for (int r = 0; r < ny; r++) {
            for (int c = 0; c < nx; c++) {
                double sum = 0.0;
                for (int dr = -1; dr <= 1; dr++) {
                    int rr = std::clamp(r + dr, 0, ny - 1);
                    for (int dc = -1; dc <= 1; dc++) {
                        int cc = std::clamp(c + dc, 0, nx - 1);
                        sum += k[dr + 1][dc + 1] * density[rr][cc];
                    }
                }
                tmp[r][c] = sum;
            }
        }
        density = std::move(tmp);
    }
}

// Phase 4: Spectral (DCT) density smoothing — proper frequency-domain filter
void AnalyticalPlacer::smooth_density_spectral(std::vector<std::vector<double>>& density) {
    SpectralDensity sd;
    sd.init(density_cfg_.bin_count_x, density_cfg_.bin_count_y,
            pd_.die_area.width(), pd_.die_area.height(),
            density_cfg_.target_density, density_cfg_.spectral_alpha);
    density = sd.smooth(density);
    // Add back the target so density represents total, not excess
    for (auto& row : density)
        for (auto& v : row)
            v += density_cfg_.target_density;
}

// ── C) compute_density_gradient ──────────────────────────────────────
AnalyticalPlacer::DensityGradient AnalyticalPlacer::compute_density_gradient() {
    int n = static_cast<int>(pd_.cells.size());
    DensityGradient dg;
    dg.grad_x.assign(n, 0.0);
    dg.grad_y.assign(n, 0.0);
    dg.overflow = 0.0;

    auto density = compute_density_map();
    if (density_cfg_.use_spectral) {
        smooth_density_spectral(density);
    } else {
        smooth_density(density);
    }

    int nx = density_cfg_.bin_count_x;
    int ny = density_cfg_.bin_count_y;
    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();
    if (die_w < 1e-9 || die_h < 1e-9) return dg;

    double bw = die_w / nx;
    double bh = die_h / ny;
    double target = density_cfg_.target_density;

    // Compute overflow
    for (int r = 0; r < ny; r++)
        for (int c = 0; c < nx; c++)
            dg.overflow += std::max(0.0, density[r][c] - target);

    // For each cell, compute gradient: direction from high-density to low-density
    for (int ci = 0; ci < n; ci++) {
        if (fixed_cells_.count(ci)) continue;
        double cx = pd_.cells[ci].position.x;
        double cy = pd_.cells[ci].position.y;

        int bc = std::clamp(static_cast<int>((cx - pd_.die_area.x0) / bw), 0, nx - 1);
        int br = std::clamp(static_cast<int>((cy - pd_.die_area.y0) / bh), 0, ny - 1);

        double d_here = density[br][bc];
        if (d_here <= target) continue;

        double excess = d_here - target;

        // Gradient from central differences
        double d_left  = (bc > 0)      ? density[br][bc - 1] : d_here;
        double d_right = (bc < nx - 1) ? density[br][bc + 1] : d_here;
        double d_down  = (br > 0)      ? density[br - 1][bc] : d_here;
        double d_up    = (br < ny - 1) ? density[br + 1][bc] : d_here;

        // Gradient points toward lower density (negative gradient direction)
        dg.grad_x[ci] = (d_right - d_left) * 0.5 * excess;
        dg.grad_y[ci] = (d_up - d_down) * 0.5 * excess;
    }
    return dg;
}

// ── D) compute_wl_gradient: log-sum-exp smooth HPWL gradient ─────────
AnalyticalPlacer::WLGradient AnalyticalPlacer::compute_wl_gradient() {
    int n = static_cast<int>(pd_.cells.size());
    WLGradient wlg;
    wlg.grad_x.assign(n, 0.0);
    wlg.grad_y.assign(n, 0.0);
    wlg.total_wl = 0.0;

    // Log-sum-exp smoothing parameter (smaller = closer to actual HPWL)
    constexpr double gamma = 5.0;
    constexpr double inv_gamma = 1.0 / gamma;

    for (auto& net : pd_.nets) {
        if (net.cell_ids.size() < 2) continue;

        double nw = 1.0;
        auto wit = net_weights_.find(net.id);
        if (wit != net_weights_.end()) nw = wit->second;

        // Log-sum-exp for max: LSE_max = gamma * log(sum(exp(x_i / gamma)))
        // Log-sum-exp for min: LSE_min = -gamma * log(sum(exp(-x_i / gamma)))
        // Gradient of LSE_max w.r.t. x_i = exp(x_i/gamma) / sum(exp(x_j/gamma))
        double max_exp_x = -1e18, max_exp_y = -1e18;
        double min_exp_x = 1e18, min_exp_y = 1e18;

        // Find max for numerical stability
        for (int cid : net.cell_ids) {
            if (cid < 0 || cid >= n) continue;
            double cx = pd_.cells[cid].position.x;
            double cy = pd_.cells[cid].position.y;
            max_exp_x = std::max(max_exp_x, cx * inv_gamma);
            max_exp_y = std::max(max_exp_y, cy * inv_gamma);
            min_exp_x = std::min(min_exp_x, -cx * inv_gamma);
            min_exp_y = std::min(min_exp_y, -cy * inv_gamma);
        }

        double sum_exp_max_x = 0, sum_exp_max_y = 0;
        double sum_exp_min_x = 0, sum_exp_min_y = 0;
        std::vector<double> e_max_x, e_max_y, e_min_x, e_min_y;
        e_max_x.reserve(net.cell_ids.size());
        e_max_y.reserve(net.cell_ids.size());
        e_min_x.reserve(net.cell_ids.size());
        e_min_y.reserve(net.cell_ids.size());

        for (int cid : net.cell_ids) {
            if (cid < 0 || cid >= n) {
                e_max_x.push_back(0); e_max_y.push_back(0);
                e_min_x.push_back(0); e_min_y.push_back(0);
                continue;
            }
            double cx = pd_.cells[cid].position.x;
            double cy = pd_.cells[cid].position.y;

            double emx = std::exp(cx * inv_gamma - max_exp_x);
            double emy = std::exp(cy * inv_gamma - max_exp_y);
            double enx = std::exp(-cx * inv_gamma - min_exp_x);
            double eny = std::exp(-cy * inv_gamma - min_exp_y);

            e_max_x.push_back(emx); sum_exp_max_x += emx;
            e_max_y.push_back(emy); sum_exp_max_y += emy;
            e_min_x.push_back(enx); sum_exp_min_x += enx;
            e_min_y.push_back(eny); sum_exp_min_y += eny;
        }

        // LSE-based HPWL approximation
        double lse_max_x = gamma * (std::log(std::max(sum_exp_max_x, 1e-30)) + max_exp_x);
        double lse_min_x = -gamma * (std::log(std::max(sum_exp_min_x, 1e-30)) + min_exp_x);
        double lse_max_y = gamma * (std::log(std::max(sum_exp_max_y, 1e-30)) + max_exp_y);
        double lse_min_y = -gamma * (std::log(std::max(sum_exp_min_y, 1e-30)) + min_exp_y);

        wlg.total_wl += nw * ((lse_max_x - lse_min_x) + (lse_max_y - lse_min_y));

        // Gradient for each cell
        for (size_t k = 0; k < net.cell_ids.size(); k++) {
            int cid = net.cell_ids[k];
            if (cid < 0 || cid >= n) continue;

            // d(LSE_max)/dx_i = exp(x_i/gamma) / sum(exp(x_j/gamma))
            // d(LSE_min)/dx_i = -( exp(-x_i/gamma) / sum(exp(-x_j/gamma)) ) * (-1)
            //                 =  exp(-x_i/gamma) / sum(exp(-x_j/gamma))
            // Total: d(HPWL)/dx_i = d(max)/dx_i - d(min)/dx_i * (-1) ... but
            // d(HPWL)/dx = d(LSE_max)/dx - d(LSE_min)/dx
            double gx = nw * (e_max_x[k] / std::max(sum_exp_max_x, 1e-30)
                             + e_min_x[k] / std::max(sum_exp_min_x, 1e-30));
            double gy = nw * (e_max_y[k] / std::max(sum_exp_max_y, 1e-30)
                             + e_min_y[k] / std::max(sum_exp_min_y, 1e-30));

            wlg.grad_x[cid] += gx;
            wlg.grad_y[cid] += gy;
        }
    }
    return wlg;
}

// ── E) nesterov_step: Nesterov accelerated gradient ──────────────────
void AnalyticalPlacer::nesterov_step(NesterovState& state,
                                     const WLGradient& wl_grad,
                                     const DensityGradient& den_grad) {
    int n = static_cast<int>(state.x.size());
    double penalty = density_cfg_.smooth_penalty;

    // Momentum coefficient: increases over iterations (capped at 0.9)
    double momentum = std::min(0.9, static_cast<double>(state.iteration) /
                                    (state.iteration + 3.0));

    for (int i = 0; i < n; i++) {
        if (fixed_cells_.count(i)) continue;

        // Combined gradient
        double gx = wl_grad.grad_x[i] + penalty * den_grad.grad_x[i];
        double gy = wl_grad.grad_y[i] + penalty * den_grad.grad_y[i];

        // Nesterov update: x_next = x + momentum*(x - x_prev) - step*grad
        double new_x = state.x[i] + momentum * (state.x[i] - state.x_prev[i])
                      - state.step_size * gx;
        double new_y = state.y[i] + momentum * (state.y[i] - state.y_prev[i])
                      - state.step_size * gy;

        state.x_prev[i] = state.x[i];
        state.y_prev[i] = state.y[i];

        // Clamp to die area
        state.x[i] = std::clamp(new_x, pd_.die_area.x0,
                                pd_.die_area.x1 - pd_.cells[i].width);
        state.y[i] = std::clamp(new_y, pd_.die_area.y0,
                                pd_.die_area.y1 - pd_.cells[i].height);
    }

    // Adaptive step size based on overflow
    if (den_grad.overflow > 0.5 * density_cfg_.bin_count_x * density_cfg_.bin_count_y) {
        state.step_size *= 0.95;  // reduce step if very congested
    } else {
        state.step_size = std::min(state.step_size * 1.02, 0.1);
    }
    state.iteration++;
}

// ── H) place_io_pads: place IO cells on chip boundary ────────────────
void AnalyticalPlacer::place_io_pads() {
    if (io_pads_.empty()) return;
    int n = static_cast<int>(pd_.cells.size());

    double die_w = pd_.die_area.width();
    double die_h = pd_.die_area.height();

    // Sort pads per side by requested position
    std::vector<std::vector<size_t>> side_pads(4); // N, S, E, W
    for (size_t i = 0; i < io_pads_.size(); i++) {
        side_pads[static_cast<int>(io_pads_[i].side)].push_back(i);
    }
    for (auto& sp : side_pads) {
        std::sort(sp.begin(), sp.end(), [&](size_t a, size_t b) {
            return io_pads_[a].position < io_pads_[b].position;
        });
    }

    auto place_pad = [&](const IoPad& pad) {
        int cid = pad.cell_idx;
        if (cid < 0 || cid >= n) return;
        auto& cell = pd_.cells[cid];
        double pos = std::clamp(pad.position, 0.0, 1.0);

        switch (pad.side) {
            case IoPad::Side::NORTH:
                cell.position.x = pd_.die_area.x0 + pos * (die_w - cell.width);
                cell.position.y = pd_.die_area.y1 - cell.height;
                break;
            case IoPad::Side::SOUTH:
                cell.position.x = pd_.die_area.x0 + pos * (die_w - cell.width);
                cell.position.y = pd_.die_area.y0;
                break;
            case IoPad::Side::EAST:
                cell.position.x = pd_.die_area.x1 - cell.width;
                cell.position.y = pd_.die_area.y0 + pos * (die_h - cell.height);
                break;
            case IoPad::Side::WEST:
                cell.position.x = pd_.die_area.x0;
                cell.position.y = pd_.die_area.y0 + pos * (die_h - cell.height);
                break;
        }
        cell.placed = true;
        fixed_cells_.insert(cid);
    };

    // Place pads, spacing evenly per-side if positions collide
    for (int s = 0; s < 4; s++) {
        auto& sp = side_pads[s];
        if (sp.empty()) continue;

        // Check for overlapping positions and re-space if needed
        bool needs_respace = false;
        for (size_t i = 1; i < sp.size(); i++) {
            if (std::abs(io_pads_[sp[i]].position - io_pads_[sp[i-1]].position) < 1e-6) {
                needs_respace = true;
                break;
            }
        }
        if (needs_respace) {
            for (size_t i = 0; i < sp.size(); i++) {
                io_pads_[sp[i]].position = static_cast<double>(i + 1) /
                                           static_cast<double>(sp.size() + 1);
            }
        }
        for (size_t idx : sp) {
            place_pad(io_pads_[idx]);
        }
    }
}

// ── I) legalize_lookahead: enhanced legalization ─────────────────────
void AnalyticalPlacer::legalize_lookahead(std::vector<double>& lx, std::vector<double>& ly) {
    int n = static_cast<int>(pd_.cells.size());
    if (n == 0) return;

    int num_rows = std::max(1, static_cast<int>(pd_.die_area.height() / pd_.row_height));

    struct RowState {
        double next_x;
        double y;
    };
    std::vector<RowState> rows(num_rows);
    for (int r = 0; r < num_rows; r++) {
        rows[r].next_x = pd_.die_area.x0;
        rows[r].y = pd_.die_area.y0 + r * pd_.row_height;
    }

    // Sort cells by x-coordinate
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return lx[a] < lx[b];
    });

    for (int ci : order) {
        if (fixed_cells_.count(ci)) continue;

        double cell_w = pd_.cells[ci].width;
        double cell_h = pd_.cells[ci].height;

        // Determine target row from global placement y
        int target_row = std::clamp(
            static_cast<int>((ly[ci] - pd_.die_area.y0) / pd_.row_height),
            0, num_rows - 1);

        // Try target row and ±2 adjacent rows (look-ahead window)
        int best_row = target_row;
        double best_cost = 1e18;
        double best_x = lx[ci];

        int r_lo = std::max(0, target_row - 2);
        int r_hi = std::min(num_rows - 1, target_row + 2);

        for (int r = r_lo; r <= r_hi; r++) {
            double opt_x = std::max(rows[r].next_x, lx[ci]);
            if (opt_x + cell_w > pd_.die_area.x1) {
                opt_x = rows[r].next_x;
            }
            double cost = std::abs(ly[ci] - rows[r].y) * 2.0
                        + std::abs(lx[ci] - opt_x);
            if (opt_x + cell_w > pd_.die_area.x1)
                cost += 1e6;
            if (cost < best_cost) {
                best_cost = cost;
                best_row = r;
                best_x = opt_x;
            }
        }

        best_x = std::min(best_x, pd_.die_area.x1 - cell_w);
        lx[ci] = best_x;
        ly[ci] = rows[best_row].y;
        pd_.cells[ci].position.x = best_x;
        pd_.cells[ci].position.y = rows[best_row].y;
        pd_.cells[ci].placed = true;
        rows[best_row].next_x = best_x + cell_w;
    }
}

// ── Multi-row legalization ───────────────────────────────────────────
void AnalyticalPlacer::legalize_multi_row(std::vector<double>& lx, std::vector<double>& ly) {
    if (multi_row_.empty()) return;

    int num_rows = std::max(1, static_cast<int>(pd_.die_area.height() / pd_.row_height));

    // Sort multi-row cells by area (largest first) for priority placement
    auto sorted = multi_row_;
    std::sort(sorted.begin(), sorted.end(), [&](const MultiRowCell& a, const MultiRowCell& b) {
        return a.row_span > b.row_span;
    });

    for (auto& mr : sorted) {
        int ci = mr.cell_idx;
        if (ci < 0 || ci >= static_cast<int>(pd_.cells.size())) continue;
        if (fixed_cells_.count(ci)) continue;

        double cell_w = pd_.cells[ci].width;
        int span = mr.row_span;

        // Find best starting row that minimizes displacement
        int target_row = std::clamp(
            static_cast<int>((ly[ci] - pd_.die_area.y0) / pd_.row_height),
            0, num_rows - span);

        int best_row = target_row;
        double best_cost = 1e18;

        int max_start = std::max(0, num_rows - span);
        for (int r = 0; r <= max_start; r++) {
            double ry = pd_.die_area.y0 + r * pd_.row_height;
            double cost = std::abs(ly[ci] - ry) + std::abs(lx[ci] - pd_.cells[ci].position.x) * 0.5;
            if (cost < best_cost) {
                best_cost = cost;
                best_row = r;
            }
        }

        double final_y = pd_.die_area.y0 + best_row * pd_.row_height;
        double final_x = std::clamp(lx[ci], pd_.die_area.x0, pd_.die_area.x1 - cell_w);

        lx[ci] = final_x;
        ly[ci] = final_y;
        pd_.cells[ci].position.x = final_x;
        pd_.cells[ci].position.y = final_y;
        pd_.cells[ci].height = mr.height;
        pd_.cells[ci].placed = true;
    }
}

// ── F) place_eplace: full ePlace-style global placement ──────────────
PlaceResult AnalyticalPlacer::place_eplace() {
    auto t0 = std::chrono::high_resolution_clock::now();
    int n = static_cast<int>(pd_.cells.size());
    PlaceResult r;

    if (n == 0) {
        r.message = "ePlace: no cells";
        return r;
    }

    // Step 1: Place IO pads first (they become fixed)
    if (!io_pads_.empty()) {
        place_io_pads();
    }

    // Step 2: Initialize positions — center placement with small perturbation
    NesterovState ns;
    ns.x.resize(n); ns.y.resize(n);
    ns.x_prev.resize(n); ns.y_prev.resize(n);
    ns.step_size = 0.01;
    ns.iteration = 0;

    double cx_init = pd_.die_area.center().x;
    double cy_init = pd_.die_area.center().y;
    double spread_x = pd_.die_area.width() * 0.3;
    double spread_y = pd_.die_area.height() * 0.3;

    for (int i = 0; i < n; i++) {
        if (fixed_cells_.count(i) && pd_.cells[i].placed) {
            ns.x[i] = pd_.cells[i].position.x;
            ns.y[i] = pd_.cells[i].position.y;
        } else {
            // Deterministic spread based on cell index
            double fx = static_cast<double>(i % 17) / 17.0 - 0.5;
            double fy = static_cast<double>((i / 17) % 13) / 13.0 - 0.5;
            ns.x[i] = std::clamp(cx_init + fx * spread_x,
                                 pd_.die_area.x0, pd_.die_area.x1 - pd_.cells[i].width);
            ns.y[i] = std::clamp(cy_init + fy * spread_y,
                                 pd_.die_area.y0, pd_.die_area.y1 - pd_.cells[i].height);
        }
        ns.x_prev[i] = ns.x[i];
        ns.y_prev[i] = ns.y[i];
        pd_.cells[i].position.x = ns.x[i];
        pd_.cells[i].position.y = ns.y[i];
        pd_.cells[i].placed = true;
    }

    // Also initialize internal x_/y_ for B2B model and other methods
    x_ = ns.x;
    y_ = ns.y;

    // Step 3: Global placement loop (Nesterov iterations)
    constexpr int MAX_ITERS = 200;
    double prev_overflow = 1e18;

    for (int iter = 0; iter < MAX_ITERS; iter++) {
        // Sync positions
        for (int i = 0; i < n; i++) {
            pd_.cells[i].position.x = ns.x[i];
            pd_.cells[i].position.y = ns.y[i];
        }
        x_ = ns.x;
        y_ = ns.y;

        // Compute gradients
        auto wl_grad = compute_wl_gradient();
        auto den_grad = compute_density_gradient();

        // Nesterov step
        nesterov_step(ns, wl_grad, den_grad);

        // Every 10 iterations: check convergence
        if (iter % 10 == 9) {
            double overflow = den_grad.overflow;
            double total_bins = static_cast<double>(density_cfg_.bin_count_x *
                                                    density_cfg_.bin_count_y);
            double overflow_ratio = overflow / std::max(total_bins, 1.0);

            if (overflow_ratio < 0.10) break;  // < 10% overflow

            // Adaptive density penalty
            if (overflow >= prev_overflow) {
                density_cfg_.smooth_penalty *= 1.1;
            }
            prev_overflow = overflow;
        }
    }

    // Step 4: Sync final positions
    for (int i = 0; i < n; i++) {
        pd_.cells[i].position.x = ns.x[i];
        pd_.cells[i].position.y = ns.y[i];
    }
    x_ = ns.x;
    y_ = ns.y;

    // Step 5: Look-ahead legalization
    legalize_lookahead(x_, y_);

    // Step 6: Multi-row legalization if needed
    if (!multi_row_.empty()) {
        legalize_multi_row(x_, y_);
    }

    // Step 7: Detailed placement — local adjacent-cell swaps
    {
        int num_rows = std::max(1, static_cast<int>(pd_.die_area.height() / pd_.row_height));
        std::vector<std::vector<int>> row_cells(num_rows);
        for (int i = 0; i < n; i++) {
            if (fixed_cells_.count(i)) continue;
            int row = std::clamp(
                static_cast<int>((pd_.cells[i].position.y - pd_.die_area.y0) / pd_.row_height),
                0, num_rows - 1);
            row_cells[row].push_back(i);
        }
        for (auto& rcells : row_cells) {
            if (rcells.size() < 2) continue;
            std::sort(rcells.begin(), rcells.end(), [&](int a, int b) {
                return pd_.cells[a].position.x < pd_.cells[b].position.x;
            });
            bool improved = true;
            int passes = 0;
            while (improved && passes < 3) {
                improved = false; passes++;
                for (size_t i = 0; i + 1 < rcells.size(); i++) {
                    int a = rcells[i], b = rcells[i + 1];
                    double old_hpwl = compute_hpwl();
                    std::swap(pd_.cells[a].position.x, pd_.cells[b].position.x);
                    if (pd_.cells[a].width != pd_.cells[b].width) {
                        double lx_pos = std::min(pd_.cells[a].position.x,
                                                 pd_.cells[b].position.x);
                        pd_.cells[a].position.x = lx_pos;
                        pd_.cells[b].position.x = lx_pos + pd_.cells[a].width;
                    }
                    if (compute_hpwl() < old_hpwl) {
                        improved = true;
                        x_[a] = pd_.cells[a].position.x;
                        x_[b] = pd_.cells[b].position.x;
                        std::swap(rcells[i], rcells[i + 1]);
                    } else {
                        std::swap(pd_.cells[a].position.x, pd_.cells[b].position.x);
                        if (pd_.cells[a].width != pd_.cells[b].width) {
                            double lx_pos = std::min(pd_.cells[a].position.x,
                                                     pd_.cells[b].position.x);
                            pd_.cells[a].position.x = lx_pos;
                            pd_.cells[b].position.x = lx_pos + pd_.cells[a].width;
                        }
                    }
                }
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.hpwl = compute_hpwl();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.iterations = ns.iteration;
    r.legal = !pd_.has_overlaps();
    r.message = "Placement ";
    r.message += r.legal ? "legal" : "complete (minor overlaps)";
    r.message += " (ePlace: Nesterov + density smoothing + lookahead legalization)";
    return r;
}

// ── G) incremental_place: ECO placement ──────────────────────────────
PlaceResult AnalyticalPlacer::incremental_place(const std::vector<int>& modified_cells) {
    auto t0 = std::chrono::high_resolution_clock::now();
    int n = static_cast<int>(pd_.cells.size());
    PlaceResult r;

    if (n == 0 || modified_cells.empty()) {
        r.message = "incremental_place: nothing to do";
        return r;
    }

    // Sync internal vectors
    x_.resize(n); y_.resize(n);
    anchor_x_.resize(n); anchor_y_.resize(n);
    anchor_w_.resize(n);

    std::unordered_set<int> mod_set(modified_cells.begin(), modified_cells.end());

    // Fix all cells except modified ones
    auto saved_fixed = fixed_cells_;
    for (int i = 0; i < n; i++) {
        x_[i] = pd_.cells[i].position.x;
        y_[i] = pd_.cells[i].position.y;
        if (!mod_set.count(i)) {
            fixed_cells_.insert(i);
        }
    }

    // Set anchors: for modified cells, anchor to neighbors' centroid
    for (int ci : modified_cells) {
        if (ci < 0 || ci >= n) continue;

        // Find connected cells (neighbors via nets)
        double sum_x = 0, sum_y = 0;
        int count = 0;
        for (auto& net : pd_.nets) {
            bool has_ci = false;
            for (int cid : net.cell_ids) {
                if (cid == ci) { has_ci = true; break; }
            }
            if (!has_ci) continue;
            for (int cid : net.cell_ids) {
                if (cid != ci && cid >= 0 && cid < n) {
                    sum_x += pd_.cells[cid].position.x;
                    sum_y += pd_.cells[cid].position.y;
                    count++;
                }
            }
        }
        if (count > 0) {
            anchor_x_[ci] = sum_x / count;
            anchor_y_[ci] = sum_y / count;
        } else {
            anchor_x_[ci] = pd_.die_area.center().x;
            anchor_y_[ci] = pd_.die_area.center().y;
        }
        anchor_w_[ci] = 0.5;

        // Initialize modified cell at centroid of neighbors
        x_[ci] = anchor_x_[ci];
        y_[ci] = anchor_y_[ci];
    }

    // Set fixed cell anchors
    for (int i = 0; i < n; i++) {
        if (!mod_set.count(i)) {
            anchor_x_[i] = x_[i];
            anchor_y_[i] = y_[i];
            anchor_w_[i] = 100.0;  // very strong anchor for fixed cells
        }
    }

    // Run CG solver (only modifieds move due to fixed_cells_)
    constexpr int ECO_ITERS = 5;
    for (int oi = 0; oi < ECO_ITERS; oi++) {
        solve_quadratic_cg();
        for (int ci : modified_cells) {
            if (ci >= 0 && ci < n) {
                anchor_x_[ci] = x_[ci];
                anchor_y_[ci] = y_[ci];
                anchor_w_[ci] *= 1.3;
            }
        }
    }

    // Legalize only modified cells (simple row snap)
    int num_rows = std::max(1, static_cast<int>(pd_.die_area.height() / pd_.row_height));
    for (int ci : modified_cells) {
        if (ci < 0 || ci >= n) continue;
        int best_row = std::clamp(
            static_cast<int>((y_[ci] - pd_.die_area.y0) / pd_.row_height),
            0, num_rows - 1);
        double ry = pd_.die_area.y0 + best_row * pd_.row_height;
        double rx = std::clamp(x_[ci], pd_.die_area.x0,
                               pd_.die_area.x1 - pd_.cells[ci].width);
        pd_.cells[ci].position.x = rx;
        pd_.cells[ci].position.y = ry;
        x_[ci] = rx;
        y_[ci] = ry;
    }

    // Restore fixed cells
    fixed_cells_ = saved_fixed;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.hpwl = compute_hpwl();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.iterations = ECO_ITERS;
    r.legal = !pd_.has_overlaps();
    r.message = "Incremental placement: " + std::to_string(modified_cells.size()) + " cells re-placed";
    return r;
}

// ── Macro Halo Enforcement (Tier 2) ─────────────────────────────────
// Converts macro halos into blockage regions and enforces them.
// After calling this, standard cells won't overlap macro keepout zones.
void AnalyticalPlacer::enforce_macro_halos() {
    if (macro_halos_.empty()) return;
    int n = (int)pd_.cells.size();

    for (auto& mh : macro_halos_) {
        if (mh.cell_id < 0 || mh.cell_id >= n) continue;
        auto& macro = pd_.cells[mh.cell_id];

        // Create blockage rectangle = macro bbox expanded by halo
        Rect halo_rect(
            macro.position.x - mh.halo_west,
            macro.position.y - mh.halo_south,
            macro.position.x + macro.width + mh.halo_east,
            macro.position.y + macro.height + mh.halo_north
        );

        // Push all non-fixed standard cells out of the halo zone
        for (int ci = 0; ci < n; ci++) {
            if (ci == mh.cell_id) continue;
            if (fixed_cells_.count(ci)) continue;

            auto& cell = pd_.cells[ci];
            Rect cr(cell.position.x, cell.position.y,
                    cell.position.x + cell.width,
                    cell.position.y + cell.height);

            // Check overlap with halo zone
            if (cr.x1 > halo_rect.x0 && cr.x0 < halo_rect.x1 &&
                cr.y1 > halo_rect.y0 && cr.y0 < halo_rect.y1) {
                // Push to nearest halo edge (minimum displacement)
                double push_right = halo_rect.x1 - cell.position.x;
                double push_left  = halo_rect.x0 - cell.width - cell.position.x;
                double push_up    = halo_rect.y1 - cell.position.y;
                double push_down  = halo_rect.y0 - cell.height - cell.position.y;

                double dx = 0, dy = 0;
                double best = 1e18;
                if (std::abs(push_right) < best) { best = std::abs(push_right); dx = push_right; dy = 0; }
                if (std::abs(push_left) < best)  { best = std::abs(push_left);  dx = push_left;  dy = 0; }
                if (std::abs(push_up) < best)    { best = std::abs(push_up);    dx = 0; dy = push_up; }
                if (std::abs(push_down) < best)  { dx = 0; dy = push_down; }

                cell.position.x += dx;
                cell.position.y += dy;
                if (ci < (int)x_.size()) { x_[ci] = cell.position.x; y_[ci] = cell.position.y; }
            }
        }
    }
}

// ── Thermal-Aware Placement (Tier 2) ────────────────────────────────
// Iterative thermal model using Gauss-Seidel heat diffusion.
// Spreads cells away from thermal hotspots to stay under max_temp.
void AnalyticalPlacer::thermal_aware_refine() {
    if (!thermal_cfg_.enabled) return;
    int n = (int)pd_.cells.size();
    if (n == 0) return;

    int gnx = thermal_cfg_.grid_nx;
    int gny = thermal_cfg_.grid_ny;

    // Compute die extents
    double die_w = pd_.die_area.width() > 0 ? pd_.die_area.width() : 1000.0;
    double die_h = pd_.die_area.height() > 0 ? pd_.die_area.height() : 1000.0;
    double bin_w = die_w / gnx;
    double bin_h = die_h / gny;

    // Initialize thermal grid
    thermal_grid_.assign(gnx, std::vector<ThermalBin>(gny));

    // Step 1: Compute power density per bin
    // Estimate power proportional to cell area (simple model)
    double total_area = 0;
    for (int ci = 0; ci < n; ci++) {
        total_area += pd_.cells[ci].width * pd_.cells[ci].height;
    }
    double avg_power_per_area = (total_area > 0) ? 1.0 / total_area : 0;  // normalized

    for (int ci = 0; ci < n; ci++) {
        auto& cell = pd_.cells[ci];
        int bx = std::clamp((int)(cell.position.x / bin_w), 0, gnx - 1);
        int by = std::clamp((int)(cell.position.y / bin_h), 0, gny - 1);
        double cell_power = cell.width * cell.height * avg_power_per_area;
        thermal_grid_[bx][by].power_density += cell_power / (bin_w * bin_h);
    }

    // Step 2: Gauss-Seidel heat diffusion (steady-state T = T_ambient + R_th × P)
    // With neighbor averaging for heat spreading
    for (int iter = 0; iter < 20; iter++) {
        for (int bx = 0; bx < gnx; bx++) {
            for (int by = 0; by < gny; by++) {
                double self_temp = thermal_cfg_.ambient_temp +
                    thermal_cfg_.thermal_resist * thermal_grid_[bx][by].power_density;
                // Average with neighbors (heat conduction)
                double neighbor_sum = 0;
                int nc = 0;
                if (bx > 0)     { neighbor_sum += thermal_grid_[bx-1][by].temperature; nc++; }
                if (bx < gnx-1) { neighbor_sum += thermal_grid_[bx+1][by].temperature; nc++; }
                if (by > 0)     { neighbor_sum += thermal_grid_[bx][by-1].temperature; nc++; }
                if (by < gny-1) { neighbor_sum += thermal_grid_[bx][by+1].temperature; nc++; }
                double avg_neighbor = (nc > 0) ? neighbor_sum / nc : thermal_cfg_.ambient_temp;
                thermal_grid_[bx][by].temperature = 0.5 * self_temp + 0.5 * avg_neighbor;
            }
        }
    }

    // Step 3: Spread cells away from hotspots exceeding max_temp
    double max_temp = thermal_cfg_.max_temp;
    for (int ci = 0; ci < n; ci++) {
        if (fixed_cells_.count(ci)) continue;
        auto& cell = pd_.cells[ci];
        int bx = std::clamp((int)(cell.position.x / bin_w), 0, gnx - 1);
        int by = std::clamp((int)(cell.position.y / bin_h), 0, gny - 1);

        double temp = thermal_grid_[bx][by].temperature;
        if (temp <= max_temp) continue;

        // Find coolest neighbor bin
        double coolest = temp;
        int best_bx = bx, best_by = by;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int nx = bx + dx, ny = by + dy;
                if (nx < 0 || nx >= gnx || ny < 0 || ny >= gny) continue;
                if (thermal_grid_[nx][ny].temperature < coolest) {
                    coolest = thermal_grid_[nx][ny].temperature;
                    best_bx = nx; best_by = ny;
                }
            }
        }

        if (best_bx != bx || best_by != by) {
            double target_x = (best_bx + 0.5) * bin_w;
            double target_y = (best_by + 0.5) * bin_h;
            double sf = thermal_cfg_.spreading_factor;
            cell.position.x += sf * (target_x - cell.position.x);
            cell.position.y += sf * (target_y - cell.position.y);
            if (ci < (int)x_.size()) { x_[ci] = cell.position.x; y_[ci] = cell.position.y; }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tier 3: Multi-Vdd Power Domain Placement
// ═══════════════════════════════════════════════════════════════════════════

void AnalyticalPlacer::assign_cell_to_domain(int cell_id, int domain_id) {
    if (domain_id >= 0 && domain_id < (int)voltage_domains_.size())
        cell_domain_map_[cell_id] = domain_id;
}

int AnalyticalPlacer::find_domain_for_cell(int cell_id) const {
    auto it = cell_domain_map_.find(cell_id);
    return (it != cell_domain_map_.end()) ? it->second : -1;
}

bool AnalyticalPlacer::nets_cross_domains(NetId net) const {
    if (net < 0 || net >= (int)pd_.nets.size()) return false;
    auto& n = pd_.nets[net];
    if (n.cell_ids.size() < 2) return false;
    int first_domain = find_domain_for_cell(n.cell_ids[0]);
    for (size_t i = 1; i < n.cell_ids.size(); i++) {
        int d = find_domain_for_cell(n.cell_ids[i]);
        if (d >= 0 && first_domain >= 0 && d != first_domain)
            return true;
    }
    return false;
}

AnalyticalPlacer::DomainPlaceResult AnalyticalPlacer::place_with_domains() {
    DomainPlaceResult dr;

    // Step 1: Create placement fences from domain regions
    for (size_t di = 0; di < voltage_domains_.size(); di++) {
        auto& vd = voltage_domains_[di];
        if (vd.region.width() > 0 && vd.region.height() > 0) {
            add_fence("domain_" + vd.name, vd.region, vd.cell_ids);
        }
        for (int cid : vd.cell_ids)
            cell_domain_map_[cid] = (int)di;
    }

    // Step 2: Run standard placement with domain fences
    place();
    dr.domains_placed = (int)voltage_domains_.size();

    // Step 3: Enforce domain containment — push escaped cells back
    for (auto& [cid, did] : cell_domain_map_) {
        if (cid < 0 || cid >= (int)pd_.cells.size()) continue;
        auto& vd = voltage_domains_[did];
        if (vd.region.width() <= 0) continue;
        auto& cell = pd_.cells[cid];
        cell.position.x = std::max(vd.region.x0, std::min(cell.position.x, vd.region.x1 - cell.width));
        cell.position.y = std::max(vd.region.y0, std::min(cell.position.y, vd.region.y1 - cell.height));
    }

    // Step 4: Identify cross-domain nets
    for (size_t ni = 0; ni < pd_.nets.size(); ni++) {
        if (nets_cross_domains((NetId)ni))
            dr.cross_domain_nets++;
    }

    // Step 5: Insert level shifters and isolation cells
    dr.level_shifters_inserted = insert_level_shifters();
    dr.message = "Placed " + std::to_string(dr.domains_placed) + " voltage domains, " +
                 std::to_string(dr.cross_domain_nets) + " cross-domain nets, " +
                 std::to_string(dr.level_shifters_inserted) + " level shifters";
    return dr;
}

int AnalyticalPlacer::insert_level_shifters() {
    int count = 0;
    for (size_t ni = 0; ni < pd_.nets.size(); ni++) {
        if (!nets_cross_domains((NetId)ni)) continue;
        auto& n = pd_.nets[ni];
        if (n.cell_ids.empty()) continue;
        int drv_id = n.cell_ids[0];
        int drv_domain = find_domain_for_cell(drv_id);
        if (drv_domain < 0) continue;

        for (size_t fi = 1; fi < n.cell_ids.size(); fi++) {
            int fo = n.cell_ids[fi];
            int fo_domain = find_domain_for_cell(fo);
            if (fo_domain < 0 || fo_domain == drv_domain) continue;

            double v_from = voltage_domains_[drv_domain].voltage;
            double v_to = voltage_domains_[fo_domain].voltage;
            std::string ls_type = (v_from < v_to) ? "LS_LH" : "LS_HL";

            // Place level shifter at boundary between domains
            LevelShifterCell ls;
            ls.from_domain = drv_domain;
            ls.to_domain = fo_domain;
            ls.net_id = (NetId)ni;
            ls.cell_type = ls_type;

            CellInstance lsc;
            lsc.cell_type = ls_type;
            lsc.width = 3.0;
            lsc.height = 1.4;
            // Place at midpoint between driver and fanout
            auto& drv_cell = pd_.cells[drv_id];
            auto& fo_cell = pd_.cells[fo];
            lsc.position.x = (drv_cell.position.x + fo_cell.position.x) / 2.0;
            lsc.position.y = (drv_cell.position.y + fo_cell.position.y) / 2.0;
            lsc.placed = true;
            ls.cell_id = (int)pd_.cells.size();
            pd_.cells.push_back(lsc);
            level_shifters_.push_back(ls);
            count++;
        }
    }
    return count;
}

int AnalyticalPlacer::insert_isolation_cells(const std::string& control_signal,
                                              const std::string& clamp) {
    int count = 0;
    // Insert isolation cells at outputs of each non-default domain
    for (size_t di = 1; di < voltage_domains_.size(); di++) {
        auto& vd = voltage_domains_[di];
        for (int cid : vd.cell_ids) {
            if (cid < 0 || cid >= (int)pd_.cells.size()) continue;
            // Check if any output net goes outside this domain
            for (size_t ni = 0; ni < pd_.nets.size(); ni++) {
                auto& n = pd_.nets[ni];
                if (n.cell_ids.empty() || n.cell_ids[0] != cid) continue;
                bool crosses = false;
                for (size_t fi = 1; fi < n.cell_ids.size(); fi++) {
                    int fo_dom = find_domain_for_cell(n.cell_ids[fi]);
                    if (fo_dom >= 0 && fo_dom != (int)di) { crosses = true; break; }
                }
                if (!crosses) continue;

                IsolationCell ic;
                ic.domain_id = (int)di;
                ic.net_id = (NetId)ni;
                ic.clamp_type = clamp;
                ic.control_signal = control_signal;

                CellInstance iso;
                iso.cell_type = "ISO_" + clamp;
                iso.width = 2.0;
                iso.height = 1.4;
                iso.position = pd_.cells[cid].position;
                iso.position.x += pd_.cells[cid].width + 0.5;
                iso.placed = true;
                ic.cell_id = (int)pd_.cells.size();
                pd_.cells.push_back(iso);
                isolation_cells_.push_back(ic);
                count++;
            }
        }
    }
    return count;
}

int AnalyticalPlacer::insert_power_switches(double switch_pitch,
                                             const std::string& enable) {
    int count = 0;
    for (size_t di = 1; di < voltage_domains_.size(); di++) {
        auto& vd = voltage_domains_[di];
        if (vd.region.width() <= 0) continue;

        // Place power switches in a grid within the domain region
        for (double y = vd.region.y0; y < vd.region.y1; y += switch_pitch) {
            for (double x = vd.region.x0; x < vd.region.x1; x += switch_pitch) {
                PowerSwitchCell ps;
                ps.domain_id = (int)di;
                ps.position = {x, y};
                ps.enable_signal = enable;
                ps.switch_resistance = 0.1;

                CellInstance psc;
                psc.cell_type = "POWER_SWITCH";
                psc.width = 4.0;
                psc.height = 2.8;
                psc.position = {x, y};
                psc.placed = true;
                ps.cell_id = (int)pd_.cells.size();
                pd_.cells.push_back(psc);
                power_switches_.push_back(ps);
                count++;
            }
        }
    }
    return count;
}

} // namespace sf
