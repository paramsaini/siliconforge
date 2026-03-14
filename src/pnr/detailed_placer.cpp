// SiliconForge — Detailed Placement Engine Implementation
// Abacus legalization, cell swapping, and local reordering.

#include "pnr/detailed_placer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

namespace sf {

// ============================================================================
// Construction & row setup
// ============================================================================

DetailedPlacer::DetailedPlacer(PhysicalDesign& pd, const Netlist& nl)
    : pd_(pd), nl_(nl) {}

void DetailedPlacer::build_rows() {
    rows_.clear();
    double rh = cfg_.row_height_um > 0 ? cfg_.row_height_um : pd_.row_height;
    double sw = cfg_.site_width_um > 0 ? cfg_.site_width_um : pd_.site_width;
    if (rh <= 0) rh = 2.8;
    if (sw <= 0) sw = 0.38;

    double y = pd_.die_area.y0;
    while (y + rh <= pd_.die_area.y1 + 1e-9) {
        PlacementRow row;
        row.y       = y;
        row.x_start = pd_.die_area.x0;
        row.x_end   = pd_.die_area.x1;
        row.height  = rh;
        row.site_w  = sw;
        rows_.push_back(row);
        y += rh;
    }
    row_occupancy_.assign(rows_.size(), {});
}

int DetailedPlacer::find_nearest_row(double y) const {
    if (rows_.empty()) return -1;
    int best = 0;
    double best_dist = std::abs(y - rows_[0].y);
    for (int i = 1; i < (int)rows_.size(); ++i) {
        double d = std::abs(y - rows_[i].y);
        if (d < best_dist) { best_dist = d; best = i; }
    }
    return best;
}

double DetailedPlacer::snap_to_grid(double x) const {
    double sw = rows_.empty() ? cfg_.site_width_um : rows_[0].site_w;
    if (sw <= 0) return x;
    double origin = pd_.die_area.x0;
    double offset = x - origin;
    double snapped = std::round(offset / sw) * sw;
    return origin + snapped;
}

bool DetailedPlacer::check_row_space(int row_idx, double x, double w) const {
    if (row_idx < 0 || row_idx >= (int)rows_.size()) return false;
    return row_has_space(row_idx, x, w);
}

// ============================================================================
// Row occupancy management
// ============================================================================

bool DetailedPlacer::row_has_space(int row, double x, double w) const {
    if (row < 0 || row >= (int)row_occupancy_.size()) return false;
    double x1 = x + w;
    for (auto& iv : row_occupancy_[row]) {
        if (!(x1 <= iv.x0 + 1e-9 || x >= iv.x1 - 1e-9))
            return false;
    }
    // Check row bounds
    if (x < rows_[row].x_start - 1e-9 || x1 > rows_[row].x_end + 1e-9)
        return false;
    return true;
}

void DetailedPlacer::add_to_occupancy(int row, double x, double w, int cell_idx) {
    if (row < 0 || row >= (int)row_occupancy_.size()) return;
    row_occupancy_[row].push_back({x, x + w, cell_idx});
}

void DetailedPlacer::remove_from_occupancy(int row, int cell_idx) {
    if (row < 0 || row >= (int)row_occupancy_.size()) return;
    auto& occ = row_occupancy_[row];
    occ.erase(std::remove_if(occ.begin(), occ.end(),
        [cell_idx](const Interval& iv) { return iv.cell_idx == cell_idx; }),
        occ.end());
}

double DetailedPlacer::find_nearest_free(int row, double x, double w) const {
    if (row < 0 || row >= (int)rows_.size()) return x;
    double sw   = rows_[row].site_w;
    double xmin = rows_[row].x_start;
    double xmax = rows_[row].x_end;

    // Try the snapped position first
    double snapped = snap_to_grid(x);
    if (snapped < xmin) snapped = xmin;
    if (snapped + w > xmax) snapped = xmax - w;
    if (row_has_space(row, snapped, w)) return snapped;

    // Search outward from snapped position
    double best_x = snapped;
    double best_dist = std::numeric_limits<double>::max();
    double step = sw > 0 ? sw : 0.38;

    for (double trial = xmin; trial + w <= xmax + 1e-9; trial += step) {
        if (row_has_space(row, trial, w)) {
            double d = std::abs(trial - x);
            if (d < best_dist) {
                best_dist = d;
                best_x = trial;
            }
        }
    }
    return best_x;
}

void DetailedPlacer::insert_cell_in_row(int row_idx, int cell_idx, double x) {
    auto& c = pd_.cells[cell_idx];
    c.position.x = x;
    c.position.y = rows_[row_idx].y;
    c.placed = true;
    add_to_occupancy(row_idx, x, c.width, cell_idx);
}

void DetailedPlacer::remove_cell_from_row(int cell_idx) {
    for (int r = 0; r < (int)row_occupancy_.size(); ++r)
        remove_from_occupancy(r, cell_idx);
}

// ============================================================================
// HPWL computation
// ============================================================================

double DetailedPlacer::net_hpwl(int net_idx) const {
    auto& net = pd_.nets[net_idx];
    if (net.cell_ids.size() < 2) return 0;

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;

    for (size_t i = 0; i < net.cell_ids.size(); ++i) {
        auto& c = pd_.cells[net.cell_ids[i]];
        double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c.width / 2);
        double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c.height / 2);
        min_x = std::min(min_x, px); max_x = std::max(max_x, px);
        min_y = std::min(min_y, py); max_y = std::max(max_y, py);
    }
    return (max_x - min_x) + (max_y - min_y);
}

double DetailedPlacer::compute_hpwl() const {
    double total = 0;
    for (int i = 0; i < (int)pd_.nets.size(); ++i)
        total += net_hpwl(i);
    return total;
}

// Compute HPWL delta if cells ci and cj swap positions
double DetailedPlacer::compute_swap_delta(int ci, int cj) const {
    // Collect nets connected to ci or cj
    double before = 0, after = 0;

    // Save original positions
    Point pos_i = pd_.cells[ci].position;
    Point pos_j = pd_.cells[cj].position;

    // Compute before HPWL for affected nets
    for (int n = 0; n < (int)pd_.nets.size(); ++n) {
        auto& net = pd_.nets[n];
        bool touches_i = false, touches_j = false;
        for (int cid : net.cell_ids) {
            if (cid == ci) touches_i = true;
            if (cid == cj) touches_j = true;
        }
        if (touches_i || touches_j)
            before += net_hpwl(n);
    }

    // Swap positions temporarily (const_cast for measurement)
    auto& cells = const_cast<std::vector<CellInstance>&>(pd_.cells);
    cells[ci].position = pos_j;
    cells[cj].position = pos_i;

    // Compute after HPWL
    for (int n = 0; n < (int)pd_.nets.size(); ++n) {
        auto& net = pd_.nets[n];
        bool touches_i = false, touches_j = false;
        for (int cid : net.cell_ids) {
            if (cid == ci) touches_i = true;
            if (cid == cj) touches_j = true;
        }
        if (touches_i || touches_j)
            after += net_hpwl(n);
    }

    // Restore
    cells[ci].position = pos_i;
    cells[cj].position = pos_j;

    return before - after; // positive means improvement
}

// ============================================================================
// Abacus legalization (ISPD 2008)
// ============================================================================

int DetailedPlacer::abacus_legalize() {
    build_rows();
    if (rows_.empty() || pd_.cells.empty()) return 0;

    // Sort cells by x-coordinate (left-to-right sweep)
    std::vector<int> order(pd_.cells.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return pd_.cells[a].position.x < pd_.cells[b].position.x;
    });

    int moved = 0;

    for (int idx : order) {
        auto& cell = pd_.cells[idx];
        if (cell.is_macro) continue; // skip macros

        double orig_x = cell.position.x;
        double orig_y = cell.position.y;

        // Find best row: closest to original y with available space
        int best_row = -1;
        double best_cost = std::numeric_limits<double>::max();
        double best_x = orig_x;

        for (int r = 0; r < (int)rows_.size(); ++r) {
            double row_y = rows_[r].y;
            double dy = std::abs(orig_y - row_y);

            // Skip if y-displacement already exceeds current best
            if (dy >= best_cost) continue;

            double placed_x = find_nearest_free(r, orig_x, cell.width);
            double dx = std::abs(placed_x - orig_x);
            double cost = dx + dy;

            // Check displacement limit
            if (cfg_.max_displacement_um > 0 && cost > cfg_.max_displacement_um)
                continue;

            if (cost < best_cost) {
                best_cost = cost;
                best_row = r;
                best_x = placed_x;
            }
        }

        if (best_row >= 0) {
            double new_x = snap_to_grid(best_x);
            // Re-check after snapping
            if (!row_has_space(best_row, new_x, cell.width))
                new_x = find_nearest_free(best_row, best_x, cell.width);

            insert_cell_in_row(best_row, idx, new_x);

            if (std::abs(new_x - orig_x) > 1e-9 ||
                std::abs(rows_[best_row].y - orig_y) > 1e-9) {
                moved++;
            }
        }
    }
    return moved;
}

// ============================================================================
// Pairwise cell swapping for HPWL improvement
// ============================================================================

int DetailedPlacer::cell_swap_optimize(double max_dist) {
    int total_swaps = 0;
    bool improved = true;
    int pass = 0;

    while (improved && pass < cfg_.iterations) {
        improved = false;
        pass++;

        for (int i = 0; i < (int)pd_.cells.size(); ++i) {
            if (pd_.cells[i].is_macro) continue;

            for (int j = i + 1; j < (int)pd_.cells.size(); ++j) {
                if (pd_.cells[j].is_macro) continue;

                // Distance check
                double dist = pd_.cells[i].position.dist(pd_.cells[j].position);
                if (dist > max_dist) continue;

                // Skip if different heights (cannot swap across row heights)
                if (std::abs(pd_.cells[i].height - pd_.cells[j].height) > 1e-9)
                    continue;

                double delta = compute_swap_delta(i, j);
                if (delta > 1e-9) {
                    // Perform swap
                    Point tmp = pd_.cells[i].position;
                    pd_.cells[i].position = pd_.cells[j].position;
                    pd_.cells[j].position = tmp;

                    // Update occupancy
                    remove_cell_from_row(i);
                    remove_cell_from_row(j);
                    int ri = find_nearest_row(pd_.cells[i].position.y);
                    int rj = find_nearest_row(pd_.cells[j].position.y);
                    if (ri >= 0)
                        add_to_occupancy(ri, pd_.cells[i].position.x, pd_.cells[i].width, i);
                    if (rj >= 0)
                        add_to_occupancy(rj, pd_.cells[j].position.x, pd_.cells[j].width, j);

                    total_swaps++;
                    improved = true;
                }
            }
        }
    }
    return total_swaps;
}

// ============================================================================
// Window-based local reordering
// ============================================================================

int DetailedPlacer::local_reorder(int window_size) {
    if (window_size < 2) window_size = 3;
    if (window_size > 5) window_size = 5;
    int total_reorders = 0;

    for (int r = 0; r < (int)row_occupancy_.size(); ++r) {
        auto& occ = row_occupancy_[r];
        if ((int)occ.size() < window_size) continue;

        // Sort intervals by x position
        std::sort(occ.begin(), occ.end(),
            [](const Interval& a, const Interval& b) { return a.x0 < b.x0; });

        // Sliding window
        for (int start = 0; start + window_size <= (int)occ.size(); ++start) {
            // Collect cell indices in window
            std::vector<int> window_cells;
            std::vector<double> positions;
            for (int w = 0; w < window_size; ++w) {
                window_cells.push_back(occ[start + w].cell_idx);
                positions.push_back(occ[start + w].x0);
            }

            // Compute current HPWL for affected nets
            double best_hpwl = 0;
            for (int n = 0; n < (int)pd_.nets.size(); ++n) {
                for (int cid : pd_.nets[n].cell_ids) {
                    for (int wc : window_cells) {
                        if (cid == wc) { best_hpwl += net_hpwl(n); goto next_net_cur; }
                    }
                }
                next_net_cur:;
            }

            // Try all permutations
            std::vector<int> perm(window_size);
            std::iota(perm.begin(), perm.end(), 0);
            std::vector<int> best_perm = perm;

            // Save original positions
            std::vector<Point> orig_pos(window_size);
            for (int w = 0; w < window_size; ++w)
                orig_pos[w] = pd_.cells[window_cells[w]].position;

            while (std::next_permutation(perm.begin(), perm.end())) {
                // Assign cells to positions according to permutation
                // Place cells left-to-right into available positions
                double x_cursor = positions[0];
                for (int w = 0; w < window_size; ++w) {
                    int ci = window_cells[perm[w]];
                    pd_.cells[ci].position.x = x_cursor;
                    x_cursor += pd_.cells[ci].width;
                }

                // Compute new HPWL
                double trial_hpwl = 0;
                for (int n = 0; n < (int)pd_.nets.size(); ++n) {
                    for (int cid : pd_.nets[n].cell_ids) {
                        for (int wc : window_cells) {
                            if (cid == wc) { trial_hpwl += net_hpwl(n); goto next_net_trial; }
                        }
                    }
                    next_net_trial:;
                }

                if (trial_hpwl < best_hpwl - 1e-9) {
                    best_hpwl = trial_hpwl;
                    best_perm = perm;
                }
            }

            // Restore and apply best permutation
            for (int w = 0; w < window_size; ++w)
                pd_.cells[window_cells[w]].position = orig_pos[w];

            // Check if best permutation differs from identity
            bool changed = false;
            for (int w = 0; w < window_size; ++w) {
                if (best_perm[w] != w) { changed = true; break; }
            }

            if (changed) {
                // Apply best permutation
                double x_cursor = positions[0];
                for (int w = 0; w < window_size; ++w) {
                    int ci = window_cells[best_perm[w]];
                    pd_.cells[ci].position.x = x_cursor;
                    x_cursor += pd_.cells[ci].width;
                }

                // Update occupancy for moved cells
                for (int w = 0; w < window_size; ++w) {
                    int ci = window_cells[w];
                    remove_from_occupancy(r, ci);
                }
                for (int w = 0; w < window_size; ++w) {
                    int ci = window_cells[best_perm[w]];
                    double xp = pd_.cells[ci].position.x;
                    add_to_occupancy(r, xp, pd_.cells[ci].width, ci);
                }
                total_reorders++;
            }
        }
    }
    return total_reorders;
}

// ============================================================================
// High-level entry points
// ============================================================================

DetailedPlaceResult DetailedPlacer::legalize(const DetailedPlaceConfig& cfg) {
    cfg_ = cfg;
    auto t0 = std::chrono::high_resolution_clock::now();

    DetailedPlaceResult result;
    result.hpwl_before = compute_hpwl();

    // Save original positions for displacement tracking
    std::vector<Point> orig(pd_.cells.size());
    for (size_t i = 0; i < pd_.cells.size(); ++i)
        orig[i] = pd_.cells[i].position;

    if (cfg_.enable_legalization) {
        result.cells_moved = abacus_legalize();
    }

    // Compute displacement
    for (size_t i = 0; i < pd_.cells.size(); ++i) {
        double d = pd_.cells[i].position.dist(orig[i]);
        result.total_displacement += d;
        result.max_displacement = std::max(result.max_displacement, d);
    }

    result.hpwl_after = compute_hpwl();
    result.legalization_pass = !pd_.has_overlaps();

    if (result.hpwl_before > 1e-9)
        result.hpwl_improvement_pct =
            (result.hpwl_before - result.hpwl_after) / result.hpwl_before * 100.0;

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "Legalization: " << result.cells_moved << " cells moved, "
        << "max disp=" << result.max_displacement << "um, "
        << "HPWL " << result.hpwl_before << " -> " << result.hpwl_after
        << (result.legalization_pass ? " [PASS]" : " [FAIL]");
    result.report = oss.str();

    return result;
}

DetailedPlaceResult DetailedPlacer::optimize(const DetailedPlaceConfig& cfg) {
    cfg_ = cfg;
    auto t0 = std::chrono::high_resolution_clock::now();

    DetailedPlaceResult result;
    result.hpwl_before = compute_hpwl();

    // Save originals
    std::vector<Point> orig(pd_.cells.size());
    for (size_t i = 0; i < pd_.cells.size(); ++i)
        orig[i] = pd_.cells[i].position;

    // Phase 1: Legalize
    if (cfg_.enable_legalization)
        result.cells_moved = abacus_legalize();

    // Phase 2: Cell swap
    if (cfg_.enable_cell_swap) {
        double max_dist = cfg_.max_swap_distance * cfg_.site_width_um;
        result.swaps_performed = cell_swap_optimize(max_dist);
    }

    // Phase 3: Local reorder
    if (cfg_.enable_local_reorder)
        local_reorder(3);

    // Compute displacement
    for (size_t i = 0; i < pd_.cells.size(); ++i) {
        double d = pd_.cells[i].position.dist(orig[i]);
        result.total_displacement += d;
        result.max_displacement = std::max(result.max_displacement, d);
    }

    result.hpwl_after = compute_hpwl();
    result.legalization_pass = !pd_.has_overlaps();

    if (result.hpwl_before > 1e-9)
        result.hpwl_improvement_pct =
            (result.hpwl_before - result.hpwl_after) / result.hpwl_before * 100.0;

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "Optimize: moved=" << result.cells_moved
        << " swaps=" << result.swaps_performed
        << " HPWL " << result.hpwl_before << " -> " << result.hpwl_after
        << " (" << result.hpwl_improvement_pct << "% improvement)"
        << (result.legalization_pass ? " [LEGAL]" : " [ILLEGAL]");
    result.report = oss.str();

    return result;
}

DetailedPlaceResult DetailedPlacer::run_enhanced() {
    // Full flow: legalize with tight constraints, then multi-pass optimize
    DetailedPlaceConfig cfg;
    cfg.enable_legalization  = true;
    cfg.enable_cell_swap     = true;
    cfg.enable_local_reorder = true;
    cfg.max_displacement_um  = cfg_.max_displacement_um > 0 ? cfg_.max_displacement_um : 50.0;
    cfg.row_height_um        = cfg_.row_height_um > 0 ? cfg_.row_height_um : 2.8;
    cfg.site_width_um        = cfg_.site_width_um > 0 ? cfg_.site_width_um : 0.38;
    cfg.max_swap_distance    = 10;
    cfg.iterations           = 50;

    auto t0 = std::chrono::high_resolution_clock::now();

    DetailedPlaceResult result;
    result.hpwl_before = compute_hpwl();

    std::vector<Point> orig(pd_.cells.size());
    for (size_t i = 0; i < pd_.cells.size(); ++i)
        orig[i] = pd_.cells[i].position;

    cfg_ = cfg;

    // Pass 1: Legalize
    result.cells_moved = abacus_legalize();

    // Pass 2: Iterative swap + reorder
    double prev_hpwl = compute_hpwl();
    for (int iter = 0; iter < 3; ++iter) {
        double md = cfg.max_swap_distance * cfg.site_width_um;
        result.swaps_performed += cell_swap_optimize(md);
        local_reorder(3);

        double cur_hpwl = compute_hpwl();
        if (prev_hpwl - cur_hpwl < 1e-6) break;
        prev_hpwl = cur_hpwl;
    }

    // Pass 3: Wider window reorder
    local_reorder(4);

    for (size_t i = 0; i < pd_.cells.size(); ++i) {
        double d = pd_.cells[i].position.dist(orig[i]);
        result.total_displacement += d;
        result.max_displacement = std::max(result.max_displacement, d);
    }

    result.hpwl_after = compute_hpwl();
    result.legalization_pass = !pd_.has_overlaps();

    if (result.hpwl_before > 1e-9)
        result.hpwl_improvement_pct =
            (result.hpwl_before - result.hpwl_after) / result.hpwl_before * 100.0;

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream oss;
    oss << "Enhanced: moved=" << result.cells_moved
        << " swaps=" << result.swaps_performed
        << " HPWL " << result.hpwl_before << " -> " << result.hpwl_after
        << " (" << result.hpwl_improvement_pct << "% improvement)"
        << " time=" << result.time_ms << "ms"
        << (result.legalization_pass ? " [LEGAL]" : " [ILLEGAL]");
    result.report = oss.str();

    return result;
}

} // namespace sf
