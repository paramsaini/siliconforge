// SiliconForge — Floorplanner Implementation (Simulated Annealing)
#include "pnr/floorplan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <cassert>
#include <limits>

namespace sf {

// ---------------------------------------------------------------------------
// B*-tree contour-based packing
// ---------------------------------------------------------------------------

BStarTree::PackResult BStarTree::pack(const std::vector<Macro>& macros) const {
    int n = (int)nodes.size();
    PackResult res;
    res.x.assign(n, 0);
    res.y.assign(n, 0);
    if (n == 0 || root < 0) return res;

    // Contour: list of (x_start, x_end, height) segments sorted by x
    struct ContourSeg { double x0, x1, h; };
    std::vector<ContourSeg> contour;
    contour.push_back({0, 1e9, 0});

    auto query_contour = [&](double x, double w) -> double {
        double max_h = 0;
        double x_end = x + w;
        for (auto& seg : contour) {
            if (seg.x1 <= x || seg.x0 >= x_end) continue;
            max_h = std::max(max_h, seg.h);
        }
        return max_h;
    };

    auto update_contour = [&](double x, double w, double h) {
        double x_end = x + w;
        double new_top = h;
        std::vector<ContourSeg> new_contour;
        for (auto& seg : contour) {
            if (seg.x1 <= x || seg.x0 >= x_end) {
                // No overlap — keep as-is
                new_contour.push_back(seg);
            } else {
                // Partially or fully overlapping
                if (seg.x0 < x)
                    new_contour.push_back({seg.x0, x, seg.h});
                if (seg.x1 > x_end)
                    new_contour.push_back({x_end, seg.x1, seg.h});
            }
        }
        new_contour.push_back({x, x_end, new_top});
        std::sort(new_contour.begin(), new_contour.end(),
                  [](auto& a, auto& b) { return a.x0 < b.x0; });
        contour = std::move(new_contour);
    };

    // DFS traversal: left child → right of parent, right child → above parent (sibling)
    // Use iterative stack to avoid deep recursion
    struct StackEntry { int node_idx; bool is_left_child; int parent_idx; };
    std::vector<StackEntry> stack;
    stack.push_back({root, false, -1});

    while (!stack.empty()) {
        auto [ni, is_left, pi] = stack.back();
        stack.pop_back();
        if (ni < 0 || ni >= n) continue;

        auto& nd = nodes[ni];
        int mi = nd.macro_idx;
        double mw = nd.is_rotated ? macros[mi].height : macros[mi].width;
        double mh = nd.is_rotated ? macros[mi].width  : macros[mi].height;

        double px = 0;
        if (pi >= 0) {
            int pmi = nodes[pi].macro_idx;
            double pw = nodes[pi].is_rotated ? macros[pmi].height : macros[pmi].width;
            if (is_left) {
                // Left child goes RIGHT of parent
                px = res.x[pi] + pw;
            } else {
                // Right child goes same x as parent (sibling above)
                px = res.x[pi];
            }
        }

        double py = query_contour(px, mw);
        res.x[ni] = px;
        res.y[ni] = py;
        update_contour(px, mw, py + mh);

        // Push right child first so left is processed first
        if (nd.right_child >= 0)
            stack.push_back({nd.right_child, false, ni});
        if (nd.left_child >= 0)
            stack.push_back({nd.left_child, true, ni});
    }

    // Compute bounding box
    res.total_width = 0;
    res.total_height = 0;
    for (int i = 0; i < n; ++i) {
        int mi = nodes[i].macro_idx;
        double mw = nodes[i].is_rotated ? macros[mi].height : macros[mi].width;
        double mh = nodes[i].is_rotated ? macros[mi].width  : macros[mi].height;
        res.total_width  = std::max(res.total_width,  res.x[i] + mw);
        res.total_height = std::max(res.total_height, res.y[i] + mh);
    }
    res.area = res.total_width * res.total_height;
    res.aspect_ratio = (res.total_height > 0) ? res.total_width / res.total_height : 1.0;
    return res;
}

void Floorplanner::add_macro(const std::string& name, double w, double h, bool fixed) {
    int id = (int)macros_.size();
    macros_.push_back({id, name, w, h, fixed, {0,0}, false});
}

void Floorplanner::add_connection(int a, int b, double weight) {
    connections_.push_back({a, b, weight});
}

void Floorplanner::random_swap(std::mt19937& rng) {
    if (macros_.size() < 2) return;
    std::uniform_int_distribution<int> dist(0, (int)macros_.size()-1);
    int a = dist(rng), b = dist(rng);
    while (b == a || macros_[a].fixed || macros_[b].fixed) {
        a = dist(rng); b = dist(rng);
    }
    std::swap(macros_[a].position, macros_[b].position);
}

void Floorplanner::random_rotate(std::mt19937& rng) {
    std::uniform_int_distribution<int> dist(0, (int)macros_.size()-1);
    int idx = dist(rng);
    int tries = 0;
    while (macros_[idx].fixed && tries++ < 20) idx = dist(rng);
    if (macros_[idx].fixed) return;
    macros_[idx].rotated = !macros_[idx].rotated;
    std::swap(macros_[idx].width, macros_[idx].height);
}

void Floorplanner::random_move(std::mt19937& rng, double range) {
    std::uniform_int_distribution<int> idx_dist(0, (int)macros_.size()-1);
    std::uniform_real_distribution<double> delta(-range, range);
    int idx = idx_dist(rng);
    int tries = 0;
    while (macros_[idx].fixed && tries++ < 20) idx = idx_dist(rng);
    if (macros_[idx].fixed) return;
    macros_[idx].position.x += delta(rng);
    macros_[idx].position.y += delta(rng);
    macros_[idx].position.x = std::max(0.0, macros_[idx].position.x);
    macros_[idx].position.y = std::max(0.0, macros_[idx].position.y);
}

void Floorplanner::pack() {
    // Sort by area (largest first) then pack left-bottom
    std::vector<int> order(macros_.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return macros_[a].width * macros_[a].height > macros_[b].width * macros_[b].height;
    });

    // Simple skyline packing
    double max_width = 0;
    for (auto& m : macros_) max_width += m.width;
    max_width = std::sqrt(max_width * max_width / macros_.size()) * 2; // estimate width

    std::vector<double> skyline((int)max_width + 1, 0);
    for (int idx : order) {
        auto& m = macros_[idx];
        if (m.fixed) continue;
        // Find lowest position where macro fits
        double best_y = 1e18;
        int best_x = 0;
        int mw = std::max(1, (int)m.width);
        for (int x = 0; x + mw <= (int)skyline.size(); ++x) {
            double max_y = 0;
            for (int dx = 0; dx < mw; ++dx)
                max_y = std::max(max_y, skyline[x + dx]);
            if (max_y < best_y) {
                best_y = max_y;
                best_x = x;
            }
        }
        m.position.x = best_x;
        m.position.y = best_y;
        for (int dx = 0; dx < mw && best_x + dx < (int)skyline.size(); ++dx)
            skyline[best_x + dx] = best_y + m.height;
    }
}

double Floorplanner::compute_wirelength() const {
    double wl = 0;
    for (auto& c : connections_) {
        auto& ma = macros_[c.a]; auto& mb = macros_[c.b];
        double cx_a = ma.position.x + ma.width/2;
        double cy_a = ma.position.y + ma.height/2;
        double cx_b = mb.position.x + mb.width/2;
        double cy_b = mb.position.y + mb.height/2;
        wl += c.weight * (std::abs(cx_a - cx_b) + std::abs(cy_a - cy_b));
    }
    return wl;
}

double Floorplanner::bounding_area() const {
    double max_x = 0, max_y = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
    }
    return max_x * max_y;
}

double Floorplanner::compute_cost() const {
    double area = bounding_area();
    double wl = compute_wirelength();
    // Check aspect ratio
    double max_x = 0, max_y = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
    }
    double ar = (max_y > 0) ? max_x / max_y : 1.0;
    double ar_penalty = 0;
    if (ar < min_ar_) ar_penalty = (min_ar_ - ar) * 1000;
    if (ar > max_ar_) ar_penalty = (ar - max_ar_) * 1000;

    return area + wl * 10 + ar_penalty;
}

FloorplanResult Floorplanner::solve(int max_iters, double init_temp, double cool_rate) {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Initial packing
    pack();

    std::mt19937 rng(42);
    double temp = init_temp;
    double best_cost = compute_cost();
    auto best_state = macros_;
    double current_cost = best_cost;
    double move_range = 20.0;

    for (int iter = 0; iter < max_iters; ++iter) {
        auto saved = macros_;

        // Random perturbation
        std::uniform_int_distribution<int> move_type(0, 2);
        int mt = move_type(rng);
        if (mt == 0) random_swap(rng);
        else if (mt == 1) random_rotate(rng);
        else random_move(rng, move_range);

        pack();
        double new_cost = compute_cost();
        double delta = new_cost - current_cost;

        // Metropolis criterion
        std::uniform_real_distribution<double> unif(0, 1);
        if (delta < 0 || unif(rng) < std::exp(-delta / temp)) {
            current_cost = new_cost;
            if (new_cost < best_cost) {
                best_cost = new_cost;
                best_state = macros_;
            }
        } else {
            macros_ = saved; // reject move
        }

        temp *= cool_rate;
        move_range *= 0.999;
    }

    macros_ = best_state;
    pack(); // final pack

    auto t1 = std::chrono::high_resolution_clock::now();
    FloorplanResult r;
    double max_x = 0, max_y = 0;
    double macro_area = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
        macro_area += m.width * m.height;
    }
    r.die_width = max_x; r.die_height = max_y;
    r.total_area = max_x * max_y;
    r.dead_space_pct = r.total_area > 0 ? (1.0 - macro_area / r.total_area) * 100 : 0;
    r.wirelength = compute_wirelength();
    r.iterations = max_iters;
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Floorplan: " + std::to_string((int)r.die_width) + "×" +
                std::to_string((int)r.die_height) + ", dead space: " +
                std::to_string((int)r.dead_space_pct) + "%";
    return r;
}

PhysicalDesign Floorplanner::to_physical_design() const {
    PhysicalDesign pd;
    double max_x = 0, max_y = 0;
    for (auto& m : macros_) {
        max_x = std::max(max_x, m.position.x + m.width);
        max_y = std::max(max_y, m.position.y + m.height);
    }
    pd.die_area = Rect(0, 0, max_x, max_y);
    for (auto& m : macros_) {
        int cid = pd.add_cell(m.name, "MACRO", m.width, m.height);
        pd.cells[cid].position = m.position;
        pd.cells[cid].placed = true;
    }
    for (auto& c : connections_)
        pd.add_net("fn" + std::to_string(c.a) + "_" + std::to_string(c.b), {c.a, c.b});
    return pd;
}

// ---------------------------------------------------------------------------
// Pin-aware orientation
// ---------------------------------------------------------------------------

void Floorplanner::set_pin_positions(
        const std::vector<std::vector<std::pair<double,double>>>& pins) {
    pin_positions_ = pins;
}

// ---------------------------------------------------------------------------
// Wirelength estimation — HPWL over connections
// ---------------------------------------------------------------------------

double Floorplanner::estimate_wirelength(const BStarTree::PackResult& pk) const {
    double wl = 0;
    for (auto& c : connections_) {
        if (c.a < 0 || c.a >= (int)macros_.size()) continue;
        if (c.b < 0 || c.b >= (int)macros_.size()) continue;
        auto& ma = macros_[c.a];
        auto& mb = macros_[c.b];
        double cx_a = ma.position.x + ma.width / 2;
        double cy_a = ma.position.y + ma.height / 2;
        double cx_b = mb.position.x + mb.width / 2;
        double cy_b = mb.position.y + mb.height / 2;
        // If pin positions available, use them for more accurate HPWL
        if (!pin_positions_.empty() &&
            c.a < (int)pin_positions_.size() && !pin_positions_[c.a].empty() &&
            c.b < (int)pin_positions_.size() && !pin_positions_[c.b].empty()) {
            auto& pa = pin_positions_[c.a][0];
            auto& pb = pin_positions_[c.b][0];
            cx_a = ma.position.x + pa.first;
            cy_a = ma.position.y + pa.second;
            cx_b = mb.position.x + pb.first;
            cy_b = mb.position.y + pb.second;
        }
        wl += c.weight * (std::abs(cx_a - cx_b) + std::abs(cy_a - cy_b));
    }
    return wl;
}

// ---------------------------------------------------------------------------
// B*-tree initial construction
// ---------------------------------------------------------------------------

BStarTree Floorplanner::create_initial_tree() {
    BStarTree tree;
    int n = (int)macros_.size();
    if (n == 0) return tree;

    tree.nodes.resize(n);
    // Sort macros by area descending for better initial placement
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return macros_[a].width * macros_[a].height >
               macros_[b].width * macros_[b].height;
    });

    for (int i = 0; i < n; ++i) {
        tree.nodes[i].macro_idx = order[i];
        tree.nodes[i].left_child = -1;
        tree.nodes[i].right_child = -1;
        tree.nodes[i].parent = -1;
        tree.nodes[i].is_rotated = false;
    }

    // Build a balanced binary tree: node i's left child = 2i+1, right child = 2i+2
    tree.root = 0;
    for (int i = 0; i < n; ++i) {
        int lc = 2 * i + 1;
        int rc = 2 * i + 2;
        if (lc < n) {
            tree.nodes[i].left_child = lc;
            tree.nodes[lc].parent = i;
        }
        if (rc < n) {
            tree.nodes[i].right_child = rc;
            tree.nodes[rc].parent = i;
        }
    }
    return tree;
}

// ---------------------------------------------------------------------------
// B*-tree perturbation moves
// ---------------------------------------------------------------------------

Floorplanner::MoveType Floorplanner::random_move_type(std::mt19937& rng) {
    std::uniform_real_distribution<double> d(0, 1);
    double r = d(rng);
    if (r < 0.4) return MoveType::SWAP;
    if (r < 0.7) return MoveType::ROTATE;
    if (r < 0.9) return MoveType::MOVE_SUBTREE;
    return MoveType::RESIZE_CHANNEL;
}

BStarTree Floorplanner::apply_swap(const BStarTree& tree, int a, int b) {
    BStarTree t = tree;
    // Swap the macro assignments of two nodes
    std::swap(t.nodes[a].macro_idx, t.nodes[b].macro_idx);
    std::swap(t.nodes[a].is_rotated, t.nodes[b].is_rotated);
    return t;
}

BStarTree Floorplanner::apply_rotate(const BStarTree& tree, int idx) {
    BStarTree t = tree;
    t.nodes[idx].is_rotated = !t.nodes[idx].is_rotated;
    return t;
}

BStarTree Floorplanner::apply_move_subtree(const BStarTree& tree, int node,
                                            int new_parent, std::mt19937& rng) {
    BStarTree t = tree;
    int n = (int)t.nodes.size();
    if (n < 3 || node == t.root || node == new_parent) return t;
    if (node < 0 || node >= n || new_parent < 0 || new_parent >= n) return t;

    // Check that new_parent is not a descendant of node
    {
        int cur = new_parent;
        while (cur >= 0) {
            if (cur == node) return t;  // would create cycle
            cur = t.nodes[cur].parent;
        }
    }

    // Detach node from its current parent
    int old_parent = t.nodes[node].parent;
    if (old_parent >= 0) {
        if (t.nodes[old_parent].left_child == node) {
            // Replace with node's children: promote one child
            if (t.nodes[node].left_child >= 0) {
                t.nodes[old_parent].left_child = t.nodes[node].left_child;
                t.nodes[t.nodes[node].left_child].parent = old_parent;
                // Attach node's right child to the rightmost of the promoted subtree
                if (t.nodes[node].right_child >= 0) {
                    int cur = t.nodes[node].left_child;
                    while (t.nodes[cur].right_child >= 0)
                        cur = t.nodes[cur].right_child;
                    t.nodes[cur].right_child = t.nodes[node].right_child;
                    t.nodes[t.nodes[node].right_child].parent = cur;
                }
            } else if (t.nodes[node].right_child >= 0) {
                t.nodes[old_parent].left_child = t.nodes[node].right_child;
                t.nodes[t.nodes[node].right_child].parent = old_parent;
            } else {
                t.nodes[old_parent].left_child = -1;
            }
        } else if (t.nodes[old_parent].right_child == node) {
            if (t.nodes[node].left_child >= 0) {
                t.nodes[old_parent].right_child = t.nodes[node].left_child;
                t.nodes[t.nodes[node].left_child].parent = old_parent;
                if (t.nodes[node].right_child >= 0) {
                    int cur = t.nodes[node].left_child;
                    while (t.nodes[cur].right_child >= 0)
                        cur = t.nodes[cur].right_child;
                    t.nodes[cur].right_child = t.nodes[node].right_child;
                    t.nodes[t.nodes[node].right_child].parent = cur;
                }
            } else if (t.nodes[node].right_child >= 0) {
                t.nodes[old_parent].right_child = t.nodes[node].right_child;
                t.nodes[t.nodes[node].right_child].parent = old_parent;
            } else {
                t.nodes[old_parent].right_child = -1;
            }
        }
    }

    // Clear node's children
    t.nodes[node].left_child = -1;
    t.nodes[node].right_child = -1;

    // Insert node as a child of new_parent
    std::uniform_int_distribution<int> side_dist(0, 1);
    bool insert_left = (side_dist(rng) == 0);
    if (insert_left) {
        // Insert as left child; old left child becomes node's left child
        int old_lc = t.nodes[new_parent].left_child;
        t.nodes[new_parent].left_child = node;
        t.nodes[node].parent = new_parent;
        if (old_lc >= 0) {
            t.nodes[node].left_child = old_lc;
            t.nodes[old_lc].parent = node;
        }
    } else {
        int old_rc = t.nodes[new_parent].right_child;
        t.nodes[new_parent].right_child = node;
        t.nodes[node].parent = new_parent;
        if (old_rc >= 0) {
            t.nodes[node].right_child = old_rc;
            t.nodes[old_rc].parent = node;
        }
    }

    return t;
}

BStarTree Floorplanner::perturb(const BStarTree& tree, std::mt19937& rng) {
    int n = (int)tree.nodes.size();
    if (n < 2) return tree;

    MoveType mv = random_move_type(rng);
    std::uniform_int_distribution<int> dist(0, n - 1);

    switch (mv) {
    case MoveType::SWAP: {
        int a = dist(rng), b = dist(rng);
        int tries = 0;
        while ((b == a || macros_[tree.nodes[a].macro_idx].fixed ||
                macros_[tree.nodes[b].macro_idx].fixed) && tries++ < 50) {
            a = dist(rng); b = dist(rng);
        }
        if (a == b) return tree;
        return apply_swap(tree, a, b);
    }
    case MoveType::ROTATE: {
        int idx = dist(rng);
        int tries = 0;
        while (macros_[tree.nodes[idx].macro_idx].fixed && tries++ < 50)
            idx = dist(rng);
        if (macros_[tree.nodes[idx].macro_idx].fixed) return tree;
        return apply_rotate(tree, idx);
    }
    case MoveType::MOVE_SUBTREE: {
        if (n < 3) return tree;
        int node = dist(rng), np = dist(rng);
        int tries = 0;
        while ((node == tree.root || node == np ||
                macros_[tree.nodes[node].macro_idx].fixed) && tries++ < 50) {
            node = dist(rng); np = dist(rng);
        }
        if (node == tree.root || node == np) return tree;
        return apply_move_subtree(tree, node, np, rng);
    }
    case MoveType::RESIZE_CHANNEL: {
        // Toggle rotation of a random non-fixed macro (acts as channel resize)
        int idx = dist(rng);
        int tries = 0;
        while (macros_[tree.nodes[idx].macro_idx].fixed && tries++ < 50)
            idx = dist(rng);
        if (macros_[tree.nodes[idx].macro_idx].fixed) return tree;
        return apply_rotate(tree, idx);
    }
    }
    return tree;  // unreachable
}

// ---------------------------------------------------------------------------
// Cost computation for B*-tree SA
// ---------------------------------------------------------------------------

double Floorplanner::compute_cost(const BStarTree::PackResult& pk) const {
    if (pk.area <= 0) return std::numeric_limits<double>::max();

    // Normalisation bases: sum of macro areas
    double macro_area = 0;
    for (auto& m : macros_) macro_area += m.width * m.height;
    if (macro_area <= 0) macro_area = 1;

    double norm_area = pk.area / macro_area;

    // Wirelength (use pk coordinates to compute HPWL inline)
    double wl = 0;
    int n = (int)pk.x.size();
    // Build macro_idx → node index map
    std::vector<int> macro_node(macros_.size(), -1);
    // We need the tree nodes to know macro_idx, but we only have PackResult.
    // Convention: caller should set pk.wirelength, or we compute from macros_.
    // For robustness, use macros_ positions if available, else pk.wirelength.
    wl = pk.wirelength;
    if (wl <= 0 && !connections_.empty()) {
        // Fall back to current macro positions (set by floorplan_sa after packing)
        for (auto& c : connections_) {
            if (c.a < 0 || c.a >= (int)macros_.size()) continue;
            if (c.b < 0 || c.b >= (int)macros_.size()) continue;
            auto& ma = macros_[c.a];
            auto& mb = macros_[c.b];
            wl += c.weight * (std::abs(ma.position.x + ma.width/2 -
                                        mb.position.x - mb.width/2) +
                              std::abs(ma.position.y + ma.height/2 -
                                        mb.position.y - mb.height/2));
        }
    }
    double norm_wl = (macro_area > 0) ? wl / std::sqrt(macro_area) : 0;

    double ar_diff = std::abs(pk.aspect_ratio - sa_cfg_.target_aspect_ratio);

    // Reject extreme aspect ratios
    if (pk.aspect_ratio > sa_cfg_.max_aspect_ratio ||
        pk.aspect_ratio < 1.0 / sa_cfg_.max_aspect_ratio) {
        ar_diff += 10.0;
    }

    return sa_cfg_.area_weight     * norm_area +
           sa_cfg_.wirelength_weight * norm_wl +
           sa_cfg_.aspect_weight    * ar_diff;
}

double Floorplanner::estimate_channel_area(const BStarTree::PackResult& pk) const {
    double macro_area = 0;
    for (auto& m : macros_) macro_area += m.width * m.height;
    return pk.area - macro_area;
}

// ---------------------------------------------------------------------------
// Simulated annealing main loop
// ---------------------------------------------------------------------------

FloorplanResult Floorplanner::floorplan_sa() {
    auto t0 = std::chrono::high_resolution_clock::now();
    int n = (int)macros_.size();
    FloorplanResult result;

    if (n == 0) {
        result.message = "No macros to place";
        return result;
    }

    std::mt19937 rng(42);

    // Create initial B*-tree and pack
    BStarTree best_tree = create_initial_tree();
    BStarTree::PackResult best_pack = best_tree.pack(macros_);

    // Compute wirelength for initial packing by temporarily setting positions
    auto apply_pack_to_macros = [&](const BStarTree& tree, const BStarTree::PackResult& pk) {
        for (int i = 0; i < (int)tree.nodes.size(); ++i) {
            int mi = tree.nodes[i].macro_idx;
            macros_[mi].position.x = pk.x[i];
            macros_[mi].position.y = pk.y[i];
            macros_[mi].rotated = tree.nodes[i].is_rotated;
            if (tree.nodes[i].is_rotated) {
                // Keep original width/height; rotation handled in pack
            }
        }
    };

    auto compute_wl_for_pack = [&](const BStarTree& tree,
                                    BStarTree::PackResult& pk) {
        // Temporarily update macro positions to compute wirelength
        auto saved = macros_;
        apply_pack_to_macros(tree, pk);
        double wl = 0;
        for (auto& c : connections_) {
            if (c.a < 0 || c.a >= (int)macros_.size()) continue;
            if (c.b < 0 || c.b >= (int)macros_.size()) continue;
            auto& ma = macros_[c.a];
            auto& mb = macros_[c.b];
            wl += c.weight * (std::abs(ma.position.x + ma.width/2 -
                                        mb.position.x - mb.width/2) +
                              std::abs(ma.position.y + ma.height/2 -
                                        mb.position.y - mb.height/2));
        }
        pk.wirelength = wl;
        macros_ = saved;
    };

    compute_wl_for_pack(best_tree, best_pack);
    double best_cost = compute_cost(best_pack);
    BStarTree current_tree = best_tree;
    BStarTree::PackResult current_pack = best_pack;
    double current_cost = best_cost;

    double temp = sa_cfg_.initial_temp;
    int total_moves = 0;
    int accepted_moves = 0;
    int sa_iters = 0;

    while (temp > sa_cfg_.min_temp) {
        int moves_this_temp = sa_cfg_.moves_per_temp * n;
        for (int m = 0; m < moves_this_temp; ++m) {
            BStarTree new_tree = perturb(current_tree, rng);
            BStarTree::PackResult new_pack = new_tree.pack(macros_);
            compute_wl_for_pack(new_tree, new_pack);
            double new_cost = compute_cost(new_pack);

            double delta = new_cost - current_cost;
            ++total_moves;

            // Metropolis criterion
            std::uniform_real_distribution<double> unif(0, 1);
            if (delta < 0 || (temp > 0 && unif(rng) < std::exp(-delta / temp))) {
                current_tree = new_tree;
                current_pack = new_pack;
                current_cost = new_cost;
                ++accepted_moves;

                if (new_cost < best_cost) {
                    best_tree = new_tree;
                    best_pack = new_pack;
                    best_cost = new_cost;
                }
            }
        }
        temp *= sa_cfg_.cooling_rate;
        ++sa_iters;
    }

    // Apply best solution to macros
    apply_pack_to_macros(best_tree, best_pack);
    compute_wl_for_pack(best_tree, best_pack);

    // Build result
    auto t1 = std::chrono::high_resolution_clock::now();
    double macro_area = 0;
    for (auto& m_ref : macros_) macro_area += m_ref.width * m_ref.height;

    result.die_width = best_pack.total_width;
    result.die_height = best_pack.total_height;
    result.total_area = best_pack.area;
    result.dead_space_pct = result.total_area > 0
                            ? (1.0 - macro_area / result.total_area) * 100
                            : 0;
    result.wirelength = best_pack.wirelength;
    result.aspect_ratio = best_pack.aspect_ratio;
    result.sa_iterations = sa_iters;
    result.iterations = total_moves;
    result.sa_acceptance_rate = total_moves > 0
                                ? (double)accepted_moves / total_moves
                                : 0;
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.rotated.resize(macros_.size(), false);
    for (int i = 0; i < (int)best_tree.nodes.size(); ++i) {
        int mi = best_tree.nodes[i].macro_idx;
        result.rotated[mi] = best_tree.nodes[i].is_rotated;
    }
    result.message = "SA Floorplan: " + std::to_string((int)result.die_width) + "x" +
                     std::to_string((int)result.die_height) +
                     ", dead space: " + std::to_string((int)result.dead_space_pct) +
                     "%, WL: " + std::to_string((int)result.wirelength) +
                     ", AR: " + std::to_string(result.aspect_ratio).substr(0, 4);
    return result;
}

// ---------------------------------------------------------------------------
// Original methods above are unchanged
// ---------------------------------------------------------------------------

// ── Tier 2: Hierarchical Floorplanning ──────────────────────────────

void Floorplanner::add_module(const std::string& name, const std::vector<int>& macro_ids) {
    HierModule mod;
    mod.name = name;
    mod.macro_ids = macro_ids;
    modules_.push_back(mod);
}

void Floorplanner::set_module_region(const std::string& name, const Rect& region) {
    for (auto& m : modules_) {
        if (m.name == name) {
            m.region = region;
            m.has_region = true;
            return;
        }
    }
}

Floorplanner::HierFloorplanResult Floorplanner::hierarchical_floorplan(
    const HierFloorplanConfig& cfg) {
    HierFloorplanResult result;
    result.num_modules = (int)modules_.size();

    if (modules_.empty()) {
        // No hierarchy — fall back to flat floorplan
        auto flat = floorplan_sa();
        result.total_area = flat.total_area;
        result.wirelength = flat.wirelength;
        result.levels = 1;
        result.message = "Flat floorplan (no modules defined)";
        return result;
    }

    // Level 1: Assign regions to modules based on total area of their macros
    double total_macro_area = 0;
    for (auto& m : macros_) total_macro_area += m.width * m.height;
    double die_side = std::sqrt(total_macro_area / cfg.area_util);

    // Partition die area among modules proportionally
    double y_cursor = 0;
    for (auto& mod : modules_) {
        double mod_area = 0;
        for (int mid : mod.macro_ids) {
            if (mid >= 0 && mid < (int)macros_.size())
                mod_area += macros_[mid].width * macros_[mid].height;
        }
        double mod_height = (mod_area / cfg.area_util) / die_side;
        if (!mod.has_region) {
            mod.region = Rect(0, y_cursor, die_side, y_cursor + mod_height);
            mod.has_region = true;
        }
        y_cursor += mod_height;
    }

    // Level 2: Place macros within each module's region
    double total_wl = 0;
    for (auto& mod : modules_) {
        double region_w = mod.region.width();
        double region_h = mod.region.height();

        // Simple left-bottom packing within region
        double x_cur = mod.region.x0;
        double y_cur = mod.region.y0;
        double row_height = 0;

        for (int mid : mod.macro_ids) {
            if (mid < 0 || mid >= (int)macros_.size()) continue;
            auto& macro = macros_[mid];

            if (x_cur + macro.width > mod.region.x1) {
                x_cur = mod.region.x0;
                y_cur += row_height;
                row_height = 0;
            }

            macro.position.x = x_cur;
            macro.position.y = y_cur;
            x_cur += macro.width;
            row_height = std::max(row_height, macro.height);
        }
    }

    // Compute total wirelength
    total_wl = compute_wirelength();

    result.total_area = die_side * y_cursor;
    result.wirelength = total_wl;
    result.levels = 2;
    result.message = "Hierarchical: " + std::to_string(result.num_modules) +
                     " modules, area=" + std::to_string((int)result.total_area) +
                     ", WL=" + std::to_string((int)total_wl);
    return result;
}

} // namespace sf
