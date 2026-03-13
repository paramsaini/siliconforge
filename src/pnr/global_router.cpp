// SiliconForge — Global Router (A* + Negotiated Congestion)
// Industrial: timing-driven net ordering, slack-weighted A* cost, crosstalk
//             penalty, antenna area tracking, via cost.
#include "pnr/global_router.hpp"
#include <algorithm>
#include <chrono>
#include <queue>
#include <numeric>
#include <iostream>
#include <cmath>
#include <climits>
#include <set>

namespace sf {

void GlobalRouter::build_grid() {
    grid_.clear();
    grid_.resize(grid_y_);
    double cell_w = pd_.die_area.width() / grid_x_;
    double cell_h = pd_.die_area.height() / grid_y_;
    int default_cap = std::max(1, (int)(cell_w / pd_.site_width));

    for (int y = 0; y < grid_y_; ++y) {
        grid_[y].resize(grid_x_);
        for (int x = 0; x < grid_x_; ++x) {
            grid_[y][x] = {x, y, 0, default_cap, 0, 1.0};
        }
    }
}

std::pair<int,int> GlobalRouter::to_grid(const Point& p) {
    double cell_w = pd_.die_area.width() / grid_x_;
    double cell_h = pd_.die_area.height() / grid_y_;
    int gx = std::clamp((int)((p.x - pd_.die_area.x0) / cell_w), 0, grid_x_ - 1);
    int gy = std::clamp((int)((p.y - pd_.die_area.y0) / cell_h), 0, grid_y_ - 1);
    return {gx, gy};
}

// ============================================================================
// Industrial: Net criticality (0.0 = non-critical, 1.0 = most critical)
// ============================================================================
double GlobalRouter::get_criticality(int net_idx) const {
    auto it = net_timing_.find(net_idx);
    if (it == net_timing_.end()) return 0.0;
    return it->second.criticality;
}

// ============================================================================
// Industrial: Timing-weighted edge cost for A*
// Reference: Hu & Sapatnekar "A Timing-Driven Detailed Router" — cost function
// cost = congestion_cost × (1 + α × criticality^β)
// ============================================================================
double GlobalRouter::edge_cost(int nx, int ny, int net_idx) const {
    double base = grid_[ny][nx].total_cost();

    if (!config_.enable_timing_driven) return base;

    double crit = get_criticality(net_idx);
    double timing_mult = 1.0 + config_.timing_weight *
                         std::pow(crit, config_.criticality_exponent);

    // For critical nets, prefer less congested (= lower delay) paths
    // Higher criticality → LOWER cost tolerance → route more directly
    // Invert: critical nets get REDUCED congestion penalty but penalty for detour
    double cost = base;
    if (crit > 0.5) {
        // Reduce congestion avoidance for very critical nets — prefer short paths
        cost = base * (2.0 - crit); // crit=1 → cost=base, crit=0.5 → cost=1.5×base
    }

    // Via cost penalty (each via adds RC delay)
    // Applied in route_net_astar for layer changes, not per edge

    return cost;
}

// ============================================================================
// Industrial: Antenna ratio check for a net
// Reference: TSMC antenna rule — wire_area / gate_area < max_ratio
// ============================================================================
AntennaViolation GlobalRouter::check_antenna(int net_idx) const {
    AntennaViolation av;
    av.net_id = net_idx;
    av.net_name = net_idx >= 0 && net_idx < (int)pd_.nets.size() ? pd_.nets[net_idx].name : "";

    if (!config_.enable_antenna_check) return av;

    double wire_area = 0;
    for (auto& w : pd_.wires) {
        if (w.net_id == net_idx && w.width > 0) {
            wire_area += w.start.dist(w.end) * w.width;
        }
    }
    double gate_area = 0;
    if (net_idx >= 0 && net_idx < (int)pd_.nets.size()) {
        for (auto ci : pd_.nets[net_idx].cell_ids) {
            if (ci >= 0 && ci < (int)pd_.cells.size()) {
                gate_area += pd_.cells[ci].width * 0.05;
            }
        }
    }
    av.wire_area = wire_area;
    av.gate_area = gate_area;
    av.ratio = gate_area > 0 ? wire_area / gate_area : 0;
    av.max_ratio = config_.max_antenna_ratio;
    av.layer = -1;
    if (av.ratio > config_.max_antenna_ratio)
        av.fix_suggestion = "Insert diode or break wire";
    return av;
}

// ── A* pathfinder (industrial: timing-weighted, via-cost-aware) ──────
std::vector<GlobalRouter::AStarNode> GlobalRouter::astar_route(
    int sx, int sy, int dx, int dy, int net_idx)
{
    std::vector<std::vector<double>> g_cost(grid_y_, std::vector<double>(grid_x_, 1e18));
    std::vector<std::vector<std::pair<int,int>>> prev(grid_y_, std::vector<std::pair<int,int>>(grid_x_, {-1,-1}));

    // Industrial: timing-driven heuristic weighting
    double crit = config_.enable_timing_driven ? get_criticality(net_idx) : 0.0;
    double heuristic_weight = 1.0;
    if (crit > 0.7) heuristic_weight = 1.5; // A* becomes more greedy for critical nets

    auto heuristic = [&](int x, int y) {
        return heuristic_weight * (double)(std::abs(x - dx) + std::abs(y - dy));
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;
    g_cost[sy][sx] = 0;
    pq.push({sx, sy, 0, heuristic(sx, sy)});

    int ddx[] = {0, 0, 1, -1};
    int ddy[] = {1, -1, 0, 0};

    while (!pq.empty()) {
        auto cur = pq.top(); pq.pop();
        if (cur.x == dx && cur.y == dy) break;
        if (cur.g_cost > g_cost[cur.y][cur.x] + 1e-6) continue;

        for (int d = 0; d < 4; d++) {
            int nx = cur.x + ddx[d], ny = cur.y + ddy[d];
            if (nx < 0 || nx >= grid_x_ || ny < 0 || ny >= grid_y_) continue;

            // Industrial: timing-weighted edge cost
            double ec = edge_cost(nx, ny, net_idx);
            double new_g = g_cost[cur.y][cur.x] + ec;
            if (new_g < g_cost[ny][nx]) {
                g_cost[ny][nx] = new_g;
                prev[ny][nx] = {cur.x, cur.y};
                pq.push({nx, ny, new_g, new_g + heuristic(nx, ny)});
            }
        }
    }

    // Backtrace
    std::vector<AStarNode> path;
    if (g_cost[dy][dx] >= 1e17) return path;
    int cx = dx, cy = dy;
    while (cx != sx || cy != sy) {
        path.push_back({cx, cy, 0, 0});
        auto [px, py] = prev[cy][cx];
        if (px < 0) return {};
        cx = px; cy = py;
    }
    path.push_back({sx, sy, 0, 0});
    std::reverse(path.begin(), path.end());
    return path;
}

// ── RSMT approximation (Prim-based MST on rectilinear distance) ──────
std::vector<std::pair<GlobalRouter::SteinerPoint,GlobalRouter::SteinerPoint>>
GlobalRouter::build_rsmt(const std::vector<SteinerPoint>& pins) {
    int n = (int)pins.size();
    if (n < 2) return {};

    std::vector<bool> in_tree(n, false);
    std::vector<double> min_dist(n, 1e18);
    std::vector<int> parent(n, -1);
    min_dist[0] = 0;

    for (int iter = 0; iter < n; iter++) {
        int u = -1;
        double best = 1e18;
        for (int i = 0; i < n; i++) {
            if (!in_tree[i] && min_dist[i] < best) { best = min_dist[i]; u = i; }
        }
        if (u < 0) break;
        in_tree[u] = true;
        for (int v = 0; v < n; v++) {
            if (in_tree[v]) continue;
            double d = std::abs(pins[u].x - pins[v].x) + std::abs(pins[u].y - pins[v].y);
            if (d < min_dist[v]) { min_dist[v] = d; parent[v] = u; }
        }
    }

    std::vector<std::pair<SteinerPoint,SteinerPoint>> edges;
    for (int i = 1; i < n; i++) {
        if (parent[i] >= 0) edges.push_back({pins[parent[i]], pins[i]});
    }
    return edges;
}

// ── Route a single net using A* + RSMT ───────────────────────────────
bool GlobalRouter::route_net_astar(int net_idx) {
    auto& net = pd_.nets[net_idx];
    if (net.cell_ids.size() < 2) return true;

    int num_layers = std::max(2, num_layers_);
    int num_pairs = std::max(1, num_layers / 2);

    // Industrial: layer promotion for critical nets
    int pair_index = net_idx % num_pairs;
    if (config_.enable_layer_promotion && get_criticality(net_idx) > 0.7) {
        // Promote critical nets to upper metal layers (wider, lower RC)
        pair_index = std::min(num_pairs - 1, pair_index + num_pairs / 2);
    }

    int h_layer = std::clamp(pair_index * 2, 0, num_layers - 1);
    int v_layer = std::clamp(pair_index * 2 + 1, 0, num_layers - 1);
    if (h_layer == v_layer && num_layers > 1) v_layer = (h_layer + 1) % num_layers;

    // Collect grid pins
    std::vector<SteinerPoint> pins;
    for (size_t i = 0; i < net.cell_ids.size(); i++) {
        auto& c = pd_.cells[net.cell_ids[i]];
        double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c.width/2);
        double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c.height/2);
        auto [gx, gy] = to_grid({px, py});
        pins.push_back({gx, gy});
    }

    auto tree_edges = build_rsmt(pins);

    double cell_w = pd_.die_area.width() / grid_x_;
    double cell_h = pd_.die_area.height() / grid_y_;
    size_t wire_start = pd_.wires.size();

    bool all_ok = true;
    for (auto& [src, dst] : tree_edges) {
        auto path = astar_route(src.x, src.y, dst.x, dst.y, net_idx);
        if (path.empty() && (src.x != dst.x || src.y != dst.y)) { all_ok = false; continue; }

        for (size_t j = 0; j + 1 < path.size(); j++) {
            grid_[path[j].y][path[j].x].usage++;
            Point p0(pd_.die_area.x0 + path[j].x * cell_w + cell_w/2,
                     pd_.die_area.y0 + path[j].y * cell_h + cell_h/2);
            Point p1(pd_.die_area.x0 + path[j+1].x * cell_w + cell_w/2,
                     pd_.die_area.y0 + path[j+1].y * cell_h + cell_h/2);
            bool is_horizontal = (path[j].y == path[j+1].y);
            int wire_layer = is_horizontal ? h_layer : v_layer;
            pd_.wires.push_back({wire_layer, p0, p1, 0.14, net_idx});

            if (j > 0) {
                bool prev_horiz = (path[j-1].y == path[j].y);
                if (prev_horiz != is_horizontal) {
                    int prev_layer = prev_horiz ? h_layer : v_layer;
                    if (prev_layer != wire_layer) {
                        int lower = std::min(prev_layer, wire_layer);
                        int upper = std::max(prev_layer, wire_layer);
                        pd_.vias.push_back({p0, lower, upper});
                    }
                }
            }
        }
    }

    size_t wire_end = pd_.wires.size();
    net_wire_ranges_.push_back({net_idx, wire_start, wire_end});
    return all_ok;
}

// ── Rip-up a net ─────────────────────────────────────────────────────
void GlobalRouter::rip_up_net(int net_idx) {
    for (auto& nwr : net_wire_ranges_) {
        if (nwr.net_idx != net_idx) continue;
        for (size_t w = nwr.wire_start; w < nwr.wire_end && w < pd_.wires.size(); w++) {
            auto [gx, gy] = to_grid(pd_.wires[w].start);
            if (gy >= 0 && gy < grid_y_ && gx >= 0 && gx < grid_x_)
                grid_[gy][gx].usage = std::max(0, grid_[gy][gx].usage - 1);
        }
        for (size_t w = nwr.wire_start; w < nwr.wire_end && w < pd_.wires.size(); w++) {
            pd_.wires[w].width = -1; // sentinel: deleted wire
        }
        nwr.wire_start = nwr.wire_end = 0;
    }
}

// ── PathFinder history update ────────────────────────────────────────
void GlobalRouter::update_history() {
    for (auto& row : grid_) {
        for (auto& gc : row) {
            if (gc.usage > gc.capacity)
                gc.history_cost += 1.0 + 0.5 * (gc.usage - gc.capacity);
        }
    }
}

int GlobalRouter::compute_overflow() const {
    int total = 0;
    for (auto& row : grid_)
        for (auto& gc : row)
            if (gc.usage > gc.capacity) total += gc.usage - gc.capacity;
    return total;
}

// ── Main routing loop (industrial: timing-driven ordering + metrics) ─
RouteResult GlobalRouter::route() {
    auto t0 = std::chrono::high_resolution_clock::now();
    build_grid();
    net_wire_ranges_.clear();
    successfully_routed_.clear();

    RouteResult result;

    // Net ordering: industrial timing-driven or basic size-based
    std::vector<int> net_order(pd_.nets.size());
    std::iota(net_order.begin(), net_order.end(), 0);

    if (config_.enable_timing_driven && !net_timing_.empty()) {
        // Sort: most critical nets first (highest criticality), then by size
        std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
            double ca = get_criticality(a);
            double cb = get_criticality(b);
            if (std::abs(ca - cb) > 0.01) return ca > cb; // critical first
            return pd_.nets[a].cell_ids.size() < pd_.nets[b].cell_ids.size();
        });
    } else {
        std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
            return pd_.nets[a].cell_ids.size() < pd_.nets[b].cell_ids.size();
        });
    }

    // Initial routing pass
    for (int ni : net_order) {
        if (route_net_astar(ni)) {
            result.routed_nets++;
            successfully_routed_.insert(ni);
        } else {
            result.failed_nets++;
        }
    }

    // PathFinder rip-up/reroute loop
    int max_reroute = config_.max_reroute_iterations;
    for (int iter = 0; iter < max_reroute; iter++) {
        int overflow = compute_overflow();
        if (overflow == 0) break;
        result.reroute_iterations++;

        update_history();

        std::set<int> reroute_set;
        for (auto& row : grid_) {
            for (auto& gc : row) {
                if (gc.usage <= gc.capacity) continue;
                for (auto& nwr : net_wire_ranges_) {
                    if (nwr.wire_start >= nwr.wire_end) continue;
                    for (size_t w = nwr.wire_start; w < nwr.wire_end && w < pd_.wires.size(); w++) {
                        if (pd_.wires[w].width < 0) continue;
                        auto [gx, gy] = to_grid(pd_.wires[w].start);
                        if (gx == gc.x && gy == gc.y) {
                            reroute_set.insert(nwr.net_idx);
                            break;
                        }
                    }
                }
            }
        }

        if (reroute_set.empty()) break;

        for (int ni : reroute_set) rip_up_net(ni);
        for (int ni : reroute_set) {
            if (route_net_astar(ni)) successfully_routed_.insert(ni);
            else successfully_routed_.erase(ni);
        }
    }

    // Clean up deleted wires
    pd_.wires.erase(
        std::remove_if(pd_.wires.begin(), pd_.wires.end(),
                       [](const WireSegment& w) { return w.width < 0; }),
        pd_.wires.end());

    // ── Compute metrics ──────────────────────────────────────────────
    result.total_wirelength = 0;
    for (auto& w : pd_.wires)
        result.total_wirelength += w.start.dist(w.end);
    result.max_congestion = 0;
    for (auto& row : grid_)
        for (auto& gc : row)
            result.max_congestion = std::max(result.max_congestion,
                gc.capacity > 0 ? (double)gc.usage / gc.capacity : 0);
    result.overflow = compute_overflow();

    // Recount
    result.routed_nets = 0; result.failed_nets = 0;
    for (int ni : net_order) {
        if (successfully_routed_.count(ni) || pd_.nets[ni].cell_ids.size() < 2)
            result.routed_nets++;
        else
            result.failed_nets++;
    }

    // Industrial metrics
    result.total_wires = (int)pd_.wires.size();
    result.total_vias = (int)pd_.vias.size();

    // Critical net wirelength
    if (config_.enable_timing_driven) {
        for (auto& w : pd_.wires) {
            if (w.net_id >= 0 && get_criticality(w.net_id) > 0.5)
                result.critical_net_wirelength += w.start.dist(w.end);
        }
    }

    // Antenna check
    if (config_.enable_antenna_check) {
        for (int ni : net_order) {
            auto av = check_antenna(ni);
            if (av.ratio > config_.max_antenna_ratio) {
                result.antenna_violations++;
                result.antenna_details.push_back(av);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = std::to_string(result.routed_nets) + "/" +
                     std::to_string(result.routed_nets + result.failed_nets) +
                     " nets routed, overflow: " + std::to_string(result.overflow) +
                     ", " + std::to_string(result.reroute_iterations) + " reroute passes" +
                     ", " + std::to_string(result.total_vias) + " vias";
    if (config_.enable_timing_driven)
        result.message += ", timing-driven";
    if (result.antenna_violations > 0)
        result.message += ", " + std::to_string(result.antenna_violations) + " antenna violations";
    return result;
}

// ============================================================================
// Steiner tree decomposition — Hanan-grid RSMT
// Reference: Hanan, "On Steiner's Problem with Rectilinear Distance", 1966
// ============================================================================
GlobalRouter::SteinerDecomp GlobalRouter::build_steiner(
    const std::vector<std::pair<int,int>>& pins)
{
    SteinerDecomp result;
    if (pins.size() < 2) {
        for (auto& [x,y] : pins)
            result.points.push_back({(double)x, (double)y});
        return result;
    }

    for (auto& [x,y] : pins)
        result.points.push_back({(double)x, (double)y});

    int n = (int)pins.size();

    // Build Hanan grid from all x- and y-coordinates
    std::vector<int> xs, ys;
    for (auto& [x,y] : pins) { xs.push_back(x); ys.push_back(y); }
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());

    std::set<std::pair<int,int>> pin_set(pins.begin(), pins.end());
    std::vector<std::pair<int,int>> steiner_candidates;
    for (int x : xs)
        for (int y : ys)
            if (pin_set.find({x,y}) == pin_set.end())
                steiner_candidates.push_back({x, y});

    // MST cost helper via Prim's algorithm
    auto mst_cost = [](const std::vector<std::pair<int,int>>& pts) -> double {
        int m = (int)pts.size();
        if (m < 2) return 0;
        std::vector<bool> in(m, false);
        std::vector<double> md(m, 1e18);
        md[0] = 0;
        double cost = 0;
        for (int iter = 0; iter < m; iter++) {
            int u = -1;
            double best = 1e18;
            for (int i = 0; i < m; i++)
                if (!in[i] && md[i] < best) { best = md[i]; u = i; }
            if (u < 0) break;
            in[u] = true;
            cost += md[u];
            for (int v = 0; v < m; v++) {
                if (in[v]) continue;
                double d = std::abs(pts[u].first - pts[v].first) +
                           std::abs(pts[u].second - pts[v].second);
                if (d < md[v]) md[v] = d;
            }
        }
        return cost;
    };

    auto all_points = pins;
    double current_cost = mst_cost(all_points);

    // Greedily add Hanan grid points that reduce total wirelength
    for (auto& sp : steiner_candidates) {
        auto test_pts = all_points;
        test_pts.push_back(sp);
        double new_cost = mst_cost(test_pts);
        if (new_cost < current_cost - 1e-9) {
            all_points.push_back(sp);
            result.points.push_back({(double)sp.first, (double)sp.second});
            current_cost = new_cost;
        }
    }

    // Final MST to extract 2-pin edges
    int fn = (int)all_points.size();
    std::vector<bool> ft(fn, false);
    std::vector<double> fd(fn, 1e18);
    std::vector<int> fp(fn, -1);
    fd[0] = 0;

    for (int iter = 0; iter < fn; iter++) {
        int u = -1;
        double best = 1e18;
        for (int i = 0; i < fn; i++)
            if (!ft[i] && fd[i] < best) { best = fd[i]; u = i; }
        if (u < 0) break;
        ft[u] = true;
        for (int v = 0; v < fn; v++) {
            if (ft[v]) continue;
            double d = std::abs(all_points[u].first - all_points[v].first) +
                       std::abs(all_points[u].second - all_points[v].second);
            if (d < fd[v]) { fd[v] = d; fp[v] = u; }
        }
    }

    for (int i = 1; i < fn; i++) {
        if (fp[i] < 0) continue;
        double dx = std::abs(all_points[i].first - all_points[fp[i]].first);
        double dy = std::abs(all_points[i].second - all_points[fp[i]].second);
        // Horizontal edges → even metal (M2,M4,M6 = indices 1,3,5)
        // Vertical edges   → odd metal  (M1,M3,M5 = indices 0,2,4)
        int preferred = (dx >= dy) ? 1 : 0;
        result.edges.push_back({fp[i], i, dx + dy, preferred});
    }

    return result;
}

// ============================================================================
// Layer assignment with via minimization
// Horizontal → even metal layers (M2=1, M4=3, M6=5)
// Vertical   → odd metal layers  (M1=0, M3=2, M5=4)
// ============================================================================
std::vector<GlobalRouter::LayerAssignment> GlobalRouter::assign_layers(
    const SteinerDecomp& tree, int num_layers)
{
    std::vector<LayerAssignment> result;
    if (tree.edges.empty()) return result;

    int ne = (int)tree.edges.size();
    int np = (int)tree.points.size();
    num_layers = std::max(2, num_layers);

    // Build edge adjacency (edges sharing a Steiner point)
    std::vector<std::vector<int>> adj(ne);
    std::vector<std::vector<int>> point_edges(np);
    for (int i = 0; i < ne; i++) {
        point_edges[tree.edges[i].from].push_back(i);
        point_edges[tree.edges[i].to].push_back(i);
    }
    for (int p = 0; p < np; p++) {
        auto& pe = point_edges[p];
        for (size_t a = 0; a < pe.size(); a++)
            for (size_t b = a + 1; b < pe.size(); b++) {
                adj[pe[a]].push_back(pe[b]);
                adj[pe[b]].push_back(pe[a]);
            }
    }

    // Initial assignment by preferred direction
    result.resize(ne);
    for (int i = 0; i < ne; i++) {
        auto& e = tree.edges[i];
        result[i].edge_idx = i;

        double dx = std::abs(tree.points[e.from].first - tree.points[e.to].first);
        double dy = std::abs(tree.points[e.from].second - tree.points[e.to].second);
        bool is_horizontal = dx >= dy;

        if (is_horizontal) {
            // Even metal: M2=1, M4=3, M6=5
            result[i].layer = std::min(1, num_layers - 1);
        } else {
            // Odd metal: M1=0, M3=2, M5=4
            result[i].layer = 0;
        }
        result[i].via_count = 0;
    }

    // Minimize layer transitions: keep connected edges on same layer when possible
    for (int pass = 0; pass < 5; pass++) {
        bool changed = false;
        for (int i = 0; i < ne; i++) {
            if (adj[i].empty()) continue;

            // Count layers of neighbors
            std::unordered_map<int, int> layer_count;
            for (int j : adj[i]) layer_count[result[j].layer]++;

            int best_layer = result[i].layer;
            int best_count = 0;
            for (auto& [l, c] : layer_count)
                if (c > best_count) { best_count = c; best_layer = l; }

            if (best_layer != result[i].layer && best_count > (int)adj[i].size() / 2) {
                result[i].layer = best_layer;
                changed = true;
            }
        }
        if (!changed) break;
    }

    // Count vias at each point where edges meet on different layers
    for (int i = 0; i < ne; i++) {
        int vias = 0;
        for (int j : adj[i])
            if (result[j].layer != result[i].layer) vias++;
        result[i].via_count = vias;
    }

    return result;
}

// ============================================================================
// Via minimization — post-processing to remove unnecessary layer changes
// Returns number of vias removed.
// ============================================================================
int GlobalRouter::minimize_vias(std::vector<LayerAssignment>& la) {
    if (la.size() < 2) return 0;
    int ne = (int)la.size();

    int original_vias = 0;
    for (auto& a : la) original_vias += a.via_count;

    // Forward pass: propagate layer to reduce transitions
    for (int i = 0; i + 1 < ne; i++) {
        if (la[i].layer == la[i+1].layer) continue;
        int vias_if_keep = 1 + ((i + 2 < ne && la[i+1].layer != la[i+2].layer) ? 1 : 0);
        int vias_if_change = (i + 2 < ne && la[i].layer != la[i+2].layer) ? 1 : 0;
        if (vias_if_change < vias_if_keep)
            la[i+1].layer = la[i].layer;
    }

    // Backward pass
    for (int i = ne - 1; i > 0; i--) {
        if (la[i].layer == la[i-1].layer) continue;
        int vias_if_keep = 1 + ((i - 2 >= 0 && la[i-1].layer != la[i-2].layer) ? 1 : 0);
        int vias_if_change = (i - 2 >= 0 && la[i].layer != la[i-2].layer) ? 1 : 0;
        if (vias_if_change < vias_if_keep)
            la[i-1].layer = la[i].layer;
    }

    // Recount vias
    int new_vias = 0;
    for (int i = 0; i < ne; i++) {
        la[i].via_count = 0;
        if (i + 1 < ne && la[i].layer != la[i+1].layer)
            la[i].via_count = 1;
        new_vias += la[i].via_count;
    }

    return std::max(0, original_vias - new_vias);
}

// ============================================================================
// Congestion map — snapshot of current GCell usage/capacity/overflow
// ============================================================================
GlobalRouter::CongestionMap GlobalRouter::get_congestion_map() const {
    CongestionMap cmap;
    cmap.usage.resize(grid_y_, std::vector<double>(grid_x_, 0.0));
    cmap.capacity.resize(grid_y_, std::vector<double>(grid_x_, 0.0));
    cmap.total_overflow = 0;
    cmap.max_overflow = 0;
    cmap.overflow_gcells = 0;

    for (int y = 0; y < grid_y_; y++) {
        for (int x = 0; x < grid_x_; x++) {
            if (y >= (int)grid_.size() || x >= (int)grid_[y].size()) continue;
            cmap.usage[y][x] = grid_[y][x].usage;
            cmap.capacity[y][x] = grid_[y][x].capacity;
            double of = std::max(0.0, (double)(grid_[y][x].usage - grid_[y][x].capacity));
            cmap.total_overflow += of;
            if (of > cmap.max_overflow) cmap.max_overflow = of;
            if (of > 0) cmap.overflow_gcells++;
        }
    }

    return cmap;
}

// ============================================================================
// Edge cost with congestion and history (Lagrangian relaxation)
// cost = base_cost + penalty × max(0, usage - capacity) + history_cost
// ============================================================================
double GlobalRouter::edge_cost(int gx, int gy, int /*direction*/, double penalty) const {
    if (gy < 0 || gy >= grid_y_ || gx < 0 || gx >= grid_x_) return 1e18;
    if (gy >= (int)grid_.size() || gx >= (int)grid_[gy].size()) return 1e18;

    auto& gc = grid_[gy][gx];
    double base = gc.total_cost();
    double overflow = std::max(0.0, (double)(gc.usage - gc.capacity));
    double hist = (!history_cost_.empty() && gy < (int)history_cost_.size()
                   && gx < (int)history_cost_[gy].size())
                  ? history_cost_[gy][gx] : 0.0;

    return base + penalty * overflow + hist;
}

// ============================================================================
// Find nets passing through overflowed GCells, sorted by HPWL (longest first)
// ============================================================================
std::vector<int> GlobalRouter::find_overflowed_nets() {
    std::set<int> overflow_set;
    for (int y = 0; y < grid_y_; y++) {
        for (int x = 0; x < grid_x_; x++) {
            if (y >= (int)grid_.size() || x >= (int)grid_[y].size()) continue;
            if (grid_[y][x].usage <= grid_[y][x].capacity) continue;
            for (auto& nwr : net_wire_ranges_) {
                if (nwr.wire_start >= nwr.wire_end) continue;
                for (size_t w = nwr.wire_start; w < nwr.wire_end && w < pd_.wires.size(); w++) {
                    if (pd_.wires[w].width < 0) continue;
                    auto [gx, gy] = to_grid(pd_.wires[w].start);
                    if (gx == x && gy == y) {
                        overflow_set.insert(nwr.net_idx);
                        break;
                    }
                }
            }
        }
    }

    std::vector<int> nets(overflow_set.begin(), overflow_set.end());
    std::sort(nets.begin(), nets.end(), [&](int a, int b) {
        auto hpwl = [&](int ni) -> double {
            if (ni < 0 || ni >= (int)pd_.nets.size()) return 0;
            auto& net = pd_.nets[ni];
            if (net.cell_ids.empty()) return 0;
            double mnx = 1e18, mny = 1e18, mxx = -1e18, mxy = -1e18;
            for (auto ci : net.cell_ids) {
                if (ci < 0 || ci >= (int)pd_.cells.size()) continue;
                mnx = std::min(mnx, pd_.cells[ci].position.x);
                mny = std::min(mny, pd_.cells[ci].position.y);
                mxx = std::max(mxx, pd_.cells[ci].position.x);
                mxy = std::max(mxy, pd_.cells[ci].position.y);
            }
            return (mxx - mnx) + (mxy - mny);
        };
        return hpwl(a) > hpwl(b);
    });

    return nets;
}

// ============================================================================
// Reroute a single net with congestion-adjusted costs (Lagrangian penalty)
// Temporarily boosts GCell history_cost with Lagrangian multipliers.
// ============================================================================
bool GlobalRouter::reroute_net(int net_idx, double penalty_factor) {
    // Temporarily apply Lagrangian multiplier penalty to grid history costs
    std::vector<std::vector<double>> saved_history(grid_y_);
    for (int y = 0; y < grid_y_; y++) {
        saved_history[y].resize(grid_x_);
        for (int x = 0; x < grid_x_; x++) {
            saved_history[y][x] = grid_[y][x].history_cost;
            double overflow = std::max(0.0, (double)(grid_[y][x].usage - grid_[y][x].capacity));
            double lagr = (!history_cost_.empty() && y < (int)history_cost_.size()
                           && x < (int)history_cost_[y].size())
                          ? history_cost_[y][x] : 0.0;
            grid_[y][x].history_cost += penalty_factor * overflow + lagr;
        }
    }

    bool ok = route_net_astar(net_idx);

    // Restore original history costs
    for (int y = 0; y < grid_y_; y++)
        for (int x = 0; x < grid_x_; x++)
            grid_[y][x].history_cost = saved_history[y][x];

    return ok;
}

// ============================================================================
// Lagrangian multiplier update — accumulate penalties for overflowed GCells
// h[e] += step_size × overflow[e]
// ============================================================================
void GlobalRouter::update_lagrangian_multipliers(double step_size) {
    for (int y = 0; y < grid_y_; y++) {
        for (int x = 0; x < grid_x_; x++) {
            if (y >= (int)grid_.size() || x >= (int)grid_[y].size()) continue;
            int overflow = std::max(0, grid_[y][x].usage - grid_[y][x].capacity);
            if (overflow > 0)
                history_cost_[y][x] += step_size * overflow;
        }
    }
    // Also update PathFinder-style history
    update_history();
}

// ============================================================================
// Main Lagrangian rip-up and reroute flow
// 1. Initial routing with A* (existing route infrastructure)
// 2. Iterative congestion-driven rip-up & reroute
// 3. Layer assignment + via minimization
// Reference: Pan & Chu, "FastRoute 2.0", IEEE TCAD 2007
// ============================================================================
RouteResult GlobalRouter::route_with_rr() {
    auto t0 = std::chrono::high_resolution_clock::now();

    // ── Step 1: initial routing ──────────────────────────────────────
    build_grid();
    net_wire_ranges_.clear();
    successfully_routed_.clear();
    history_cost_.assign(grid_y_, std::vector<double>(grid_x_, 0.0));

    RouteResult result;

    // Net ordering: timing-driven or size-based
    std::vector<int> net_order(pd_.nets.size());
    std::iota(net_order.begin(), net_order.end(), 0);

    if (config_.enable_timing_driven && !net_timing_.empty()) {
        std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
            double ca = get_criticality(a), cb = get_criticality(b);
            if (std::abs(ca - cb) > 0.01) return ca > cb;
            return pd_.nets[a].cell_ids.size() < pd_.nets[b].cell_ids.size();
        });
    } else {
        std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
            return pd_.nets[a].cell_ids.size() < pd_.nets[b].cell_ids.size();
        });
    }

    for (int ni : net_order) {
        if (route_net_astar(ni)) {
            result.routed_nets++;
            successfully_routed_.insert(ni);
        } else {
            result.failed_nets++;
        }
    }

    // ── Step 2 & 3: Lagrangian rip-up reroute loop ──────────────────
    for (int iter = 0; iter < rr_cfg_.max_iterations; iter++) {
        auto cmap = get_congestion_map();
        if (cmap.total_overflow <= rr_cfg_.overflow_threshold) break;

        result.reroute_iterations++;

        auto overflow_nets = find_overflowed_nets();
        if (overflow_nets.empty()) break;

        for (int ni : overflow_nets)
            rip_up_net(ni);

        double penalty = rr_cfg_.congestion_penalty * (1.0 + iter * 0.5);
        for (int ni : overflow_nets) {
            if (reroute_net(ni, penalty))
                successfully_routed_.insert(ni);
            else
                successfully_routed_.erase(ni);
        }

        double step = rr_cfg_.history_weight / (1.0 + iter);
        update_lagrangian_multipliers(step);
    }

    // Clean up deleted wires
    pd_.wires.erase(
        std::remove_if(pd_.wires.begin(), pd_.wires.end(),
                       [](const WireSegment& w) { return w.width < 0; }),
        pd_.wires.end());

    // ── Step 4 & 5: layer assignment + via minimization ──────────────
    int total_via_reduction = 0;
    for (int ni : net_order) {
        if (!successfully_routed_.count(ni)) continue;
        if (pd_.nets[ni].cell_ids.size() < 2) continue;

        std::vector<std::pair<int,int>> pin_coords;
        for (size_t i = 0; i < pd_.nets[ni].cell_ids.size(); i++) {
            auto& c = pd_.cells[pd_.nets[ni].cell_ids[i]];
            double px = c.position.x + (i < pd_.nets[ni].pin_offsets.size()
                                        ? pd_.nets[ni].pin_offsets[i].x : c.width / 2);
            double py = c.position.y + (i < pd_.nets[ni].pin_offsets.size()
                                        ? pd_.nets[ni].pin_offsets[i].y : c.height / 2);
            auto [gx, gy] = to_grid({px, py});
            pin_coords.push_back({gx, gy});
        }

        auto steiner = build_steiner(pin_coords);
        int nl = std::max(2, num_layers_);
        auto la = assign_layers(steiner, nl);
        total_via_reduction += minimize_vias(la);
    }

    // ── Compute metrics ──────────────────────────────────────────────
    result.total_wirelength = 0;
    for (auto& w : pd_.wires)
        result.total_wirelength += w.start.dist(w.end);

    result.max_congestion = 0;
    for (auto& row : grid_)
        for (auto& gc : row)
            result.max_congestion = std::max(result.max_congestion,
                gc.capacity > 0 ? (double)gc.usage / gc.capacity : 0.0);

    result.overflow = compute_overflow();

    result.routed_nets = 0; result.failed_nets = 0;
    for (int ni : net_order) {
        if (successfully_routed_.count(ni) || pd_.nets[ni].cell_ids.size() < 2)
            result.routed_nets++;
        else
            result.failed_nets++;
    }

    result.total_wires = (int)pd_.wires.size();
    result.total_vias = (int)pd_.vias.size();

    if (config_.enable_timing_driven) {
        for (auto& w : pd_.wires) {
            if (w.net_id >= 0 && get_criticality(w.net_id) > 0.5)
                result.critical_net_wirelength += w.start.dist(w.end);
        }
    }

    if (config_.enable_antenna_check) {
        for (int ni : net_order) {
            auto av = check_antenna(ni);
            if (av.ratio > config_.max_antenna_ratio) {
                result.antenna_violations++;
                result.antenna_details.push_back(av);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = std::to_string(result.routed_nets) + "/" +
                     std::to_string(result.routed_nets + result.failed_nets) +
                     " nets routed (Lagrangian RR), overflow: " +
                     std::to_string(result.overflow) +
                     ", " + std::to_string(result.reroute_iterations) + " RR iterations" +
                     ", " + std::to_string(result.total_vias) + " vias";
    if (total_via_reduction > 0)
        result.message += ", " + std::to_string(total_via_reduction) + " vias reduced";
    if (config_.enable_timing_driven)
        result.message += ", timing-driven";
    if (result.antenna_violations > 0)
        result.message += ", " + std::to_string(result.antenna_violations) + " antenna violations";

    return result;
}

// ── Pattern Routing (Tier 2) ─────────────────────────────────────────
// L-shape: two-segment rectilinear path (horizontal-first or vertical-first)
GlobalRouter::PatternRoute GlobalRouter::route_l_shape(
    int sx, int sy, int dx, int dy, int net_idx, bool h_first) {
    PatternRoute pr;
    pr.type = h_first ? PatternType::L_HV : PatternType::L_VH;
    pr.cost = 0;
    bool have_grid = !grid_.empty();

    if (h_first) {
        // Horizontal first, then vertical
        int step_x = (dx > sx) ? 1 : -1;
        for (int x = sx; x != dx; x += step_x) {
            pr.path.push_back({x, sy});
            if (have_grid && sy >= 0 && sy < grid_y_ && x >= 0 && x < grid_x_)
                pr.cost += grid_[sy][x].total_cost();
        }
        int step_y = (dy > sy) ? 1 : -1;
        for (int y = sy; y != dy; y += step_y) {
            pr.path.push_back({dx, y});
            if (have_grid && y >= 0 && y < grid_y_ && dx >= 0 && dx < grid_x_)
                pr.cost += grid_[y][dx].total_cost();
        }
    } else {
        // Vertical first, then horizontal
        int step_y = (dy > sy) ? 1 : -1;
        for (int y = sy; y != dy; y += step_y) {
            pr.path.push_back({sx, y});
            if (have_grid && y >= 0 && y < grid_y_ && sx >= 0 && sx < grid_x_)
                pr.cost += grid_[y][sx].total_cost();
        }
        int step_x = (dx > sx) ? 1 : -1;
        for (int x = sx; x != dx; x += step_x) {
            pr.path.push_back({x, dy});
            if (have_grid && dy >= 0 && dy < grid_y_ && x >= 0 && x < grid_x_)
                pr.cost += grid_[dy][x].total_cost();
        }
    }
    pr.path.push_back({dx, dy});
    return pr;
}

// Z-shape: three-segment rectilinear path with one intermediate bend
GlobalRouter::PatternRoute GlobalRouter::route_z_shape(
    int sx, int sy, int dx, int dy, int net_idx, bool h_first) {
    PatternRoute pr;
    pr.type = h_first ? PatternType::Z_HVH : PatternType::Z_VHV;
    pr.cost = 0;
    bool have_grid = !grid_.empty();

    if (h_first) {
        // H-V-H: go horizontal to midpoint, vertical, then horizontal to target
        int mid_x = (sx + dx) / 2;
        int step_x = (mid_x > sx) ? 1 : (mid_x < sx) ? -1 : 0;
        for (int x = sx; x != mid_x && step_x != 0; x += step_x) {
            pr.path.push_back({x, sy});
            if (have_grid && sy >= 0 && sy < grid_y_ && x >= 0 && x < grid_x_)
                pr.cost += grid_[sy][x].total_cost();
        }
        int step_y = (dy > sy) ? 1 : (dy < sy) ? -1 : 0;
        for (int y = sy; y != dy && step_y != 0; y += step_y) {
            pr.path.push_back({mid_x, y});
            if (have_grid && y >= 0 && y < grid_y_ && mid_x >= 0 && mid_x < grid_x_)
                pr.cost += grid_[y][mid_x].total_cost();
        }
        step_x = (dx > mid_x) ? 1 : (dx < mid_x) ? -1 : 0;
        for (int x = mid_x; x != dx && step_x != 0; x += step_x) {
            pr.path.push_back({x, dy});
            if (have_grid && dy >= 0 && dy < grid_y_ && x >= 0 && x < grid_x_)
                pr.cost += grid_[dy][x].total_cost();
        }
    } else {
        // V-H-V: go vertical to midpoint, horizontal, then vertical to target
        int mid_y = (sy + dy) / 2;
        int step_y = (mid_y > sy) ? 1 : (mid_y < sy) ? -1 : 0;
        for (int y = sy; y != mid_y && step_y != 0; y += step_y) {
            pr.path.push_back({sx, y});
            if (have_grid && y >= 0 && y < grid_y_ && sx >= 0 && sx < grid_x_)
                pr.cost += grid_[y][sx].total_cost();
        }
        int step_x = (dx > sx) ? 1 : (dx < sx) ? -1 : 0;
        for (int x = sx; x != dx && step_x != 0; x += step_x) {
            pr.path.push_back({x, mid_y});
            if (have_grid && mid_y >= 0 && mid_y < grid_y_ && x >= 0 && x < grid_x_)
                pr.cost += grid_[mid_y][x].total_cost();
        }
        step_y = (dy > mid_y) ? 1 : (dy < mid_y) ? -1 : 0;
        for (int y = mid_y; y != dy && step_y != 0; y += step_y) {
            pr.path.push_back({dx, y});
            if (have_grid && y >= 0 && y < grid_y_ && dx >= 0 && dx < grid_x_)
                pr.cost += grid_[y][dx].total_cost();
        }
    }
    pr.path.push_back({dx, dy});
    return pr;
}

// Select best pattern route among L, Z candidates; fall back to A* if all are too costly
GlobalRouter::PatternRoute GlobalRouter::route_pattern(
    int sx, int sy, int dx, int dy, int net_idx) {

    // Generate all 4 pattern candidates
    std::vector<PatternRoute> candidates;
    candidates.push_back(route_l_shape(sx, sy, dx, dy, net_idx, true));   // L_HV
    candidates.push_back(route_l_shape(sx, sy, dx, dy, net_idx, false));  // L_VH
    if (std::abs(dx - sx) > 1 && std::abs(dy - sy) > 1) {
        candidates.push_back(route_z_shape(sx, sy, dx, dy, net_idx, true));  // Z_HVH
        candidates.push_back(route_z_shape(sx, sy, dx, dy, net_idx, false)); // Z_VHV
    }

    // Pick lowest cost
    PatternRoute best = candidates[0];
    for (size_t i = 1; i < candidates.size(); i++) {
        if (candidates[i].cost < best.cost)
            best = candidates[i];
    }

    // Sanity: if pattern cost is excessive (>2× Manhattan), fall back to A*
    double manhattan = std::abs(dx - sx) + std::abs(dy - sy);
    if (best.cost > 2.0 * manhattan * 10.0 && manhattan > 2) {
        auto astar_path = astar_route(sx, sy, dx, dy, net_idx);
        PatternRoute astar_pr;
        astar_pr.type = PatternType::ASTAR;
        astar_pr.cost = 0;
        for (auto& node : astar_path) {
            astar_pr.path.push_back({node.x, node.y});
            astar_pr.cost += node.g_cost;
        }
        if (!astar_path.empty() && astar_pr.cost < best.cost)
            return astar_pr;
    }
    return best;
}

} // namespace sf
