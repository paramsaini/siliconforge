// SiliconForge — Global Router (A* + Negotiated Congestion)
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

// ── A* pathfinder ────────────────────────────────────────────────────
std::vector<GlobalRouter::AStarNode> GlobalRouter::astar_route(int sx, int sy, int dx, int dy) {
    std::vector<std::vector<double>> g_cost(grid_y_, std::vector<double>(grid_x_, 1e18));
    std::vector<std::vector<std::pair<int,int>>> prev(grid_y_, std::vector<std::pair<int,int>>(grid_x_, {-1,-1}));

    auto heuristic = [&](int x, int y) {
        return (double)(std::abs(x - dx) + std::abs(y - dy)); // Manhattan
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;
    g_cost[sy][sx] = 0;
    pq.push({sx, sy, 0, heuristic(sx, sy)});

    int ddx[] = {0, 0, 1, -1};
    int ddy[] = {1, -1, 0, 0};

    while (!pq.empty()) {
        auto cur = pq.top(); pq.pop();
        if (cur.x == dx && cur.y == dy) break;
        if (cur.g_cost > g_cost[cur.y][cur.x] + 1e-6) continue; // stale

        for (int d = 0; d < 4; d++) {
            int nx = cur.x + ddx[d], ny = cur.y + ddy[d];
            if (nx < 0 || nx >= grid_x_ || ny < 0 || ny >= grid_y_) continue;

            double edge_cost = grid_[ny][nx].total_cost();
            double new_g = g_cost[cur.y][cur.x] + edge_cost;
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

    // Prim's MST on Manhattan distances
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
    int pair_index = net_idx % num_pairs;
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

    // Build RSMT for multi-pin decomposition
    auto tree_edges = build_rsmt(pins);

    double cell_w = pd_.die_area.width() / grid_x_;
    double cell_h = pd_.die_area.height() / grid_y_;
    size_t wire_start = pd_.wires.size();

    bool all_ok = true;
    for (auto& [src, dst] : tree_edges) {
        auto path = astar_route(src.x, src.y, dst.x, dst.y);
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

// ── Rip-up a net (remove its wires and free grid usage) ──────────────
void GlobalRouter::rip_up_net(int net_idx) {
    double cell_w = pd_.die_area.width() / grid_x_;
    double cell_h = pd_.die_area.height() / grid_y_;

    for (auto& nwr : net_wire_ranges_) {
        if (nwr.net_idx != net_idx) continue;
        for (size_t w = nwr.wire_start; w < nwr.wire_end && w < pd_.wires.size(); w++) {
            auto [gx, gy] = to_grid(pd_.wires[w].start);
            if (gy >= 0 && gy < grid_y_ && gx >= 0 && gx < grid_x_)
                grid_[gy][gx].usage = std::max(0, grid_[gy][gx].usage - 1);
        }
        // Mark for deletion (but don't actually delete to preserve indices)
        for (size_t w = nwr.wire_start; w < nwr.wire_end && w < pd_.wires.size(); w++) {
            pd_.wires[w].width = -1; // sentinel: deleted wire
        }
        nwr.wire_start = nwr.wire_end = 0; // invalidate range
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

// ── Main routing loop ────────────────────────────────────────────────
RouteResult GlobalRouter::route() {
    auto t0 = std::chrono::high_resolution_clock::now();
    build_grid();
    net_wire_ranges_.clear();
    successfully_routed_.clear();

    RouteResult result;

    // Sort nets by size (smaller first for better initial routing)
    std::vector<int> net_order(pd_.nets.size());
    std::iota(net_order.begin(), net_order.end(), 0);
    std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
        return pd_.nets[a].cell_ids.size() < pd_.nets[b].cell_ids.size();
    });

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
    int max_reroute = 15;
    for (int iter = 0; iter < max_reroute; iter++) {
        int overflow = compute_overflow();
        if (overflow == 0) break;
        result.reroute_iterations++;

        update_history();

        // Find overflowed gcells, rip-up nets passing through them
        std::set<int> reroute_set;
        for (auto& row : grid_) {
            for (auto& gc : row) {
                if (gc.usage <= gc.capacity) continue;
                // Find nets through this cell
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

        // Rip up and reroute congested nets
        for (int ni : reroute_set) {
            rip_up_net(ni);
        }
        for (int ni : reroute_set) {
            if (route_net_astar(ni)) successfully_routed_.insert(ni);
            else successfully_routed_.erase(ni);
        }
    }

    // Clean up deleted wires (width == -1 sentinel)
    pd_.wires.erase(
        std::remove_if(pd_.wires.begin(), pd_.wires.end(),
                       [](const WireSegment& w) { return w.width < 0; }),
        pd_.wires.end());

    // Compute metrics
    result.total_wirelength = 0;
    for (auto& w : pd_.wires)
        result.total_wirelength += w.start.dist(w.end);
    result.max_congestion = 0;
    for (auto& row : grid_)
        for (auto& gc : row)
            result.max_congestion = std::max(result.max_congestion,
                gc.capacity > 0 ? (double)gc.usage / gc.capacity : 0);
    result.overflow = compute_overflow();

    // Recount routed/failed using tracked success status
    result.routed_nets = 0; result.failed_nets = 0;
    for (int ni : net_order) {
        if (successfully_routed_.count(ni) || pd_.nets[ni].cell_ids.size() < 2)
            result.routed_nets++;
        else
            result.failed_nets++;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = std::to_string(result.routed_nets) + "/" +
                     std::to_string(result.routed_nets + result.failed_nets) +
                     " nets routed, overflow: " + std::to_string(result.overflow) +
                     ", " + std::to_string(result.reroute_iterations) + " reroute passes";
    return result;
}

} // namespace sf
