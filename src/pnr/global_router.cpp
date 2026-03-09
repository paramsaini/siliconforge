// SiliconForge — Global Router Implementation
#include "pnr/global_router.hpp"
#include <algorithm>
#include <chrono>
#include <queue>
#include <numeric>
#include <iostream>
#include <cmath>
#include <climits>

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
            grid_[y][x] = {x, y, 0, default_cap, 0};
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

std::vector<GlobalRouter::GridPos> GlobalRouter::lee_route(GridPos src, GridPos dst) {
    // BFS maze routing on grid
    std::vector<std::vector<int>> dist(grid_y_, std::vector<int>(grid_x_, INT_MAX));
    std::vector<std::vector<GridPos>> prev(grid_y_, std::vector<GridPos>(grid_x_, {-1, -1}));
    std::queue<GridPos> q;

    dist[src.y][src.x] = 0;
    q.push(src);

    int dx[] = {0, 0, 1, -1};
    int dy[] = {1, -1, 0, 0};

    while (!q.empty()) {
        auto cur = q.front(); q.pop();
        if (cur.x == dst.x && cur.y == dst.y) break;

        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + dx[d];
            int ny = cur.y + dy[d];
            if (nx < 0 || nx >= grid_x_ || ny < 0 || ny >= grid_y_) continue;

            // Cost-weighted: penalize congested cells
            int new_dist = dist[cur.y][cur.x] + (int)(grid_[ny][nx].cost() * 10);
            if (new_dist < dist[ny][nx]) {
                dist[ny][nx] = new_dist;
                prev[ny][nx] = cur;
                q.push({nx, ny});
            }
        }
    }

    // Backtrace path
    std::vector<GridPos> path;
    if (dist[dst.y][dst.x] == INT_MAX) return path; // no route found

    GridPos cur = dst;
    while (cur.x != src.x || cur.y != src.y) {
        path.push_back(cur);
        cur = prev[cur.y][cur.x];
        if (cur.x < 0) return {}; // no valid path
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return path;
}

bool GlobalRouter::route_net(int net_idx) {
    auto& net = pd_.nets[net_idx];
    if (net.cell_ids.size() < 2) return true;

    // Route as a star: connect all sinks to a Steiner-like center
    // Find bounding box center
    double cx = 0, cy = 0;
    for (size_t i = 0; i < net.cell_ids.size(); ++i) {
        auto& c = pd_.cells[net.cell_ids[i]];
        double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c.width/2);
        double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c.height/2);
        cx += px; cy += py;
    }
    cx /= net.cell_ids.size();
    cy /= net.cell_ids.size();

    auto [gcx, gcy] = to_grid({cx, cy});

    // Route from center to each pin
    double cell_w = pd_.die_area.width() / grid_x_;
    double cell_h = pd_.die_area.height() / grid_y_;

    for (size_t i = 0; i < net.cell_ids.size(); ++i) {
        auto& c = pd_.cells[net.cell_ids[i]];
        double px = c.position.x + (i < net.pin_offsets.size() ? net.pin_offsets[i].x : c.width/2);
        double py = c.position.y + (i < net.pin_offsets.size() ? net.pin_offsets[i].y : c.height/2);
        auto [gx, gy] = to_grid({px, py});

        auto path = lee_route({gcx, gcy}, {gx, gy});
        if (path.empty() && (gcx != gx || gcy != gy)) return false;

        // Update congestion and create wire segments
        for (size_t j = 0; j + 1 < path.size(); ++j) {
            grid_[path[j].y][path[j].x].usage++;

            Point p0(pd_.die_area.x0 + path[j].x * cell_w + cell_w/2,
                     pd_.die_area.y0 + path[j].y * cell_h + cell_h/2);
            Point p1(pd_.die_area.x0 + path[j+1].x * cell_w + cell_w/2,
                     pd_.die_area.y0 + path[j+1].y * cell_h + cell_h/2);

            pd_.wires.push_back({1, p0, p1, 0.1});
        }
    }
    return true;
}

RouteResult GlobalRouter::route() {
    auto t0 = std::chrono::high_resolution_clock::now();
    build_grid();

    RouteResult result;

    // Sort nets by size (smaller nets first — better for congestion)
    std::vector<int> net_order(pd_.nets.size());
    std::iota(net_order.begin(), net_order.end(), 0);
    std::sort(net_order.begin(), net_order.end(), [&](int a, int b) {
        return pd_.nets[a].cell_ids.size() < pd_.nets[b].cell_ids.size();
    });

    for (int ni : net_order) {
        if (route_net(ni)) {
            result.routed_nets++;
        } else {
            result.failed_nets++;
        }
    }

    // Compute metrics
    result.total_wirelength = 0;
    for (auto& w : pd_.wires)
        result.total_wirelength += w.start.dist(w.end);

    result.max_congestion = 0;
    for (auto& row : grid_)
        for (auto& gc : row)
            result.max_congestion = std::max(result.max_congestion,
                gc.capacity > 0 ? (double)gc.usage / gc.capacity : 0);

    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.message = std::to_string(result.routed_nets) + "/" +
                     std::to_string(result.routed_nets + result.failed_nets) +
                     " nets routed, max congestion: " +
                     std::to_string((int)(result.max_congestion * 100)) + "%";
    return result;
}

} // namespace sf
