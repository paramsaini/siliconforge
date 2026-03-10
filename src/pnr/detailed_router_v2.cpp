// SiliconForge — Advanced Detailed Router v2
// Dynamic layer assignment based on design complexity.
// Routes nets across multiple metal layers with proper via insertion.
#include "pnr/detailed_router_v2.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <cmath>
#include <algorithm>

namespace sf {

// ── Compute number of routing layers from design complexity ──────────
// Real EDA tools scale metal layers based on:
//   - Number of nets (more connectivity = more layers)
//   - Die area vs cell area (utilization → congestion)
//   - Number of cells
// Typical ranges: tiny design → 2-3 layers, medium → 4-6, large → 8-12, massive → 13-15+

int DetailedRouterV2::compute_num_layers() const {
    int num_nets = static_cast<int>(pd_.nets.size());
    int num_cells = static_cast<int>(pd_.cells.size());
    double die_area = pd_.die_area.area();
    double cell_area = 0;
    for (const auto& c : pd_.cells)
        cell_area += c.width * c.height;
    double utilization = (die_area > 0) ? cell_area / die_area : 0;

    // Base layer count from net count
    int layers = 2; // minimum: 1 horizontal + 1 vertical
    if (num_nets > 10)   layers = 3;
    if (num_nets > 30)   layers = 4;
    if (num_nets > 80)   layers = 5;
    if (num_nets > 150)  layers = 6;
    if (num_nets > 300)  layers = 7;
    if (num_nets > 600)  layers = 8;
    if (num_nets > 1000) layers = 9;
    if (num_nets > 2000) layers = 10;
    if (num_nets > 4000) layers = 11;
    if (num_nets > 8000) layers = 12;
    if (num_nets > 15000) layers = 13;
    if (num_nets > 30000) layers = 14;
    if (num_nets > 60000) layers = 15;

    // Congestion adjustment: high utilization needs more layers
    if (utilization > 0.7 && layers < 15)  layers += 1;
    if (utilization > 0.85 && layers < 15) layers += 1;

    // Cell count adjustment for very dense designs
    if (num_cells > 5000 && layers < 10)  layers = std::max(layers, 8);
    if (num_cells > 20000 && layers < 12) layers = std::max(layers, 10);
    if (num_cells > 50000 && layers < 15) layers = std::max(layers, 12);

    return std::min(layers, 15); // Cap at 15 layers (matches advanced process nodes)
}

bool DetailedRouterV2::check_drc(Point p, int layer, int net_id) const {
    // Simplified DRC: don't route over another net's wire on the same layer
    for (const auto& w : pd_.wires) {
        if (w.layer == layer && w.width > 0) {
            if (p.x >= std::min(w.start.x, w.end.x) && p.x <= std::max(w.start.x, w.end.x) &&
                p.y >= std::min(w.start.y, w.end.y) && p.y <= std::max(w.start.y, w.end.y)) {
                return false;
            }
        }
    }
    return true;
}

// ── L-shape route with proper layer assignment + via ─────────────────
bool DetailedRouterV2::l_shape_route(Point start, Point end,
                                      int h_layer, int v_layer,
                                      int net_id,
                                      std::vector<WireSegment>& local_wires,
                                      std::vector<Via>& local_vias) {
    // L-shape: horizontal segment on h_layer, vertical segment on v_layer
    // Via at the corner connecting h_layer ↔ v_layer
    Point corner(end.x, start.y); // horizontal first, then vertical

    // Horizontal segment (if non-zero length)
    if (std::abs(start.x - corner.x) > 0.001 || std::abs(start.y - corner.y) > 0.001) {
        local_wires.push_back({h_layer, start, corner, 0.1});
    }

    // Vertical segment (if non-zero length)
    if (std::abs(corner.x - end.x) > 0.001 || std::abs(corner.y - end.y) > 0.001) {
        local_wires.push_back({v_layer, corner, end, 0.1});
    }

    // Via at corner if layers differ
    if (h_layer != v_layer) {
        int lower = std::min(h_layer, v_layer);
        int upper = std::max(h_layer, v_layer);
        local_vias.push_back({corner, lower, upper});
    }

    return true;
}

// ── Route a single net across appropriate layer pair ─────────────────
bool DetailedRouterV2::route_net(PhysNet& net, int net_idx,
                                  std::vector<WireSegment>& local_wires,
                                  std::vector<Via>& local_vias) {
    if (net.cell_ids.size() < 2) return true;

    // Layer assignment strategy:
    // - Even layers (0, 2, 4, ...) = preferred horizontal
    // - Odd layers  (1, 3, 5, ...) = preferred vertical
    // - Distribute nets across layer pairs to reduce congestion
    //
    // For net_idx, pick a layer pair:
    //   pair_index = net_idx % (num_layers / 2)
    //   h_layer = pair_index * 2
    //   v_layer = pair_index * 2 + 1
    int num_pairs = std::max(1, num_layers_ / 2);
    int pair_index = net_idx % num_pairs;
    int h_layer = pair_index * 2;           // horizontal layer
    int v_layer = pair_index * 2 + 1;       // vertical layer

    // Clamp to valid range
    if (h_layer >= num_layers_) h_layer = num_layers_ - 2;
    if (v_layer >= num_layers_) v_layer = num_layers_ - 1;
    if (h_layer < 0) h_layer = 0;
    if (v_layer < 0) v_layer = (num_layers_ > 1) ? 1 : 0;

    // Route each two-pin connection in the net
    for (size_t i = 0; i < net.cell_ids.size() - 1; ++i) {
        int c1 = net.cell_ids[i];
        int c2 = net.cell_ids[i + 1];

        Point p1 = pd_.cells[c1].position;
        if (i < net.pin_offsets.size()) p1 = p1 + net.pin_offsets[i];

        Point p2 = pd_.cells[c2].position;
        if (i + 1 < net.pin_offsets.size()) p2 = p2 + net.pin_offsets[i + 1];

        if (!l_shape_route(p1, p2, h_layer, v_layer, net.id, local_wires, local_vias)) {
            return false;
        }
    }
    return true;
}

// ── Multi-threaded routing with dynamic layer count ──────────────────
void DetailedRouterV2::route(int num_threads) {
    // Compute layer count from design if not explicitly set
    if (num_layers_ <= 0) {
        num_layers_ = compute_num_layers();
    }

    std::cout << "DetailedRouterV2: Starting multi-threaded routing with "
              << num_threads << " threads, " << num_layers_ << " metal layers.\n";

    // Populate RoutingLayer metadata in PhysicalDesign
    pd_.layers.clear();
    for (int i = 0; i < num_layers_; ++i) {
        RoutingLayer rl;
        rl.id = i;
        rl.name = "M" + std::to_string(i + 1);
        rl.horizontal = (i % 2 == 0);  // even = H, odd = V
        // Lower layers have tighter pitch; upper layers are wider for power/clock
        double base_pitch = 0.5;
        double base_width = 0.1;
        double base_spacing = 0.1;
        if (i >= 4) { base_pitch *= 2.0; base_width *= 2.0; base_spacing *= 1.5; }
        if (i >= 8) { base_pitch *= 2.0; base_width *= 2.0; base_spacing *= 1.5; }
        if (i >= 12) { base_pitch *= 1.5; base_width *= 1.5; base_spacing *= 1.5; }
        rl.pitch = base_pitch;
        rl.width = base_width;
        rl.spacing = base_spacing;
        pd_.layers.push_back(rl);
    }

    // Split nets among threads
    std::vector<std::future<std::pair<std::vector<WireSegment>, std::vector<Via>>>> futures;

    auto worker = [&](int start_idx, int end_idx) {
        std::vector<WireSegment> local_wires;
        std::vector<Via> local_vias;
        for (int i = start_idx; i < end_idx; ++i) {
            route_net(pd_.nets[i], i, local_wires, local_vias);
        }
        return std::make_pair(local_wires, local_vias);
    };

    int total_nets = static_cast<int>(pd_.nets.size());
    int chunk = total_nets / num_threads;
    if (chunk == 0) chunk = 1;

    for (int i = 0; i < total_nets; i += chunk) {
        int end_idx = std::min(total_nets, i + chunk);
        futures.push_back(std::async(std::launch::async, worker, i, end_idx));
    }

    // Collect results and merge
    for (auto& f : futures) {
        auto [thread_wires, thread_vias] = f.get();
        std::lock_guard<std::mutex> lock(db_mutex_);
        pd_.wires.insert(pd_.wires.end(), thread_wires.begin(), thread_wires.end());
        pd_.vias.insert(pd_.vias.end(), thread_vias.begin(), thread_vias.end());
    }

    // Count wires per layer for reporting
    std::vector<int> wires_per_layer(num_layers_, 0);
    for (const auto& w : pd_.wires) {
        if (w.layer >= 0 && w.layer < num_layers_)
            wires_per_layer[w.layer]++;
    }

    std::cout << "DetailedRouterV2: Routing complete. "
              << pd_.wires.size() << " wires, "
              << pd_.vias.size() << " vias across "
              << num_layers_ << " layers.\n";
    std::cout << "  Layer distribution: ";
    for (int i = 0; i < num_layers_; ++i) {
        std::cout << "M" << (i+1) << "=" << wires_per_layer[i];
        if (i + 1 < num_layers_) std::cout << ", ";
    }
    std::cout << "\n";
}

} // namespace sf
