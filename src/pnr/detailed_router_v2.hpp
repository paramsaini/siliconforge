#pragma once
// SiliconForge — Advanced Detailed Router v2
// Features layer-aware L-shape routing, multi-threading, and basic DRC checking.
// Layer count is determined dynamically from design complexity (nets, cells, area).

#include "pnr/physical.hpp"
#include <vector>
#include <mutex>

namespace sf {

class DetailedRouterV2 {
public:
    // num_layers is computed by the engine from design complexity
    DetailedRouterV2(PhysicalDesign& pd, int num_layers = 0)
        : pd_(pd), num_layers_(num_layers) {}

    // Routes all nets using multi-threading with proper layer assignment
    void route(int num_threads = 4);

    int num_layers() const { return num_layers_; }

private:
    PhysicalDesign& pd_;
    int num_layers_;
    std::mutex db_mutex_;

    // Compute layers from design if not provided
    int compute_num_layers() const;

    bool route_net(PhysNet& net, int net_idx,
                   std::vector<WireSegment>& local_wires,
                   std::vector<Via>& local_vias);

    // L-shape route on specified horizontal + vertical layers, with via between them
    bool l_shape_route(Point start, Point end, int h_layer, int v_layer,
                       int net_id,
                       std::vector<WireSegment>& local_wires,
                       std::vector<Via>& local_vias);

    // Simplistic DRC check to avoid crossing existing wires of different nets
    bool check_drc(Point p, int layer, int net_id) const;
};

} // namespace sf
