#pragma once
// SiliconForge — Advanced Detailed Router v2
// Features A* negotiation, multi-threading, and basic DRC checking.

#include "pnr/physical.hpp"
#include <vector>
#include <mutex>

namespace sf {

class DetailedRouterV2 {
public:
    DetailedRouterV2(PhysicalDesign& pd) : pd_(pd) {}

    // Routes a subset of nets using multi-threading
    void route(int num_threads = 4);

private:
    PhysicalDesign& pd_;
    std::mutex db_mutex_;

    bool route_net(PhysNet& net, std::vector<WireSegment>& local_wires, std::vector<Via>& local_vias);
    
    // Detailed A* search between two points taking existing wires into account (avoiding shorts/spacing)
    bool a_star_route(Point start, Point end, int layer, int net_id,
                      std::vector<WireSegment>& local_wires);

    // Simplistic DRC check to avoid crossing existing wires of different nets
    bool check_drc(Point p, int layer, int net_id) const;
};

} // namespace sf
