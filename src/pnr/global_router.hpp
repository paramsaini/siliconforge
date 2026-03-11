#pragma once
// SiliconForge — Global Router (A* + Negotiated Congestion)
// Grid-based routing with A* pathfinding and PathFinder-style rip-up/reroute.
// References:
//   McMurchie & Ebeling, "PathFinder: A Negotiation-Based Performance-Driven
//                         Router for FPGAs", FPGA 1995
//   Kastner et al., "Predictive Congestion-Driven Placement", ICCAD 2000
//   Pan & Chu, "FastRoute 2.0", IEEE TCAD 2007

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <set>
#include <functional>

namespace sf {

struct RouteResult {
    int routed_nets = 0;
    int failed_nets = 0;
    double total_wirelength = 0;
    double max_congestion = 0;
    double time_ms = 0;
    int reroute_iterations = 0;
    int overflow = 0;
    std::string message;
};

struct GCell {
    int x, y, layer;
    int capacity;
    int usage = 0;
    double history_cost = 1.0;    // PathFinder historical congestion
    double present_cost() const {
        if (usage < capacity) return 1.0 + 0.5 * (double)usage / std::max(1, capacity);
        return 1.0 + 10.0 * (usage - capacity + 1); // heavy penalty for overflow
    }
    double total_cost() const { return present_cost() * history_cost; }
};

class GlobalRouter {
public:
    explicit GlobalRouter(PhysicalDesign& pd, int grid_x = 20, int grid_y = 20, int num_layers = 0)
        : pd_(pd), grid_x_(grid_x), grid_y_(grid_y), num_layers_(num_layers) {}

    RouteResult route();

private:
    PhysicalDesign& pd_;
    int grid_x_, grid_y_;
    int num_layers_;
    std::vector<std::vector<GCell>> grid_;

    void build_grid();
    bool route_net_astar(int net_idx);
    void rip_up_net(int net_idx);
    void update_history();
    int compute_overflow() const;
    std::pair<int,int> to_grid(const Point& p);

    // A* priority queue element
    struct AStarNode {
        int x, y;
        double g_cost;  // cost from source
        double f_cost;  // g + heuristic
        bool operator>(const AStarNode& o) const { return f_cost > o.f_cost; }
    };
    std::vector<AStarNode> astar_route(int sx, int sy, int dx, int dy);

    // Rectilinear Steiner minimum tree (RSMT) approximation
    struct SteinerPoint { int x, y; };
    std::vector<std::pair<SteinerPoint,SteinerPoint>> build_rsmt(const std::vector<SteinerPoint>& pins);

    // Per-net wire tracking for rip-up
    struct NetWireRange { int net_idx; size_t wire_start, wire_end; };
    std::vector<NetWireRange> net_wire_ranges_;
    std::set<int> successfully_routed_;  // nets where route_net_astar returned true
};

} // namespace sf
