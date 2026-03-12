#pragma once
// SiliconForge — Global Router (A* + Negotiated Congestion)
// Grid-based routing with A* pathfinding and PathFinder-style rip-up/reroute.
// Industrial: timing-driven net ordering, slack-weighted A* cost, crosstalk
//             penalty, net criticality, via cost, antenna area tracking.
// References:
//   McMurchie & Ebeling, "PathFinder: A Negotiation-Based Performance-Driven
//                         Router for FPGAs", FPGA 1995
//   Kastner et al., "Predictive Congestion-Driven Placement", ICCAD 2000
//   Pan & Chu, "FastRoute 2.0", IEEE TCAD 2007
//   Hu & Sapatnekar, "A Timing-Driven Detailed Router", DAC 2002

#include "pnr/physical.hpp"
#include "verify/antenna.hpp"
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <functional>

namespace sf {

// Industrial routing configuration
struct RouterConfig {
    // Timing-driven
    bool enable_timing_driven = false;
    double timing_weight = 0.5;       // 0=pure congestion, 1=pure timing
    double criticality_exponent = 2.0; // higher = more aggressive on critical nets

    // Crosstalk
    bool enable_crosstalk_avoidance = false;
    double xtalk_spacing_mult = 2.0;  // extra spacing multiplier for critical nets

    // Via optimization
    bool enable_via_minimization = false;
    double via_cost = 1.5;            // cost penalty per via in A*

    // Antenna
    bool enable_antenna_check = false;
    double max_antenna_ratio = 400.0; // max wire_area / gate_area ratio

    // Rip-up control
    int max_reroute_iterations = 15;
    int detailed_reroute_iterations = 8;

    // Layer assignment
    bool enable_layer_promotion = false; // promote critical nets to upper layers
};

// Per-net timing info (populated from STA)
struct NetTimingInfo {
    double slack = 1e18;       // worst slack among sinks
    double criticality = 0.0;  // 0=non-critical, 1=most critical
    bool is_clock = false;
    bool is_critical = false;  // slack < 0
};

struct RouteResult {
    int routed_nets = 0;
    int failed_nets = 0;
    double total_wirelength = 0;
    double max_congestion = 0;
    double time_ms = 0;
    int reroute_iterations = 0;
    int overflow = 0;
    std::string message;

    // Industrial metrics
    int total_vias = 0;
    int total_wires = 0;
    double critical_net_wirelength = 0; // wirelength on timing-critical nets
    int timing_driven_reroutes = 0;     // nets rerouted for timing
    int antenna_violations = 0;
    std::vector<AntennaViolation> antenna_details;
    int nets_on_promoted_layers = 0;    // nets moved to upper layers

    // Per-layer wire count
    std::vector<int> wires_per_layer;
    std::vector<int> vias_per_layer;
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

    // Industrial: configuration
    void set_config(const RouterConfig& cfg) { config_ = cfg; }
    RouterConfig& config() { return config_; }

    // Industrial: set per-net timing info (from STA)
    void set_net_timing(int net_id, const NetTimingInfo& info) {
        net_timing_[net_id] = info;
    }
    void set_net_timing_map(const std::unordered_map<int, NetTimingInfo>& map) {
        net_timing_ = map;
    }

private:
    PhysicalDesign& pd_;
    int grid_x_, grid_y_;
    int num_layers_;
    std::vector<std::vector<GCell>> grid_;
    RouterConfig config_;
    std::unordered_map<int, NetTimingInfo> net_timing_;

    void build_grid();
    bool route_net_astar(int net_idx);
    void rip_up_net(int net_idx);
    void update_history();
    int compute_overflow() const;
    std::pair<int,int> to_grid(const Point& p);

    // Industrial: get net criticality (0.0 to 1.0)
    double get_criticality(int net_idx) const;

    // Industrial: timing-weighted edge cost for A*
    double edge_cost(int nx, int ny, int net_idx) const;

    // Industrial: compute antenna ratio for a net
    AntennaViolation check_antenna(int net_idx) const;

    // A* priority queue element
    struct AStarNode {
        int x, y;
        double g_cost;  // cost from source
        double f_cost;  // g + heuristic
        bool operator>(const AStarNode& o) const { return f_cost > o.f_cost; }
    };
    std::vector<AStarNode> astar_route(int sx, int sy, int dx, int dy, int net_idx);

    // Rectilinear Steiner minimum tree (RSMT) approximation
    struct SteinerPoint { int x, y; };
    std::vector<std::pair<SteinerPoint,SteinerPoint>> build_rsmt(const std::vector<SteinerPoint>& pins);

    // Per-net wire tracking for rip-up
    struct NetWireRange { int net_idx; size_t wire_start, wire_end; };
    std::vector<NetWireRange> net_wire_ranges_;
    std::set<int> successfully_routed_;
};

} // namespace sf
