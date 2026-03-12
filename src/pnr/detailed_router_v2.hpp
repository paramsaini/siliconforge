#pragma once
// SiliconForge — Detailed Router v2 (Track-Based A* + Rip-up/Reroute)
// Production-grade detailed router with:
//   - Track grid construction from layer pitch/spacing
//   - A* maze routing on track grid
//   - Net ordering by half-perimeter wirelength
//   - Rip-up and reroute loop for failed/DRC-violating nets
//   - Multi-layer via optimization
// Industrial:
//   - Timing-driven net ordering (critical nets first)
//   - Via minimization (track vias, reduce layer changes)
//   - Via cost in routing decisions
//   - Crosstalk-aware spacing for critical nets
//   - Antenna area tracking per net
//   - RouteResult with industrial metrics
// References:
//   Ozdal & Wong, "ARCHER: A History-Driven Global Routing Algorithm", ICCAD 2007
//   Pan & Chu, "Box-Router: A New Global Router Based on Box Expansion", DAC 2004
//   Hu & Sapatnekar, "A Timing-Driven Detailed Router", DAC 2002

#include "pnr/physical.hpp"
#include "pnr/global_router.hpp"  // RouterConfig, NetTimingInfo, RouteResult
#include <vector>
#include <mutex>
#include <set>
#include <unordered_map>

namespace sf {

class DetailedRouterV2 {
public:
    DetailedRouterV2(PhysicalDesign& pd, int num_layers = 0)
        : pd_(pd), num_layers_(num_layers) {}

    RouteResult route(int num_threads = 4);
    int num_layers() const { return num_layers_; }

    // Industrial: configuration + timing
    void set_config(const RouterConfig& cfg) { config_ = cfg; }
    RouterConfig& config() { return config_; }
    void set_net_timing(int net_id, const NetTimingInfo& info) {
        net_timing_[net_id] = info;
    }
    void set_net_timing_map(const std::unordered_map<int, NetTimingInfo>& map) {
        net_timing_ = map;
    }

private:
    PhysicalDesign& pd_;
    int num_layers_;
    std::mutex db_mutex_;
    RouterConfig config_;
    std::unordered_map<int, NetTimingInfo> net_timing_;

    // Track grid
    struct Track {
        int layer;
        double coord;
        bool horizontal;
    };
    std::vector<std::vector<Track>> layer_tracks_;

    // Occupancy grid
    struct TrackSeg {
        int layer, track_idx;
        double lo, hi;
        int net_id;
    };
    std::vector<TrackSeg> occupancy_;

    int compute_num_layers() const;
    void build_track_grid();
    void setup_layers();

    // A* maze routing on track grid
    struct GridNode {
        int layer, track_idx;
        double pos;
        double g_cost, f_cost;
        int parent;
        bool operator>(const GridNode& o) const { return f_cost > o.f_cost; }
    };

    bool route_two_pin(Point src, Point dst, int net_id,
                       std::vector<WireSegment>& wires,
                       std::vector<Via>& vias);
    bool route_net(PhysNet& net, int net_idx,
                   std::vector<WireSegment>& wires,
                   std::vector<Via>& vias);
    bool l_shape_route(Point start, Point end, int h_layer, int v_layer,
                       int net_id,
                       std::vector<WireSegment>& wires,
                       std::vector<Via>& vias);

    int nearest_track(int layer, double coord) const;
    bool is_free(int layer, int track_idx, double lo, double hi, int net_id) const;
    void mark_occupied(int layer, int track_idx, double lo, double hi, int net_id);
    void unmark_net(int net_id);

    // Industrial: net criticality
    double get_criticality(int net_id) const;

    // Industrial: via minimization — count vias for a net
    int count_net_vias(const std::vector<Via>& vias) const;

    // Industrial: antenna check per net
    AntennaViolation check_antenna(int net_id,
                                    const std::vector<WireSegment>& wires) const;
};

} // namespace sf
