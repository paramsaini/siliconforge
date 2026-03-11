#pragma once
// SiliconForge — Detailed Router v2 (Track-Based A* + Rip-up/Reroute)
// Production-grade detailed router with:
//   - Track grid construction from layer pitch/spacing
//   - A* maze routing on track grid
//   - Net ordering by half-perimeter wirelength
//   - Rip-up and reroute loop for failed/DRC-violating nets
//   - Multi-layer via optimization
//   - Multi-threaded initial routing
// References:
//   Ozdal & Wong, "ARCHER: A History-Driven Global Routing Algorithm", ICCAD 2007
//   Pan & Chu, "Box-Router: A New Global Router Based on Box Expansion", DAC 2004

#include "pnr/physical.hpp"
#include <vector>
#include <mutex>
#include <set>
#include <unordered_map>

namespace sf {

class DetailedRouterV2 {
public:
    DetailedRouterV2(PhysicalDesign& pd, int num_layers = 0)
        : pd_(pd), num_layers_(num_layers) {}

    void route(int num_threads = 4);
    int num_layers() const { return num_layers_; }

private:
    PhysicalDesign& pd_;
    int num_layers_;
    std::mutex db_mutex_;

    // Track grid
    struct Track {
        int layer;
        double coord;       // x for vertical tracks, y for horizontal tracks
        bool horizontal;
    };
    std::vector<std::vector<Track>> layer_tracks_;  // per-layer track list

    // Occupancy grid: which net occupies which track segment
    struct TrackSeg {
        int layer, track_idx;
        double lo, hi;      // extent along the track
        int net_id;
    };
    std::vector<TrackSeg> occupancy_;

    int compute_num_layers() const;
    void build_track_grid();
    void setup_layers();

    // A* maze routing on track grid
    struct GridNode {
        int layer, track_idx;
        double pos;         // position along track
        double g_cost, f_cost;
        int parent;         // index in closed set
        bool operator>(const GridNode& o) const { return f_cost > o.f_cost; }
    };

    // Route a single 2-pin connection
    bool route_two_pin(Point src, Point dst, int net_id,
                       std::vector<WireSegment>& wires,
                       std::vector<Via>& vias);

    // Route a full net (star topology from centroid)
    bool route_net(PhysNet& net, int net_idx,
                   std::vector<WireSegment>& wires,
                   std::vector<Via>& vias);

    // L-shape fallback (when A* fails or for simple routes)
    bool l_shape_route(Point start, Point end, int h_layer, int v_layer,
                       int net_id,
                       std::vector<WireSegment>& wires,
                       std::vector<Via>& vias);

    // Track nearest to a coordinate on a given layer
    int nearest_track(int layer, double coord) const;

    // Check if a track segment is free
    bool is_free(int layer, int track_idx, double lo, double hi, int net_id) const;

    // Mark/unmark track occupancy
    void mark_occupied(int layer, int track_idx, double lo, double hi, int net_id);
    void unmark_net(int net_id);
};

} // namespace sf
