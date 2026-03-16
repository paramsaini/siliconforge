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

// Interval tree for O(log n) track occupancy queries.
// Organized as: layer -> track_idx -> sorted intervals.
// Uses a sorted vector of intervals per (layer, track) pair with binary search.
class TrackOccupancy {
public:
    struct Interval {
        double lo, hi;
        int net_id;
    };

    void clear();

    // Insert a segment — O(log n) amortized
    void insert(int layer, int track_idx, double lo, double hi, int net_id);

    // Query if range [lo, hi] is free on (layer, track_idx), ignoring net_id — O(log n)
    bool is_free(int layer, int track_idx, double lo, double hi, int net_id,
                 double buffer, double adj_buffer, double pitch, double wire_width, double spacing) const;

    // Check same track + adjacent tracks (matching current DRC logic)
    bool is_free_with_adj(int layer, int track_idx, double lo, double hi, int net_id,
                          double wire_width, double spacing, double pitch) const;

    // Remove all segments for a net — O(n_net) where n_net = segments belonging to that net
    void remove_net(int net_id);

    // Total segment count
    size_t size() const;

private:
    // Key: (layer << 20) | track_idx -> sorted vector of Interval by lo
    // Phase 98: Expanded from 16-bit to 20-bit track index (1M tracks per layer)
    std::unordered_map<int64_t, std::vector<Interval>> tracks_;

    int64_t make_key(int layer, int track_idx) const {
        return (static_cast<int64_t>(layer) << 20) | (static_cast<int64_t>(track_idx) & 0xFFFFF);
    }

    // Binary search: find first interval whose hi+buffer > lo, then scan forward
    bool has_overlap(const std::vector<Interval>& intervals, double lo, double hi,
                     double buffer, int exclude_net) const;
};

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

    // Per-layer design rules
    struct LayerRules {
        int layer;
        double min_width;
        double min_spacing;
        double min_enclosure;    // via enclosure
        bool prefer_horizontal;  // preferred routing direction
        double pitch;
    };
    void set_layer_rules(const std::vector<LayerRules>& rules) { layer_rules_ = rules; }

    // Multi-layer track assignment
    struct TrackAssignResult {
        int net_idx;
        int layer;
        int track_idx;
        double start_x, end_x;  // or start_y, end_y for vertical
        int via_count;
    };
    std::vector<TrackAssignResult> assign_tracks_multilayer(int num_layers = 4);

    // Via pillar optimization
    struct ViaPillar {
        double x, y;
        int bottom_layer, top_layer;
        bool is_stacked;   // true if multi-layer via stack
    };
    int optimize_via_pillars(std::vector<ViaPillar>& vias);

    // DRC-aware routing
    struct DrcConstraint {
        double min_spacing = 0.14;
        double min_width = 0.14;
        double via_enclosure = 0.05;
        double end_of_line_spacing = 0.2;    // special spacing at wire ends
        double corner_spacing = 0.2;
    };
    void set_drc_constraints(const DrcConstraint& drc) { drc_constraints_ = drc; }

    // Convergence-based global rip-up reroute
    struct ConvergenceConfig {
        int max_iterations = 100;
        double overflow_target = 0.0;
        double cost_escalation = 1.5;   // multiply congestion cost each iteration
        int nets_per_iteration = 10;    // how many to rip-up per iteration
    };
    void set_convergence_config(const ConvergenceConfig& cfg) { conv_cfg_ = cfg; }

    // Enhanced routing with convergence
    RouteResult route_with_convergence();

    // DRC-clean routing (check DRC during maze routing)
    bool route_net_drc_aware(int net_idx);

    // Double patterning awareness
    struct DPConfig {
        bool enable = false;
        double min_dp_spacing = 0.2;    // min spacing for same-mask wires
        int num_masks = 2;
    };
    void set_dp_config(const DPConfig& cfg) { dp_cfg_ = cfg; }
    bool check_dp_conflict(int track1, int track2, int layer) const;

    // Pattern routing: try L/Z shapes before falling back to A*
    // Returns true if pattern route succeeded
    bool try_pattern_route(Point src, Point dst, int net_id,
                           std::vector<WireSegment>& wires,
                           std::vector<Via>& vias);

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
    TrackOccupancy occ_;  // O(log n) interval-based occupancy index

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

    // New private members for enhanced routing
    std::vector<LayerRules> layer_rules_;
    DrcConstraint drc_constraints_;
    ConvergenceConfig conv_cfg_;
    DPConfig dp_cfg_;

    // Track assignment helpers
    struct TrackResource {
        int layer;
        int track_idx;
        std::vector<std::pair<double,double>> occupied;  // occupied intervals
        bool is_available(double start, double end, double spacing) const;
    };
    std::vector<std::vector<TrackResource>> track_resources_;  // per layer

    void init_track_resources(int num_layers);
    int find_best_track(int net_idx, int layer, double start, double end);

    // DRC-aware maze helpers
    struct MazeNode {
        int x, y, layer;
        double cost;
        int parent;
        bool operator>(const MazeNode& o) const { return cost > o.cost; }
    };
    bool drc_check_segment(double x1, double y1, double x2, double y2, int layer, int net_idx) const;
    double maze_cost(const MazeNode& from, const MazeNode& to) const;

    // Convergence helpers
    std::vector<int> identify_violating_nets();
    void escalate_costs(double factor);
    double congestion_penalty_ = 1.0;

    // Per-net wire/via storage used by convergence routing
    std::vector<std::vector<WireSegment>> conv_net_wires_;
    std::vector<std::vector<Via>> conv_net_vias_;
};

} // namespace sf
