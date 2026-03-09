#pragma once
// SiliconForge — Detailed Router
// Track assignment and DRC-clean routing on a metal layer grid.
// Reference: Lengauer, "Combinatorial Algorithms for Integrated Circuit Layout", Ch. 9

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct DetailedRouteResult {
    int routed_nets = 0;
    int failed_nets = 0;
    int total_vias = 0;
    double total_wirelength = 0;
    int drc_violations = 0;
    double time_ms = 0;
    std::string message;
};

class DetailedRouter {
public:
    DetailedRouter(PhysicalDesign& pd, int num_layers = 4, double track_pitch = 1.0)
        : pd_(pd), num_layers_(num_layers), track_pitch_(track_pitch) {}

    DetailedRouteResult route();

private:
    PhysicalDesign& pd_;
    int num_layers_;
    double track_pitch_;

    // Track grid: tracks_[layer][track_index] = list of occupied intervals
    struct Interval { double start, end; int net_id; };
    std::vector<std::vector<std::vector<Interval>>> tracks_;

    void build_track_grid();
    bool assign_track(int net_idx, int layer, int track, double start, double end);
    bool route_two_pin(int net_idx, Point p0, Point p1);
    bool is_available(int layer, int track, double start, double end) const;
    int coord_to_track(double coord) const;
};

} // namespace sf
