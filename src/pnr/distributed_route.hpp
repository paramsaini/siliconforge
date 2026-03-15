#pragma once
// SiliconForge — Distributed Routing Engine
// Spatially partitions the design into tiles, routes each tile independently
// (potentially on different threads/processes), then merges results with
// boundary conflict resolution.
//
// Algorithm:
//   1. Divide die into K×K tiles (spatial partitioning)
//   2. Classify each net into:
//      a) Local — all pins in one tile → route independently
//      b) Boundary — pins span 2 tiles → route in special boundary phase
//      c) Global — pins span 3+ tiles → route in global phase
//   3. Phase 1: Route all local nets in parallel (embarrassingly parallel)
//   4. Phase 2: Route boundary nets with neighbor tile lock
//   5. Phase 3: Route global nets sequentially
//   6. Merge congestion maps from all tiles
//
// Reference:
//   Roy & Markov, "High-Performance Routing at the Nanometer Scale", IEEE TCAD 2008
//   Xu et al., "CRISP: Congestion Reduction by Iterated Spreading during Placement", ICCAD 2009
//   Lin et al., "CUGR: Detailed-Routability-Driven 3D Global Routing", DAC 2020

#include "pnr/physical.hpp"
#include "pnr/detailed_router_v2.hpp"
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

namespace sf {

// Classification of a net for distributed routing
enum class NetPartition { LOCAL, BOUNDARY, GLOBAL };

struct PartitionedNet {
    int net_id;
    NetPartition type;
    int primary_tile;     // tile containing most pins (or first pin)
    std::vector<int> tiles;  // all tiles this net touches
};

// Per-tile routing state
struct RouteTile {
    int tile_id;
    Rect area;               // spatial extent of this tile
    std::vector<int> local_nets;     // nets entirely within this tile
    std::vector<int> boundary_nets;  // nets spanning this + one neighbor
    int routed_count = 0;
    int failed_count = 0;
    double time_ms = 0;
};

// Congestion statistics for a tile
struct TileCongestion {
    int tile_id;
    double peak_usage = 0;
    double avg_usage = 0;
    int overflow_gcells = 0;
};

// Result of distributed routing
struct DistributedRouteResult {
    int total_routed = 0;
    int total_failed = 0;
    int local_routed = 0;
    int boundary_routed = 0;
    int global_routed = 0;
    double total_time_ms = 0;
    int tiles_used = 0;
    int threads_used = 0;
    std::vector<TileCongestion> tile_congestion;
    std::string message;
};

class DistributedRouter {
public:
    explicit DistributedRouter(PhysicalDesign& pd, int num_layers = 6)
        : pd_(pd), num_layers_(num_layers) {}

    // Set tile grid size (default: auto based on design size)
    void set_tile_grid(int tx, int ty) { tile_nx_ = tx; tile_ny_ = ty; }

    // Set max threads for parallel local routing
    void set_threads(int t) { max_threads_ = t; }

    // Partition nets into tiles
    void partition_nets();

    // Get partition statistics
    int count_local() const;
    int count_boundary() const;
    int count_global() const;

    // Route all nets using distributed strategy
    DistributedRouteResult route();

    // Accessors
    const std::vector<RouteTile>& tiles() const { return tiles_; }
    const std::vector<PartitionedNet>& partitioned_nets() const { return net_parts_; }

private:
    PhysicalDesign& pd_;
    int num_layers_;
    int tile_nx_ = 0, tile_ny_ = 0;
    int max_threads_ = 4;

    std::vector<RouteTile> tiles_;
    std::vector<PartitionedNet> net_parts_;

    // Phase 1: Route local nets per tile in parallel
    void route_local_parallel(DistributedRouteResult& result);

    // Phase 2: Route boundary nets with neighbor coordination
    void route_boundary(DistributedRouteResult& result);

    // Phase 3: Route global nets sequentially
    void route_global(DistributedRouteResult& result);

    // Determine which tile a point belongs to
    int point_to_tile(double x, double y) const;

    // Build tile grid
    void build_tiles();
};

} // namespace sf
