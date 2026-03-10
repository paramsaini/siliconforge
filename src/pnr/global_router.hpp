#pragma once
// SiliconForge — Global Router
// Grid-based maze routing with congestion awareness.
// Reference: Lee, "An Algorithm for Path Connections", IRE Trans, 1961
//            Kastner et al., "Pattern Routing", ICCAD 2000

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct RouteResult {
    int routed_nets = 0;
    int failed_nets = 0;
    double total_wirelength = 0;
    double max_congestion = 0;
    double time_ms = 0;
    std::string message;
};

struct GCell {
    int x, y, layer;
    int capacity;      // max wires through this cell
    int usage = 0;     // current wires through
    double cost() const { return usage >= capacity ? 1000.0 : 1.0 + (double)usage / capacity; }
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
    std::vector<std::vector<GCell>> grid_; // grid_[y][x]

    void build_grid();
    bool route_net(int net_idx);
    std::pair<int,int> to_grid(const Point& p);

    // Lee maze router (BFS-based)
    struct GridPos { int x, y; };
    std::vector<GridPos> lee_route(GridPos src, GridPos dst);
};

} // namespace sf
