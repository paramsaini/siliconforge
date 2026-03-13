#pragma once
// SiliconForge — Physical Design Data Structures
// Geometry primitives, die area, placement, and routing representation.

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace sf {

struct Point {
    double x = 0, y = 0;
    Point() = default;
    Point(double x, double y) : x(x), y(y) {}
    Point operator+(const Point& o) const { return {x+o.x, y+o.y}; }
    Point operator-(const Point& o) const { return {x-o.x, y-o.y}; }
    Point operator*(double s) const { return {x*s, y*s}; }
    double dist(const Point& o) const { return std::abs(x-o.x) + std::abs(y-o.y); } // Manhattan
};

struct Rect {
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    Rect() = default;
    Rect(double x0, double y0, double x1, double y1) : x0(x0), y0(y0), x1(x1), y1(y1) {}
    double width() const { return x1 - x0; }
    double height() const { return y1 - y0; }
    double area() const { return width() * height(); }
    Point center() const { return {(x0+x1)/2, (y0+y1)/2}; }
    bool contains(const Point& p) const { return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1; }
    bool overlaps(const Rect& o) const {
        return !(x1 <= o.x0 || o.x1 <= x0 || y1 <= o.y0 || o.y1 <= y0);
    }
};

// Cell instance in the physical layout
struct CellInstance {
    int id = -1;
    std::string name;
    std::string cell_type;
    double width = 1.0, height = 1.0;
    Point position;
    bool placed = false;
    int orientation = 0; // 0=N, 1=S, 2=W, 3=E, 4=FN, etc.
    bool is_macro = false;   // true for hard macros (SRAMs, analog blocks)
    double halo = 5.0;       // keepout zone around macros (um)
};

// Net: connects pins on cell instances
struct PhysNet {
    int id = -1;
    std::string name;
    std::vector<int> cell_ids;   // connected cell IDs
    std::vector<Point> pin_offsets; // pin offset within each cell
};

// Layer for routing
struct RoutingLayer {
    int id;
    std::string name;
    bool horizontal;  // preferred direction
    double pitch;
    double width;
    double spacing;
};

// Routed wire segment
struct WireSegment {
    int layer;
    Point start, end;
    double width;
    int net_id = -1;  // owning net (-1 = unknown)
};

// Via between layers
struct Via {
    Point position;
    int lower_layer, upper_layer;
};

// Routing blockage
struct RoutingBlockage {
    int layer = -1; // -1 = all layers (placement blockage)
    Rect rect;
    std::string type; // "ROUTING", "PLACEMENT", "FILL"
};

// Routing track definition
struct TrackDef {
    std::string direction; // "X" (vertical) or "Y" (horizontal)
    double start = 0;
    int count = 0;
    double step = 0;
    std::vector<std::string> layer_names;
};

// IO Pin definition
struct IoPin {
    std::string name;
    std::string direction; // "INPUT", "OUTPUT", "INOUT"
    std::string net_name;
    int layer = -1;
    Point position;
    bool placed = false;
};

// Via definition (from VIAS section)
struct ViaDef {
    std::string name;
    struct ViaLayer {
        std::string layer_name;
        Rect rect;
    };
    std::vector<ViaLayer> layers;
};

// Special net (power/ground)
struct SpecialNet {
    std::string name;
    std::string use; // "POWER", "GROUND", "CLOCK"
    struct SpecialWire {
        std::string layer_name;
        double width = 0;
        Point start, end;
        std::string shape; // "STRIPE", "RING", "FOLLOWPIN"
    };
    std::vector<SpecialWire> wires;
};

// Region definition
struct Region {
    std::string name;
    std::string type; // "FENCE", "GUIDE"
    std::vector<Rect> rects;
};

// Group definition
struct Group {
    std::string name;
    std::string region_name;
    std::vector<std::string> members; // cell names or patterns
};

// Complete physical design
class PhysicalDesign {
public:
    Rect die_area;
    double row_height = 10.0;
    double site_width = 1.0;
    double dbu_per_micron = 1000.0;

    std::vector<CellInstance> cells;
    std::vector<PhysNet> nets;
    std::vector<RoutingLayer> layers;
    std::vector<WireSegment> wires;
    std::vector<Via> vias;
    std::vector<RoutingBlockage> blockages;
    std::vector<TrackDef> tracks;
    std::vector<IoPin> io_pins;
    std::vector<ViaDef> via_defs;
    std::vector<SpecialNet> special_nets;
    std::vector<Region> regions;
    std::vector<Group> groups;

    // Cell name → index lookup
    std::unordered_map<std::string, int> cell_name_map;

    int add_cell(const std::string& name, const std::string& type, double w, double h);
    int add_net(const std::string& name, const std::vector<int>& cell_ids);

    // Metrics
    double total_wirelength() const;    // HPWL
    double utilization() const;          // cell area / die area
    bool has_overlaps() const;

    void print_stats() const;
};

} // namespace sf
