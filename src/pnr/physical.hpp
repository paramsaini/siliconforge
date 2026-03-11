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

// Complete physical design
class PhysicalDesign {
public:
    Rect die_area;
    double row_height = 10.0;
    double site_width = 1.0;

    std::vector<CellInstance> cells;
    std::vector<PhysNet> nets;
    std::vector<RoutingLayer> layers;
    std::vector<WireSegment> wires;
    std::vector<Via> vias;

    int add_cell(const std::string& name, const std::string& type, double w, double h);
    int add_net(const std::string& name, const std::vector<int>& cell_ids);

    // Metrics
    double total_wirelength() const;    // HPWL
    double utilization() const;          // cell area / die area
    bool has_overlaps() const;

    void print_stats() const;
};

} // namespace sf
