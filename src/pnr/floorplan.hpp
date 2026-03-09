#pragma once
// SiliconForge — Floorplanner (Simulated Annealing)
// Macro-level placement using B*-tree representation and SA optimization.
// Reference: Chang et al., "B*-Trees: A New Representation for Non-Slicing Floorplans", DAC 2000

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <random>

namespace sf {

struct Macro {
    int id;
    std::string name;
    double width, height;
    bool fixed = false;     // fixed macros cannot be moved
    Point position;
    bool rotated = false;
};

struct FloorplanResult {
    double total_area = 0;
    double die_width = 0, die_height = 0;
    double dead_space_pct = 0;
    double wirelength = 0;
    int iterations = 0;
    double time_ms = 0;
    std::string message;
};

class Floorplanner {
public:
    Floorplanner() = default;

    void add_macro(const std::string& name, double w, double h, bool fixed = false);
    void add_connection(int macro_a, int macro_b, double weight = 1.0);
    void set_aspect_ratio(double min_ar, double max_ar) { min_ar_ = min_ar; max_ar_ = max_ar; }

    FloorplanResult solve(int max_iters = 5000, double init_temp = 100.0, double cool_rate = 0.995);

    // Get placed macros
    const std::vector<Macro>& macros() const { return macros_; }

    // Export to PhysicalDesign
    PhysicalDesign to_physical_design() const;

private:
    std::vector<Macro> macros_;
    struct Connection { int a, b; double weight; };
    std::vector<Connection> connections_;
    double min_ar_ = 0.5, max_ar_ = 2.0;

    // SA moves
    void random_swap(std::mt19937& rng);
    void random_rotate(std::mt19937& rng);
    void random_move(std::mt19937& rng, double range);

    // Pack macros (simple left-bottom)
    void pack();

    // Cost function
    double compute_cost() const;
    double compute_wirelength() const;
    double bounding_area() const;
};

} // namespace sf
