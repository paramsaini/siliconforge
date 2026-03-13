#pragma once
// SiliconForge — Floorplanner (Simulated Annealing)
// Macro-level placement using B*-tree representation and SA optimization.
// Reference: Chang et al., "B*-Trees: A New Representation for Non-Slicing Floorplans", DAC 2000

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <random>
#include <utility>

namespace sf {

struct Macro {
    int id;
    std::string name;
    double width, height;
    bool fixed = false;     // fixed macros cannot be moved
    Point position;
    bool rotated = false;
};

// B*-tree representation for non-slicing floorplan
struct BStarNode {
    int macro_idx;
    int left_child = -1;   // left child in B*-tree
    int right_child = -1;  // right child (sibling)
    int parent = -1;
    bool is_rotated = false;
};

struct BStarTree {
    std::vector<BStarNode> nodes;
    int root = -1;

    // Packing: convert tree to placement coordinates
    struct PackResult {
        std::vector<double> x, y;  // coordinates per macro
        double total_width = 0, total_height = 0;
        double area = 0;
        double wirelength = 0;
        double aspect_ratio = 0;
    };
    PackResult pack(const std::vector<Macro>& macros) const;
};

// Simulated annealing configuration
struct SAConfig {
    double initial_temp = 1000.0;
    double cooling_rate = 0.995;
    double min_temp = 0.01;
    int moves_per_temp = 100;      // multiplied by num_macros
    double area_weight = 0.4;
    double wirelength_weight = 0.4;
    double aspect_weight = 0.2;
    double target_aspect_ratio = 1.0;  // width/height target
    double max_aspect_ratio = 3.0;     // reject if > this
};

struct FloorplanResult {
    double total_area = 0;
    double die_width = 0, die_height = 0;
    double dead_space_pct = 0;
    double wirelength = 0;
    int iterations = 0;
    double time_ms = 0;
    std::string message;
    // Enhanced SA result fields
    double aspect_ratio = 0;
    int sa_iterations = 0;
    double sa_acceptance_rate = 0;
    std::vector<bool> rotated;  // rotation per macro
};

class Floorplanner {
public:
    Floorplanner() = default;

    void add_macro(const std::string& name, double w, double h, bool fixed = false);
    void add_connection(int macro_a, int macro_b, double weight = 1.0);
    void set_aspect_ratio(double min_ar, double max_ar) { min_ar_ = min_ar; max_ar_ = max_ar; }

    FloorplanResult solve(int max_iters = 5000, double init_temp = 100.0, double cool_rate = 0.995);

    // Simulated annealing floorplanning with B*-tree
    void set_sa_config(const SAConfig& cfg) { sa_cfg_ = cfg; }
    FloorplanResult floorplan_sa();

    // Pin-aware orientation
    void set_pin_positions(const std::vector<std::vector<std::pair<double,double>>>& pins);

    // Wirelength estimation
    double estimate_wirelength(const BStarTree::PackResult& pack) const;

    // Aspect ratio control
    void set_target_aspect(double ar) { sa_cfg_.target_aspect_ratio = ar; }

    // Get placed macros
    const std::vector<Macro>& macros() const { return macros_; }

    // Export to PhysicalDesign
    PhysicalDesign to_physical_design() const;

private:
    std::vector<Macro> macros_;
    struct Connection { int a, b; double weight; };
    std::vector<Connection> connections_;
    double min_ar_ = 0.5, max_ar_ = 2.0;

    SAConfig sa_cfg_;
    std::vector<std::vector<std::pair<double,double>>> pin_positions_; // per macro

    // SA moves (legacy)
    void random_swap(std::mt19937& rng);
    void random_rotate(std::mt19937& rng);
    void random_move(std::mt19937& rng, double range);

    // Pack macros (simple left-bottom)
    void pack();

    // Cost function
    double compute_cost() const;
    double compute_wirelength() const;
    double bounding_area() const;

    // B*-tree operations for SA
    BStarTree create_initial_tree();
    BStarTree perturb(const BStarTree& tree, std::mt19937& rng);

    // SA move types
    enum class MoveType { SWAP, ROTATE, MOVE_SUBTREE, RESIZE_CHANNEL };
    MoveType random_move_type(std::mt19937& rng);
    BStarTree apply_swap(const BStarTree& tree, int a, int b);
    BStarTree apply_rotate(const BStarTree& tree, int idx);
    BStarTree apply_move_subtree(const BStarTree& tree, int node, int new_parent, std::mt19937& rng);

    // Cost function for B*-tree based SA
    double compute_cost(const BStarTree::PackResult& pack) const;

    // Channel estimation for routing
    double estimate_channel_area(const BStarTree::PackResult& pack) const;
};

} // namespace sf
