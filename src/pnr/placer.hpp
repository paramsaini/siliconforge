#pragma once
// SiliconForge — Analytical Placer (SimPL-style) with Timing-Driven Mode
// Quadratic wirelength minimization with look-ahead legalization density spreading.
// Timing-driven: optional STA integration for slack-aware net weighting.
// References:
//   Kim et al., "SimPL: An Effective Placement Algorithm", IEEE TCAD 2012
//   Spindler et al., "Abacus: Fast Legalization of Standard Cell Circuits with
//                    Minimal Movement", ISPD 2008
//   Viswanathan & Chu, "FastPlace: Efficient Analytical Placement using Cell
//                       Shifting, Iterative Local Refinement, and a Hybrid
//                       Net Model", IEEE TCAD 2005
//   Ren et al., "Timing-Driven Placement for FPGAs", ACM/SIGDA 2006
//   Kong, "Timing-Driven Force-Directed Placement", IEEE TCAD 2002

#include "pnr/physical.hpp"
#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>

namespace sf {

// Placement constraint types for industrial fence/blockage support
// Reference: Synopsys IC Compiler create_bounds, Cadence Innovus create_fence
enum class ConstraintType { FENCE, GUIDE, BLOCKAGE };

struct PlacementConstraint {
    ConstraintType type = ConstraintType::FENCE;
    std::string name;
    Rect area;                     // bounding box
    std::vector<int> cell_ids;     // cells confined to this region (fence/guide)
    // FENCE: hard — cells MUST be inside area
    // GUIDE: soft — cells PREFER to be inside area
    // BLOCKAGE: no cells allowed inside area
};

// Cell padding for DRC-aware legalization
struct CellPadding {
    double left = 0;   // extra spacing on left side (um)
    double right = 0;  // extra spacing on right side (um)
};

// Congestion estimation bin (RUDY model)
// Reference: Spindler et al., "Fast and Accurate Routing Demand Estimation", DAC 2007
struct CongestionBin {
    double demand = 0;   // estimated routing demand (wire density)
    double supply = 0;   // available routing tracks
    double overflow() const { return std::max(0.0, demand - supply); }
    double ratio() const { return supply > 0 ? demand / supply : 0; }
};

struct PlaceResult {
    double hpwl = 0;
    double time_ms = 0;
    int iterations = 0;
    bool legal = false;
    std::string message;

    // Timing-driven results (populated when timing mode is enabled)
    double wns = 0;           // worst negative slack after placement
    double tns = 0;           // total negative slack after placement
    int critical_nets = 0;    // number of timing-critical nets (slack < 0)
    int timing_iterations = 0; // number of STA updates during placement

    // Industrial placement metrics
    int timing_swaps = 0;              // cells swapped for timing in detailed placement
    double displacement_avg = 0;       // average cell displacement from global to legal (um)
    double displacement_max = 0;       // worst-case displacement (um)
    double congestion_peak = 0;        // peak congestion ratio (demand/supply)
    double congestion_avg = 0;         // average congestion across bins
    int constraint_violations = 0;     // fence/blockage violations remaining
    int cells_padded = 0;              // cells with non-zero padding applied

    // Slack distribution histogram (10 buckets)
    // [0]: slack < -1.0ns, [1]: -1.0 to -0.5, [2]: -0.5 to -0.2, [3]: -0.2 to -0.1,
    // [4]: -0.1 to 0, [5]: 0 to 0.1, [6]: 0.1 to 0.2, [7]: 0.2 to 0.5,
    // [8]: 0.5 to 1.0, [9]: >= 1.0ns
    std::array<int, 10> slack_histogram = {};
};

class AnalyticalPlacer {
public:
    explicit AnalyticalPlacer(PhysicalDesign& pd) : pd_(pd) {}

    void set_hyperparams(double wl_weight, double density_weight) {
        wl_weight_ = wl_weight;
        density_weight_ = density_weight;
    }

    void set_net_weight(int net_id, double weight) { net_weights_[net_id] = weight; }
    void set_target_density(double d) { target_density_ = d; }
    void set_macro_fixed(int cell_id) { fixed_cells_.insert(cell_id); }

    // --- Timing-driven placement configuration ---
    // Enable timing-driven mode: requires netlist for STA
    void enable_timing_driven(const Netlist& nl,
                              double clock_period,
                              const LibertyLibrary* lib = nullptr) {
        timing_driven_ = true;
        nl_ = &nl;
        clock_period_ = clock_period;
        lib_ = lib;
    }

    // Timing weight parameters
    void set_timing_weight(double w) { timing_weight_ = w; }
    void set_criticality_exponent(double e) { criticality_exp_ = e; }
    void set_timing_update_interval(int interval) { timing_update_interval_ = interval; }

    // --- Industrial placement constraints ---
    void add_constraint(const PlacementConstraint& c) { constraints_.push_back(c); }
    void add_fence(const std::string& name, const Rect& area, const std::vector<int>& cells) {
        constraints_.push_back({ConstraintType::FENCE, name, area, cells});
    }
    void add_blockage(const std::string& name, const Rect& area) {
        constraints_.push_back({ConstraintType::BLOCKAGE, name, area, {}});
    }

    // Cell padding for DRC-aware legalization
    void set_cell_padding(int cell_id, double left, double right) {
        cell_padding_[cell_id] = {left, right};
    }

    // Congestion-driven placement
    void enable_congestion_driven(bool en = true) { congestion_driven_ = en; }
    void set_congestion_weight(double w) { congestion_weight_ = w; }

    // Set blockages from floorplanner constraints
    void set_blockages(const std::vector<Rect>& blockages) {
        for (auto& b : blockages)
            constraints_.push_back({ConstraintType::BLOCKAGE, "blockage", b, {}});
    }

    // Post-placement congestion-driven refinement using ML predictor
    void congestion_driven_refine(const Netlist& nl);

    // Incremental placement (ECO mode): only move specified cells
    void enable_incremental_mode(const std::vector<int>& cells_to_move) {
        incremental_mode_ = true;
        incremental_cells_.clear();
        for (int c : cells_to_move) incremental_cells_.insert(c);
    }
    void set_max_displacement(double d) { max_displacement_ = d; }

    PlaceResult place();

private:
    PhysicalDesign& pd_;
    double wl_weight_ = 1.0;
    double density_weight_ = 1.0;
    double target_density_ = 0.85;
    std::unordered_map<int, double> net_weights_;
    std::unordered_set<int> fixed_cells_;

    // Timing-driven state
    bool timing_driven_ = false;
    const Netlist* nl_ = nullptr;
    const LibertyLibrary* lib_ = nullptr;
    double clock_period_ = 0;
    double timing_weight_ = 5.0;       // multiplier for critical net weights
    double criticality_exp_ = 3.0;     // exponent for criticality function
    int timing_update_interval_ = 3;   // run STA every N outer iterations
    std::unordered_map<int, double> timing_net_weights_; // STA-derived weights

    // Industrial state
    std::vector<PlacementConstraint> constraints_;
    std::unordered_map<int, CellPadding> cell_padding_;
    bool congestion_driven_ = false;
    double congestion_weight_ = 2.0;
    bool incremental_mode_ = false;
    std::unordered_set<int> incremental_cells_;
    double max_displacement_ = 50.0; // max allowed displacement in ECO mode (um)

    // Congestion estimation grid
    int cong_nx_ = 0, cong_ny_ = 0;
    std::vector<std::vector<CongestionBin>> congestion_grid_;

    // Timing-driven: run STA and update net weights by criticality
    void update_timing_weights(PlaceResult& result);

    // Industrial passes
    void timing_aware_detail_placement(PlaceResult& result);
    void enforce_constraints(PlaceResult& result);
    void apply_cell_padding();
    void estimate_congestion(PlaceResult& result);
    void compute_slack_histogram(PlaceResult& result);
    void compute_displacement(PlaceResult& result,
                              const std::vector<double>& orig_x,
                              const std::vector<double>& orig_y);

    // Congestion: RUDY (Rectangular Uniform wire DensitY) estimation
    void build_congestion_grid(int nx, int ny);
    void update_congestion_demand();
    void apply_congestion_penalty();

    // Conjugate gradient state
    std::vector<double> x_, y_;             // current positions
    std::vector<double> anchor_x_, anchor_y_; // spreading anchors
    std::vector<double> anchor_w_;          // anchor weights (increase each iter)

    // Step 1: Conjugate gradient quadratic solver
    void solve_quadratic_cg();

    // Step 2: Density-aware bin spreading (look-ahead legalization)
    void density_spread();

    // Step 3: Abacus legalization — optimal row assignment minimizing displacement
    void legalize_abacus();

    // Helper: compute HPWL
    double compute_hpwl() const;

    // Helper: build B2B (bound-to-bound) net model weights
    struct B2BEdge { int i, j; double wx, wy; };
    std::vector<B2BEdge> build_b2b_model() const;

    // Density bin grid
    struct DensityBin {
        double x0, y0, x1, y1;
        double supply;    // available area for cells
        double demand;    // total cell area placed here
        double target_x, target_y; // center after spreading
    };
    std::vector<std::vector<DensityBin>> density_grid_;
    int bin_nx_ = 0, bin_ny_ = 0;
    void build_density_grid(int nx, int ny);
    void compute_density();
    void spread_bins_x();
    void spread_bins_y();
};

} // namespace sf
