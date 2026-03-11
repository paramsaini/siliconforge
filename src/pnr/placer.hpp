#pragma once
// SiliconForge — Analytical Placer (SimPL-style)
// Quadratic wirelength minimization with look-ahead legalization density spreading.
// References:
//   Kim et al., "SimPL: An Effective Placement Algorithm", IEEE TCAD 2012
//   Spindler et al., "Abacus: Fast Legalization of Standard Cell Circuits with
//                    Minimal Movement", ISPD 2008
//   Viswanathan & Chu, "FastPlace: Efficient Analytical Placement using Cell
//                       Shifting, Iterative Local Refinement, and a Hybrid
//                       Net Model", IEEE TCAD 2005

#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace sf {

struct PlaceResult {
    double hpwl = 0;
    double time_ms = 0;
    int iterations = 0;
    bool legal = false;
    std::string message;
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

    PlaceResult place();

private:
    PhysicalDesign& pd_;
    double wl_weight_ = 1.0;
    double density_weight_ = 1.0;
    double target_density_ = 0.85;
    std::unordered_map<int, double> net_weights_;
    std::unordered_set<int> fixed_cells_;

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
