#pragma once
// SiliconForge — Analytical Placer
// Quadratic wirelength minimization with spreading.
// Reference: Naylor et al., "Non-Linear Optimization System and Methods for Wire
//            Length and Delay Optimization for an Automatic Electric Circuit Placer", 2001

#include "pnr/physical.hpp"
#include <string>

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

    // Tune hyperparams for AI optimization
    void set_hyperparams(double wl_weight, double density_weight) {
        wl_weight_ = wl_weight;
        density_weight_ = density_weight;
    }

    // Run placement
    PlaceResult place();

private:
    PhysicalDesign& pd_;
    double wl_weight_ = 1.0;
    double density_weight_ = 1.0;

    // Step 1: Quadratic placement (minimize ∑w_ij(x_i - x_j)²)
    void solve_quadratic();

    // Step 2: Spreading — remove overlaps via recursive bisection
    void spread();

    // Step 3: Legalization — snap to rows
    void legalize();

    // Helper: compute HPWL
    double compute_hpwl() const;
};

} // namespace sf
