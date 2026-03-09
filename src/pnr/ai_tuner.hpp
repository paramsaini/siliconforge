#pragma once
// SiliconForge — AI-Driven PnR Tuner
// Auto-tunes placement density and spacing weights to jointly
// optimize timing WNS and congestion using simulated annealing /
// gradient-free optimization.

#include "pnr/physical.hpp"
#include "core/netlist.hpp"
#include <string>

namespace sf {

struct TunerResult {
    double best_wns = -1e18;
    double best_congestion = 1e18;
    int iterations = 0;
    double time_ms = 0;
    
    // Tuned hyperparams
    double optimal_density_weight = 0;
    double optimal_wirelength_weight = 0;
};

class AiTuner {
public:
    AiTuner(PhysicalDesign& pd, const Netlist& nl)
        : pd_(pd), nl_(nl) {}

    // Run AI hyperparameter optimization loop
    TunerResult optimize(int max_iterations = 10, double clock_period = 1.0);

private:
    PhysicalDesign& pd_;
    const Netlist& nl_;
};

} // namespace sf
