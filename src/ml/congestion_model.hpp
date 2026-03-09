#pragma once
// SiliconForge — ML-driven Congestion Predictor
// Predicts routing congestion during placement logic using a probabilistic
// footprint model combined with feature-based density metrics.
// Fits the "AI-Native" architecture goal for early prediction.

#include "pnr/physical.hpp"
#include <vector>

namespace sf {

struct CongestionResult {
    double peak_congestion = 0;
    double average_congestion = 0;
    int overflow_bins = 0;
    std::vector<std::vector<double>> heatmap_grid; // 2D grid [y][x]
    double time_ms = 0;
};

class MlCongestionPredictor {
public:
    MlCongestionPredictor(const PhysicalDesign& pd, int grid_x = 50, int grid_y = 50)
        : pd_(pd), grid_x_(grid_x), grid_y_(grid_y) {}

    // Predict congestion before routing based on HPWL and net topology
    CongestionResult predict() const;

private:
    const PhysicalDesign& pd_;
    int grid_x_;
    int grid_y_;

    // Evaluates a probabilistic bounding box model (RUDY-like) for a net
    void add_net_prob(int net_id, std::vector<std::vector<double>>& grid,
                     double cell_w, double cell_h) const;
};

} // namespace sf
