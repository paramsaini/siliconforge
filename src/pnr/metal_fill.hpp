#pragma once
// SiliconForge — Metal Fill Insertion for CMP Density Compliance
// Inserts fill rectangles to meet minimum/maximum metal density per layer.
// Reference: Kahng et al., "Filling Algorithms and Analyses for Layout Density
//            Control", IEEE TCAD 1999

#include "pnr/physical.hpp"
#include <vector>
#include <string>

namespace sf {

struct MetalFillConfig {
    double min_density  = 0.30;   // minimum metal density per layer
    double max_density  = 0.70;   // maximum metal density per layer
    double fill_width   = 1.0;    // fill rectangle width (um)
    double fill_height  = 1.0;    // fill rectangle height (um)
    double fill_spacing = 0.5;    // minimum spacing from signal wires (um)
    int num_layers      = 5;      // number of layers to process
};

struct MetalFillResult {
    int total_fills = 0;
    std::vector<double> density_before;  // per-layer density before fill
    std::vector<double> density_after;   // per-layer density after fill
    std::string message;
};

class MetalFillEngine {
public:
    explicit MetalFillEngine(PhysicalDesign& pd) : pd_(pd) {}
    MetalFillResult fill(const MetalFillConfig& cfg = {});

private:
    PhysicalDesign& pd_;
    double compute_layer_density(int layer) const;
    int fill_layer(int layer, const MetalFillConfig& cfg);
};

} // namespace sf
