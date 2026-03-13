#pragma once
// SiliconForge — Metal Fill Insertion for CMP Density Compliance
// SI/timing-aware fill with pattern support and critical net avoidance.
// Reference: Kahng et al., "Filling Algorithms and Analyses for Layout Density
//            Control", IEEE TCAD 1999
// Reference: Calibre Pattern-Based Fill, Siemens EDA

#include "pnr/physical.hpp"
#include <vector>
#include <string>
#include <unordered_set>

namespace sf {

enum class FillPattern {
    SOLID,          // simple rectangular fill (original behavior)
    CHECKERBOARD,   // alternating fill/space grid
    STAGGERED,      // offset rows for better CMP uniformity
    SLOTTED         // slotted fill (long narrow openings)
};

struct MetalFillConfig {
    double min_density  = 0.30;   // minimum metal density per layer
    double max_density  = 0.70;   // maximum metal density per layer
    double fill_width   = 1.0;    // fill rectangle width (um)
    double fill_height  = 1.0;    // fill rectangle height (um)
    double fill_spacing = 0.5;    // minimum spacing from signal wires (um)
    int num_layers      = 5;      // number of layers to process
    FillPattern pattern = FillPattern::SOLID;

    // SI/timing-aware options
    double critical_net_spacing = 2.0;  // enlarged spacing near critical nets (um)
    bool si_aware = false;              // enable SI-aware fill avoidance
    double max_coupling_increase = 0.1; // max allowed coupling cap increase (fF)

    // Cross-layer awareness
    bool cross_layer_aware = false;     // avoid fills that create cross-layer coupling
    double cross_layer_spacing = 1.0;   // min spacing to wires on adjacent layers
};

struct MetalFillResult {
    int total_fills = 0;
    int fills_skipped_critical = 0;     // fills skipped due to critical net proximity
    int fills_skipped_cross_layer = 0;  // fills skipped due to cross-layer coupling
    std::vector<double> density_before;  // per-layer density before fill
    std::vector<double> density_after;   // per-layer density after fill
    double estimated_coupling_increase_ff = 0; // total coupling cap increase
    std::string message;
};

class MetalFillEngine {
public:
    explicit MetalFillEngine(PhysicalDesign& pd) : pd_(pd) {}
    MetalFillResult fill(const MetalFillConfig& cfg = {});

    // Set nets to treat as timing-critical (enlarged keepout)
    void set_critical_nets(const std::unordered_set<int>& net_ids) {
        critical_nets_ = net_ids;
    }

private:
    PhysicalDesign& pd_;
    std::unordered_set<int> critical_nets_;

    double compute_layer_density(int layer) const;
    int fill_layer(int layer, const MetalFillConfig& cfg, MetalFillResult& res);
    bool near_critical_wire(double x, double y, double w, double h,
                            int layer, double spacing) const;
    bool near_cross_layer_wire(double x, double y, double w, double h,
                               int layer, double spacing) const;
};

} // namespace sf
