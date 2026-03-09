#pragma once
// SiliconForge — Antenna Rule Checker
// Detects process antenna violations in routed designs.
// During fabrication, long wires can accumulate charge and damage gate oxides.
// Reference: Chen et al., "Antenna Avoidance in IC Design", IEEE TCAD 1999

#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct AntennaViolation {
    int net_id;
    std::string net_name;
    int layer;
    double wire_area;
    double gate_area;
    double ratio;
    double max_ratio;
    std::string fix_suggestion;
};

struct AntennaResult {
    int nets_checked = 0;
    int violations = 0;
    double worst_ratio = 0;
    double time_ms = 0;
    std::vector<AntennaViolation> details;
    std::string message;
};

struct AntennaRules {
    double max_ratio = 400;       // max wire_area / gate_area per layer
    double max_cum_ratio = 1000;  // cumulative across layers
    double diode_threshold = 200; // ratio above which diode fix is needed
};

class AntennaChecker {
public:
    AntennaChecker(const PhysicalDesign& pd, AntennaRules rules = {})
        : pd_(pd), rules_(rules) {}

    AntennaResult check();

private:
    const PhysicalDesign& pd_;
    AntennaRules rules_;

    double wire_area_for_net(int net_id, int layer) const;
    double gate_area_for_net(int net_id) const;
};

} // namespace sf
