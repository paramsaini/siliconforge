#pragma once
// SiliconForge — ML-driven Timing Predictor
// Predicts post-route timing (WNS/TNS) from placement data before routing
// is performed, using a feature-based analytical model.

#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include <vector>

namespace sf {

struct TimingPrediction {
    double predicted_wns = 0;
    double predicted_tns = 0;
    std::vector<TimingPath> critical_paths; // estimated
    double time_ms = 0;
};

class MlTimingPredictor {
public:
    MlTimingPredictor(const Netlist& nl, const PhysicalDesign& pd, const LibertyLibrary* lib)
        : nl_(nl), pd_(pd), lib_(lib) {}

    // Predict timing before CTS or Routing by estimating RC parasitics
    // from placement L-shape or HPWL.
    TimingPrediction predict(double clock_period) const;

private:
    const Netlist& nl_;
    const PhysicalDesign& pd_;
    const LibertyLibrary* lib_;

    // Extract features for a net
    double estimate_wire_capacitance(NetId nid) const;
    double estimate_wire_resistance(NetId nid) const;
};

} // namespace sf
