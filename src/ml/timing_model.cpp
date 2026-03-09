// SiliconForge — ML Timing Predictor
#include "ml/timing_model.hpp"
#include <chrono>

namespace sf {

double MlTimingPredictor::estimate_wire_capacitance(NetId nid) const {
    if (nid >= (int)pd_.nets.size()) return 0;
    auto& pnet = pd_.nets[nid];
    // Very simple linear model: C_wire = alpha * HPWL
    // where HPWL is half-perimeter wire length
    if (pnet.cell_ids.size() < 2) return 0;
    
    double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
    for (auto cid : pnet.cell_ids) {
        auto& c = pd_.cells[cid];
        xmin = std::min(xmin, c.position.x);
        xmax = std::max(xmax, c.position.x);
        ymin = std::min(ymin, c.position.y);
        ymax = std::max(ymax, c.position.y);
    }
    double hpwl = (xmax - xmin) + (ymax - ymin);
    
    // Assume 0.2 fF/um
    return hpwl * 0.2e-15;
}

double MlTimingPredictor::estimate_wire_resistance(NetId nid) const {
   if (nid >= (int)pd_.nets.size()) return 0;
    auto& pnet = pd_.nets[nid];
    if (pnet.cell_ids.size() < 2) return 0;
    
    double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
    for (auto cid : pnet.cell_ids) {
        auto& c = pd_.cells[cid];
        xmin = std::min(xmin, c.position.x);
        xmax = std::max(xmax, c.position.x);
        ymin = std::min(ymin, c.position.y);
        ymax = std::max(ymax, c.position.y);
    }
    double hpwl = (xmax - xmin) + (ymax - ymin);
    
    // Assume 0.1 ohm/um
    return hpwl * 0.1;
}

TimingPrediction MlTimingPredictor::predict(double clock_period) const {
    auto t0 = std::chrono::high_resolution_clock::now();
    TimingPrediction r;
    
    // In a real flow, this would run a fast path-based STA substituting
    // the ML-predicted RC delays. We emulate the feature extraction and 
    // run the actual STA engine but with "model" scaled delays.
    
    // Run normal STA (since our STA already falls back to HPWL if unrouted)
    StaEngine sta(nl_, lib_);
    auto base_sta = sta.analyze(clock_period);
    
    // ML adjustment factor: apply a learned scalar based on historical routing detours
    double detours_factor = 1.15; // 15% worse delay after actual routing
    
    // Shift slack negatively by the detour penalty factor based on delay
    r.predicted_wns = base_sta.wns - (clock_period * (detours_factor - 1.0));
    r.predicted_tns = base_sta.tns * detours_factor;
    r.critical_paths = base_sta.critical_paths; // Keep top paths

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return r;
}

} // namespace sf
