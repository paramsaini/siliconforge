// SiliconForge — MCMM Analyzer Implementation
#include "timing/mcmm.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace sf {

void McmmAnalyzer::load_default_corners() {
    corners_.clear();
    // Standard PVT corners
    corners_.push_back({"ss_0p81v_125c", 0.81, 125, PvtCorner::SLOW, 1.5, 0.8});
    corners_.push_back({"tt_0p90v_25c",  0.90, 25,  PvtCorner::TYPICAL, 1.0, 1.0});
    corners_.push_back({"ff_0p99v_m40c", 0.99, -40, PvtCorner::FAST, 0.7, 1.3});
}

void McmmAnalyzer::load_default_modes(double base_freq_mhz) {
    modes_.clear();
    modes_.push_back({"func", base_freq_mhz, 0.15, "Functional mode"});
    modes_.push_back({"scan", base_freq_mhz * 0.1, 0.5, "Scan test mode"});
    modes_.push_back({"standby", 0, 0.001, "Standby / retention"});
}

McmmResult McmmAnalyzer::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    McmmResult r;
    r.corners = (int)corners_.size();
    r.modes = (int)modes_.size();
    r.worst_wns = 1e18;

    for (auto& corner : corners_) {
        for (auto& mode : modes_) {
            McmmScenario scenario;
            scenario.corner = corner;
            scenario.mode = mode;

            // STA with scaled delays
            if (mode.clock_freq_mhz > 0) {
                double period = 1000.0 / mode.clock_freq_mhz; // ns
                StaEngine sta(nl_);
                scenario.sta = sta.analyze(period * corner.delay_scale);

                if (scenario.sta.wns < r.worst_wns) {
                    r.worst_wns = scenario.sta.wns;
                    r.worst_tns = scenario.sta.tns;
                    r.worst_corner = corner.name + "/" + mode.name;
                }
            }

            // Power with scaled voltage
            PowerAnalyzer pa(nl_);
            double freq = mode.clock_freq_mhz > 0 ? mode.clock_freq_mhz : 1.0;
            scenario.power = pa.analyze(freq, corner.voltage, mode.switching_activity);
            scenario.power.total_power_mw *= corner.power_scale;
            scenario.power.dynamic_power_mw *= corner.power_scale;

            if (scenario.power.total_power_mw > r.max_power_mw) {
                r.max_power_mw = scenario.power.total_power_mw;
                r.max_power_scenario = corner.name + "/" + mode.name;
            }

            r.details.push_back(scenario);
            r.scenarios++;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.scenarios) + " scenarios analyzed. Worst WNS: " +
                std::to_string(r.worst_wns) + "ns at " + r.worst_corner +
                ". Max power: " + std::to_string(r.max_power_mw) + "mW at " + r.max_power_scenario;
    return r;
}

} // namespace sf
