#pragma once
// SiliconForge — Timing Closure Engine
// Iterative place → route → STA → fix → re-place convergence loop.
// Integrates StaEngine and PostRouteOptimizer into a unified flow that
// tracks WNS/TNS per iteration and stops on convergence or stall.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include "pnr/post_route_opt.hpp"
#include "pnr/detailed_router_v2.hpp"
#include <string>
#include <vector>

namespace sf {

struct TimingClosureConfig {
    int max_iterations = 10;
    double target_wns = 0.0;            // target worst negative slack (0 = all paths met)
    double target_tns = 0.0;            // target total negative slack
    double clock_period = 10.0;         // ns
    bool enable_hold_fix = true;
    bool enable_vt_swap = true;
    bool enable_sizing = true;
    double convergence_threshold = 0.01; // stop when WNS improves less than this
    int stall_limit = 3;                // stop after N iterations with no improvement
};

struct TimingClosureResult {
    int iterations_run = 0;
    bool converged = false;
    double initial_wns = 0.0;
    double final_wns = 0.0;
    double initial_tns = 0.0;
    double final_tns = 0.0;
    int setup_violations_fixed = 0;
    int hold_violations_fixed = 0;
    double time_ms = 0.0;
    std::string summary;
    std::vector<double> wns_per_iteration;
};

class TimingClosureEngine {
public:
    TimingClosureEngine(Netlist& nl, PhysicalDesign& pd,
                        const LibertyLibrary* lib = nullptr);

    TimingClosureResult run(const TimingClosureConfig& cfg = {});

    // Individual steps (for manual control)
    double run_sta(double clock_period);
    int run_post_route_opt(double target_wns);
    int run_hold_fix();

private:
    Netlist& nl_;
    PhysicalDesign& pd_;
    const LibertyLibrary* lib_;
};

} // namespace sf
