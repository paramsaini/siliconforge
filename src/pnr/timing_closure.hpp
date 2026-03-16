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

    // MCMM (Multi-Corner Multi-Mode) analysis
    bool enable_mcmm = false;
    std::vector<std::string> corner_names;         // corner names for MCMM
    std::vector<double> corner_derates;            // late cell derate per corner (e.g., 1.0, 1.15)

    // Feedback loop controls
    bool enable_incremental_sta = true;
    bool enable_criticality_routing = true;
    bool enable_timing_placement = true;
    double criticality_weight = 10.0;              // max weight multiplier for critical nets
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

    // Enhanced tracking
    std::vector<double> tns_per_iteration;
    std::vector<std::vector<int>> slack_histogram;  // per-iteration, 10 buckets
    std::vector<std::string> iteration_summaries;

    // MCMM per-corner results
    std::vector<double> per_corner_wns;
    std::vector<std::string> corner_names;
};

// Per-net criticality metric for timing-driven feedback
struct NetCriticality {
    int net_id;
    double slack;
    double criticality;  // 1 - slack/clock_period, clamped [0,1]
};

class TimingClosureEngine {
public:
    TimingClosureEngine(Netlist& nl, PhysicalDesign& pd,
                        const LibertyLibrary* lib = nullptr);

    TimingClosureResult run(const TimingClosureConfig& cfg = {});

    // Enhanced closure loop with criticality feedback into placer/router
    TimingClosureResult run_enhanced(const TimingClosureConfig& cfg = {});

    // Inject placer/router for feedback (void* to avoid circular deps)
    void set_placer(void* placer) { placer_ = placer; }
    void set_global_router(void* grouter) { grouter_ = grouter; }

    // Individual steps (for manual control)
    double run_sta(double clock_period);
    int run_post_route_opt(double target_wns);
    int run_hold_fix();

private:
    Netlist& nl_;
    PhysicalDesign& pd_;
    const LibertyLibrary* lib_;

    // Feedback handles (cast to concrete types in .cpp)
    void* placer_ = nullptr;
    void* grouter_ = nullptr;

    // Criticality computation and feedback helpers
    std::vector<NetCriticality> compute_criticality(const StaResult& sta, double clock_period);
    void update_placer_weights(const std::vector<NetCriticality>& crits, double crit_weight);
    void update_router_timing(const std::vector<NetCriticality>& crits);
    std::vector<int> build_slack_histogram(const StaResult& sta, double clock_period);
    std::string format_iteration_summary(int iter, const StaResult& sta);
};

} // namespace sf
