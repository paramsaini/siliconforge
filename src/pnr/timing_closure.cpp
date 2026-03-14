// SiliconForge — Timing Closure Engine implementation

#include "pnr/timing_closure.hpp"
#include <chrono>
#include <cmath>
#include <sstream>

namespace sf {

TimingClosureEngine::TimingClosureEngine(Netlist& nl, PhysicalDesign& pd,
                                         const LibertyLibrary* lib)
    : nl_(nl), pd_(pd), lib_(lib) {}

double TimingClosureEngine::run_sta(double clock_period) {
    StaEngine sta(nl_, lib_, &pd_);
    StaResult res = sta.analyze(clock_period);
    return res.wns;
}

int TimingClosureEngine::run_post_route_opt(double target_wns) {
    PostRouteOptimizer opt(nl_, pd_, lib_);
    PostRouteResult res = opt.optimize(target_wns);
    return res.gates_resized + res.buffers_inserted + res.cells_vt_swapped;
}

int TimingClosureEngine::run_hold_fix() {
    PostRouteOptimizer opt(nl_, pd_, lib_);
    PostRouteOptimizer::HoldFixResult res = opt.fix_hold_violations();
    return res.num_fixed;
}

TimingClosureResult TimingClosureEngine::run(const TimingClosureConfig& cfg) {
    auto t0 = std::chrono::steady_clock::now();
    TimingClosureResult result;

    // --- Initial STA to establish baseline ---
    StaEngine sta_init(nl_, lib_, &pd_);
    StaResult sta_res = sta_init.analyze(cfg.clock_period);

    result.initial_wns = sta_res.wns;
    result.initial_tns = sta_res.tns;
    result.wns_per_iteration.push_back(sta_res.wns);

    // Check if already converged
    if (sta_res.wns >= cfg.target_wns && sta_res.tns >= cfg.target_tns) {
        result.converged = true;
        result.final_wns = sta_res.wns;
        result.final_tns = sta_res.tns;
        result.iterations_run = 0;
        auto t1 = std::chrono::steady_clock::now();
        result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        result.summary = "Timing already met — no optimization needed.";
        return result;
    }

    // --- Iterative closure loop ---
    int stall_count = 0;
    double prev_wns = sta_res.wns;

    for (int iter = 0; iter < cfg.max_iterations; ++iter) {
        result.iterations_run = iter + 1;

        // Post-route optimization pass
        PostRouteOptimizer opt(nl_, pd_, lib_);
        opt.set_clock_period(cfg.clock_period);

        PostRouteConfig pcfg;
        pcfg.enable_vt_swap = cfg.enable_vt_swap;
        pcfg.enable_buffer_insertion = cfg.enable_sizing;
        pcfg.enable_hold_fix = false; // hold fix is a separate final pass
        opt.set_config(pcfg);

        PostRouteResult opt_res = opt.optimize(cfg.target_wns);
        result.setup_violations_fixed += opt_res.setup_fixes;

        // Re-run STA after optimization
        StaEngine sta(nl_, lib_, &pd_);
        sta_res = sta.analyze(cfg.clock_period);
        result.wns_per_iteration.push_back(sta_res.wns);

        // Check convergence: timing met
        if (sta_res.wns >= cfg.target_wns && sta_res.tns >= cfg.target_tns) {
            result.converged = true;
            break;
        }

        // Check stall: WNS improvement below threshold
        double improvement = sta_res.wns - prev_wns;
        if (improvement < cfg.convergence_threshold) {
            ++stall_count;
        } else {
            stall_count = 0;
        }
        if (stall_count >= cfg.stall_limit) {
            break; // stalled — no meaningful progress
        }

        prev_wns = sta_res.wns;
    }

    // --- Final hold-fix pass ---
    if (cfg.enable_hold_fix) {
        PostRouteOptimizer hold_opt(nl_, pd_, lib_);
        hold_opt.set_clock_period(cfg.clock_period);
        PostRouteOptimizer::HoldFixResult hres = hold_opt.fix_hold_violations();
        result.hold_violations_fixed = hres.num_fixed;

        // Final STA to capture post-hold-fix numbers
        StaEngine sta_final(nl_, lib_, &pd_);
        sta_res = sta_final.analyze(cfg.clock_period);
    }

    result.final_wns = sta_res.wns;
    result.final_tns = sta_res.tns;

    auto t1 = std::chrono::steady_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Build summary string
    std::ostringstream ss;
    ss << "Timing closure " << (result.converged ? "CONVERGED" : "DID NOT CONVERGE")
       << " after " << result.iterations_run << " iteration(s). "
       << "WNS: " << result.initial_wns << " -> " << result.final_wns << " ns, "
       << "TNS: " << result.initial_tns << " -> " << result.final_tns << " ns. "
       << "Setup fixes: " << result.setup_violations_fixed
       << ", Hold fixes: " << result.hold_violations_fixed
       << ". Time: " << result.time_ms << " ms.";
    result.summary = ss.str();

    return result;
}

} // namespace sf
