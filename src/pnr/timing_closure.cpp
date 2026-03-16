// SiliconForge — Timing Closure Engine implementation

#include "pnr/timing_closure.hpp"
#include "pnr/detailed_router_v2.hpp"
#include "pnr/placer.hpp"
#include "pnr/global_router.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <unordered_map>

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

        // Incremental re-route: gate sizing / buffer insertion changes physical
        // netlist, so wire parasitics need updating before STA.
        {
            int num_layers = 0;
            for (auto& w : pd_.wires)
                num_layers = std::max(num_layers, w.layer + 1);
            if (num_layers < 2) num_layers = 2;
            DetailedRouterV2 router(pd_, num_layers);
            router.route(1); // single-threaded incremental re-route
        }

        // Re-run STA after optimization + re-route
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

// ---------------------------------------------------------------------------
// Enhanced timing closure: criticality feedback into placer and router
// ---------------------------------------------------------------------------

std::vector<NetCriticality> TimingClosureEngine::compute_criticality(
    const StaResult& sta, double clock_period) {
    std::vector<NetCriticality> crits;
    if (clock_period <= 0) return crits;

    // Build worst-slack-per-net map from STA critical paths
    std::unordered_map<int, double> net_worst_slack;
    for (const auto& path : sta.critical_paths) {
        double path_slack = path.slack;
        for (int nid : path.nets) {
            auto it = net_worst_slack.find(nid);
            if (it == net_worst_slack.end() || path_slack < it->second) {
                net_worst_slack[nid] = path_slack;
            }
        }
    }

    // Assign criticality for every net in the design
    int nn = static_cast<int>(nl_.num_nets());
    crits.reserve(nn);
    for (int i = 0; i < nn; ++i) {
        double slack = clock_period; // default: non-critical
        auto it = net_worst_slack.find(i);
        if (it != net_worst_slack.end()) {
            slack = it->second;
        }
        double crit = std::max(0.0, std::min(1.0, 1.0 - slack / clock_period));
        crits.push_back({i, slack, crit});
    }
    return crits;
}

void TimingClosureEngine::update_placer_weights(
    const std::vector<NetCriticality>& crits, double crit_weight) {
    if (!placer_) return;
    auto* placer = static_cast<AnalyticalPlacer*>(placer_);
    for (const auto& nc : crits) {
        if (nc.criticality > 0.1) {
            double weight = 1.0 + nc.criticality * crit_weight;
            placer->set_net_weight(nc.net_id, weight);
        }
    }
}

void TimingClosureEngine::update_router_timing(
    const std::vector<NetCriticality>& crits) {
    if (!grouter_) return;
    auto* router = static_cast<GlobalRouter*>(grouter_);
    for (const auto& nc : crits) {
        NetTimingInfo info;
        info.slack = nc.slack;
        info.criticality = nc.criticality;
        info.is_critical = (nc.slack < 0);
        router->set_net_timing(nc.net_id, info);
    }
}

std::vector<int> TimingClosureEngine::build_slack_histogram(
    const StaResult& sta, double clock_period) {
    // 10 buckets: <-1, [-1,-0.5), [-0.5,-0.2), [-0.2,-0.1), [-0.1,0),
    //             [0,0.1), [0.1,0.2), [0.2,0.5), [0.5,1), >=1
    std::vector<int> hist(10, 0);

    int neg_count = sta.num_violations;
    int total_endpoints = std::max(1, static_cast<int>(nl_.num_gates()));
    int pos_count = std::max(0, total_endpoints - neg_count);

    // Distribute violations into negative buckets based on WNS magnitude
    if (neg_count > 0) {
        if (sta.wns < -1.0)      hist[0] = neg_count;
        else if (sta.wns < -0.5) hist[1] = neg_count;
        else if (sta.wns < -0.2) hist[2] = neg_count;
        else if (sta.wns < -0.1) hist[3] = neg_count;
        else if (sta.wns < 0)    hist[4] = neg_count;
    }

    // Place positive-slack endpoints in a representative bucket
    if (pos_count > 0) {
        hist[7] = pos_count;  // [0.2, 0.5) bucket — typical for non-critical paths
    }
    return hist;
}

std::string TimingClosureEngine::format_iteration_summary(int iter, const StaResult& sta) {
    std::ostringstream ss;
    ss << "Iter " << iter << ": WNS=" << sta.wns << "ns TNS=" << sta.tns
       << "ns violations=" << sta.num_violations;
    return ss.str();
}

TimingClosureResult TimingClosureEngine::run_enhanced(const TimingClosureConfig& cfg) {
    auto t0 = std::chrono::steady_clock::now();
    TimingClosureResult result;

    // Initial STA baseline
    StaEngine sta_init(nl_, lib_, &pd_);
    StaResult sta_res = sta_init.analyze(cfg.clock_period);

    result.initial_wns = sta_res.wns;
    result.initial_tns = sta_res.tns;
    result.wns_per_iteration.push_back(sta_res.wns);
    result.tns_per_iteration.push_back(sta_res.tns);
    result.slack_histogram.push_back(build_slack_histogram(sta_res, cfg.clock_period));
    result.iteration_summaries.push_back(format_iteration_summary(0, sta_res));

    // Populate MCMM corner names even for early return
    if (cfg.enable_mcmm && !cfg.corner_names.empty()) {
        result.corner_names = cfg.corner_names;
        result.per_corner_wns.resize(cfg.corner_names.size(), sta_res.wns);
    }

    // Already met timing?
    if (sta_res.wns >= cfg.target_wns && sta_res.tns >= cfg.target_tns) {
        result.converged = true;
        result.final_wns = sta_res.wns;
        result.final_tns = sta_res.tns;
        result.iterations_run = 0;
        auto t1 = std::chrono::steady_clock::now();
        result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        result.summary = "Timing already met.";
        return result;
    }

    int stall_count = 0;
    double prev_wns = sta_res.wns;

    for (int iter = 0; iter < cfg.max_iterations; ++iter) {
        result.iterations_run = iter + 1;

        // 1. Compute per-net criticality from current STA snapshot
        auto crits = compute_criticality(sta_res, cfg.clock_period);

        // 2. Feed criticality back into the analytical placer
        if (cfg.enable_timing_placement && placer_) {
            update_placer_weights(crits, cfg.criticality_weight);
        }

        // 3. Feed criticality back into the global router
        if (cfg.enable_criticality_routing && grouter_) {
            update_router_timing(crits);
        }

        // 4. Post-route optimization (gate sizing, Vt swap, buffer insertion)
        PostRouteOptimizer opt(nl_, pd_, lib_);
        opt.set_clock_period(cfg.clock_period);
        PostRouteConfig pcfg;
        pcfg.enable_vt_swap = cfg.enable_vt_swap;
        pcfg.enable_buffer_insertion = cfg.enable_sizing;
        pcfg.enable_hold_fix = false;
        opt.set_config(pcfg);
        PostRouteResult opt_res = opt.optimize(cfg.target_wns);
        result.setup_violations_fixed += opt_res.setup_fixes;

        // 5. Incremental re-route to update parasitics after sizing changes
        {
            int num_layers = 0;
            for (auto& w : pd_.wires)
                num_layers = std::max(num_layers, w.layer + 1);
            if (num_layers < 2) num_layers = 2;
            DetailedRouterV2 router(pd_, num_layers);
            router.route(1);
        }

        // 6. Re-run STA (MCMM: worst across corners, or single-corner)
        StaEngine sta(nl_, lib_, &pd_);
        if (cfg.enable_mcmm && !cfg.corner_names.empty()) {
            double worst_wns = 1e18;
            double worst_tns = 0;
            result.per_corner_wns.clear();
            result.corner_names = cfg.corner_names;
            for (size_t c = 0; c < cfg.corner_names.size(); ++c) {
                double derate = (c < cfg.corner_derates.size()) ? cfg.corner_derates[c] : 1.0;
                StaEngine sta_corner(nl_, lib_, &pd_);
                if (derate != 1.0) {
                    sta_corner.enable_ocv(derate, 2.0 - derate);
                }
                StaResult corner_res = sta_corner.analyze(cfg.clock_period);
                result.per_corner_wns.push_back(corner_res.wns);
                if (corner_res.wns < worst_wns) {
                    worst_wns = corner_res.wns;
                    sta_res = corner_res;
                }
                worst_tns = std::min(worst_tns, corner_res.tns);
            }
            sta_res.wns = worst_wns;
            sta_res.tns = worst_tns;
        } else {
            sta_res = sta.analyze(cfg.clock_period);
        }

        result.wns_per_iteration.push_back(sta_res.wns);
        result.tns_per_iteration.push_back(sta_res.tns);
        result.slack_histogram.push_back(build_slack_histogram(sta_res, cfg.clock_period));
        result.iteration_summaries.push_back(format_iteration_summary(iter + 1, sta_res));

        // Check convergence
        if (sta_res.wns >= cfg.target_wns && sta_res.tns >= cfg.target_tns) {
            result.converged = true;
            break;
        }

        double improvement = sta_res.wns - prev_wns;
        if (improvement < cfg.convergence_threshold) {
            ++stall_count;
        } else {
            stall_count = 0;
        }
        if (stall_count >= cfg.stall_limit) break;
        prev_wns = sta_res.wns;
    }

    // Final hold-fix pass
    if (cfg.enable_hold_fix) {
        PostRouteOptimizer hold_opt(nl_, pd_, lib_);
        hold_opt.set_clock_period(cfg.clock_period);
        PostRouteOptimizer::HoldFixResult hres = hold_opt.fix_hold_violations();
        result.hold_violations_fixed = hres.num_fixed;

        StaEngine sta_final(nl_, lib_, &pd_);
        sta_res = sta_final.analyze(cfg.clock_period);
    }

    result.final_wns = sta_res.wns;
    result.final_tns = sta_res.tns;

    auto t1 = std::chrono::steady_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::ostringstream ss;
    ss << "Enhanced timing closure " << (result.converged ? "CONVERGED" : "DID NOT CONVERGE")
       << " after " << result.iterations_run << " iteration(s). "
       << "WNS: " << result.initial_wns << " -> " << result.final_wns << " ns, "
       << "TNS: " << result.initial_tns << " -> " << result.final_tns << " ns.";
    result.summary = ss.str();

    return result;
}

} // namespace sf
