// SiliconForge — Multi-Vt Optimizer Implementation
#include "synth/multi_vt.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>

namespace sf {

static constexpr double kBaseDelay = 0.1; // ns per gate

// ─── Depth helpers (original) ────────────────────────────────────────────────

int MultiVtOptimizer::compute_depth(GateId gid, std::vector<int>& cache) const {
    if (gid < 0 || gid >= (int)nl_.num_gates()) return 0;
    if (cache[gid] >= 0) return cache[gid];

    auto& g = nl_.gate(gid);
    if (g.type == GateType::INPUT) { cache[gid] = 0; return 0; }

    int max_d = 0;
    for (auto inp : g.inputs) {
        auto& net = nl_.net(inp);
        if (net.driver >= 0) {
            max_d = std::max(max_d, compute_depth(net.driver, cache));
        }
    }
    cache[gid] = max_d + 1;
    return cache[gid];
}

int MultiVtOptimizer::max_depth() const {
    std::vector<int> cache(nl_.num_gates(), -1);
    int md = 0;
    for (size_t g = 0; g < nl_.num_gates(); ++g)
        md = std::max(md, compute_depth(g, cache));
    return md;
}

// ─── Vt factor helpers ───────────────────────────────────────────────────────

double MultiVtOptimizer::delay_factor(VtType vt) const {
    switch (vt) {
        case VtType::LVT: return cfg_.lvt_speed_factor;
        case VtType::HVT: return cfg_.hvt_speed_factor;
        default:          return 1.0;
    }
}

double MultiVtOptimizer::leakage_factor(VtType vt) const {
    switch (vt) {
        case VtType::LVT: return cfg_.lvt_leakage_factor;
        case VtType::HVT: return cfg_.hvt_leakage_factor;
        default:          return 1.0;
    }
}

// ─── Static timing analysis ──────────────────────────────────────────────────

void MultiVtOptimizer::forward_propagate(std::vector<double>& arrival) const {
    auto topo = nl_.topo_order();
    for (auto gid : topo) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT) {
            arrival[gid] = 0.0;
            continue;
        }

        double max_arr = 0.0;
        for (auto inp_net : g.inputs) {
            auto& n = nl_.net(inp_net);
            if (n.driver >= 0) {
                max_arr = std::max(max_arr, arrival[n.driver]);
            }
        }

        VtType vt = (gid < (GateId)current_assignment_.size())
                         ? current_assignment_[gid]
                         : VtType::SVT;
        double gate_delay = kBaseDelay * delay_factor(vt);
        if (g.type == GateType::OUTPUT) gate_delay = 0.0;

        arrival[gid] = max_arr + gate_delay;
    }
}

void MultiVtOptimizer::backward_propagate(std::vector<double>& required,
                                           double clock_period) const {
    auto topo = nl_.topo_order();
    // Initialize all to a large value
    std::fill(required.begin(), required.end(),
              std::numeric_limits<double>::max());

    // Outputs get the clock period as required time
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::OUTPUT) {
            required[gid] = clock_period;
        } else if (g.output >= 0) {
            auto& out_net = nl_.net(g.output);
            if (out_net.fanout.empty()) {
                // Dangling output — treat as endpoint
                required[gid] = clock_period;
            }
        }
    }

    // Reverse topological order
    for (auto it = topo.rbegin(); it != topo.rend(); ++it) {
        GateId gid = *it;
        auto& g = nl_.gate(gid);

        // Propagate required time from fanout gates back through input nets
        if (g.output >= 0) {
            auto& out_net = nl_.net(g.output);
            for (auto fanout_gid : out_net.fanout) {
                auto& fo_gate = nl_.gate(fanout_gid);
                VtType fo_vt = (fanout_gid < (GateId)current_assignment_.size())
                                   ? current_assignment_[fanout_gid]
                                   : VtType::SVT;
                double fo_delay = kBaseDelay * delay_factor(fo_vt);
                if (fo_gate.type == GateType::OUTPUT) fo_delay = 0.0;

                required[gid] = std::min(required[gid],
                                         required[fanout_gid] - fo_delay);
            }
        }

        // If still max, set to clock period (unreachable gate)
        if (required[gid] == std::numeric_limits<double>::max()) {
            required[gid] = clock_period;
        }
    }
}

MultiVtOptimizer::TimingInfo MultiVtOptimizer::compute_static_timing() const {
    size_t n = nl_.num_gates();
    TimingInfo ti;
    ti.gate_arrival.resize(n, 0.0);
    ti.gate_required.resize(n, 0.0);
    ti.gate_slack.resize(n, 0.0);

    // Forward pass — compute arrival times
    // Need a mutable copy to call propagation helpers (they read current_assignment_)
    auto* self = const_cast<MultiVtOptimizer*>(this);
    self->forward_propagate(ti.gate_arrival);

    // Clock period = max arrival at any output (or any gate)
    ti.clock_period = 0.0;
    for (size_t gid = 0; gid < n; ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::OUTPUT) {
            ti.clock_period = std::max(ti.clock_period, ti.gate_arrival[gid]);
        }
    }
    // Ensure a minimum clock period
    if (ti.clock_period < kBaseDelay) {
        ti.clock_period = kBaseDelay;
    }

    // Backward pass — compute required times
    self->backward_propagate(ti.gate_required, ti.clock_period);

    // Slack computation
    for (size_t gid = 0; gid < n; ++gid) {
        ti.gate_slack[gid] = ti.gate_required[gid] - ti.gate_arrival[gid];
    }

    return ti;
}

// ─── Sensitivity analysis ────────────────────────────────────────────────────

std::vector<MultiVtOptimizer::VtSensitivity>
MultiVtOptimizer::compute_sensitivity() {
    if (current_assignment_.size() != nl_.num_gates()) {
        current_assignment_.assign(nl_.num_gates(), VtType::SVT);
    }

    std::vector<VtSensitivity> result;
    result.reserve(nl_.num_gates());

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;

        VtType cur_vt = current_assignment_[gid];
        double cur_delay = kBaseDelay * delay_factor(cur_vt);
        double lvt_delay = kBaseDelay * delay_factor(VtType::LVT);

        double timing_gain = cur_delay - lvt_delay;  // positive = improvement
        double cur_leak = leakage_factor(cur_vt);
        double lvt_leak = leakage_factor(VtType::LVT);
        double leakage_cost = lvt_leak - cur_leak;    // positive = cost

        double efficiency = (leakage_cost > 1e-12)
                                ? timing_gain / leakage_cost
                                : 0.0;

        result.push_back({(GateId)gid, timing_gain, leakage_cost, efficiency});
    }

    // Sort by efficiency descending
    std::sort(result.begin(), result.end(),
              [](const VtSensitivity& a, const VtSensitivity& b) {
                  return a.efficiency > b.efficiency;
              });

    return result;
}

// ─── Leakage recovery ────────────────────────────────────────────────────────

int MultiVtOptimizer::leakage_recovery(double timing_margin_ns) {
    if (current_assignment_.size() != nl_.num_gates()) return 0;

    auto ti = compute_static_timing();
    int recovered = 0;

    // Build list of gates sorted by slack (most positive first)
    struct SlackEntry { GateId gid; double slack; };
    std::vector<SlackEntry> entries;
    entries.reserve(nl_.num_gates());
    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
        entries.push_back({(GateId)gid, ti.gate_slack[gid]});
    }
    std::sort(entries.begin(), entries.end(),
              [](const SlackEntry& a, const SlackEntry& b) {
                  return a.slack > b.slack;
              });

    for (auto& [gid, slack] : entries) {
        if (slack <= timing_margin_ns) continue;

        VtType cur_vt = current_assignment_[gid];
        VtType new_vt;
        if (cur_vt == VtType::LVT) {
            new_vt = VtType::SVT;
        } else if (cur_vt == VtType::SVT) {
            new_vt = VtType::HVT;
        } else {
            continue; // already HVT
        }

        // Try the swap
        double old_delay = kBaseDelay * delay_factor(cur_vt);
        double new_delay = kBaseDelay * delay_factor(new_vt);
        double delay_increase = new_delay - old_delay;

        // Accept swap only if slack can absorb the delay increase
        if (slack - delay_increase > timing_margin_ns) {
            current_assignment_[gid] = new_vt;
            recovered++;
            // Update slack for downstream awareness (approximate)
            slack -= delay_increase;
        }
    }

    return recovered;
}

// ─── Iterative optimization ──────────────────────────────────────────────────

MultiVtResult MultiVtOptimizer::optimize_iterative(int max_iterations) {
    auto t0 = std::chrono::high_resolution_clock::now();

    size_t n = nl_.num_gates();

    // Step 1: Start with all cells as SVT
    current_assignment_.assign(n, VtType::SVT);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Step 2: Compute timing
        auto ti = compute_static_timing();

        // Count violations (negative slack)
        int violation_count = 0;
        for (size_t gid = 0; gid < n; ++gid) {
            auto& g = nl_.gate(gid);
            if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
            if (ti.gate_slack[gid] < 0.0) violation_count++;
        }

        if (violation_count == 0 && iter > 0) {
            // Timing met — run leakage recovery and stop
            leakage_recovery(0.5);
            break;
        }

        // Step 3: Compute sensitivity
        auto sens = compute_sensitivity();

        // Step 4: Swap cells with worst slack to LVT
        // Prioritize by: negative slack first, then by sensitivity efficiency
        struct SwapCandidate {
            GateId gid;
            double slack;
            double efficiency;
        };
        std::vector<SwapCandidate> candidates;
        for (auto& s : sens) {
            auto& g = nl_.gate(s.gate);
            if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
            if (current_assignment_[s.gate] == VtType::LVT) continue;
            candidates.push_back(
                {s.gate, ti.gate_slack[s.gate], s.efficiency});
        }

        // Sort: worst slack first, then efficiency
        std::sort(candidates.begin(), candidates.end(),
                  [](const SwapCandidate& a, const SwapCandidate& b) {
                      if (a.slack != b.slack) return a.slack < b.slack;
                      return a.efficiency > b.efficiency;
                  });

        // Swap top 20% of candidate cells to LVT
        int swap_count = std::max(1, (int)(candidates.size() * 0.2));
        for (int i = 0; i < swap_count && i < (int)candidates.size(); ++i) {
            current_assignment_[candidates[i].gid] = VtType::LVT;
        }

        // Step 5: After last iteration with remaining violations, do recovery
        if (iter == max_iterations - 1) {
            leakage_recovery(0.5);
        }
    }

    // Final leakage recovery pass
    leakage_recovery(0.3);

    // Build result
    MultiVtResult r;
    double total_leakage_before = 0;
    double total_leakage_after = 0;

    for (size_t gid = 0; gid < n; ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
        r.total_cells++;

        VtType vt = current_assignment_[gid];
        switch (vt) {
            case VtType::LVT: r.lvt_cells++; break;
            case VtType::SVT: r.svt_cells++; break;
            case VtType::HVT: r.hvt_cells++; break;
        }

        r.assignments.push_back({(GateId)gid, g.name, vt});

        total_leakage_before += 1.0;
        total_leakage_after += leakage_factor(vt);
    }

    if (total_leakage_before > 0) {
        r.leakage_reduction_pct =
            (1.0 - total_leakage_after / total_leakage_before) * 100;
    }
    if (r.total_cells > 0) {
        double avg_speed =
            (r.lvt_cells * cfg_.lvt_speed_factor + r.svt_cells * 1.0 +
             r.hvt_cells * cfg_.hvt_speed_factor) /
            r.total_cells;
        r.timing_impact_pct = (avg_speed - 1.0) * 100;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Multi-Vt (iterative): " + std::to_string(r.lvt_cells) +
                " LVT, " + std::to_string(r.svt_cells) + " SVT, " +
                std::to_string(r.hvt_cells) + " HVT — leakage " +
                std::to_string((int)r.leakage_reduction_pct) + "% reduction";
    return r;
}

// ─── Enhanced optimize() — uses slack-based when timing is available ─────────

MultiVtResult MultiVtOptimizer::optimize() {
    // If timing info is available, use iterative slack-based optimization
    if (has_timing_) {
        current_assignment_.assign(nl_.num_gates(), VtType::SVT);
        return optimize_iterative();
    }

    // Fallback: original depth-based assignment
    auto t0 = std::chrono::high_resolution_clock::now();
    MultiVtResult r;

    int md = max_depth();
    double critical_threshold = md * (1.0 - cfg_.timing_margin);
    std::vector<int> depth_cache(nl_.num_gates(), -1);

    double total_leakage_before = 0;
    double total_leakage_after = 0;

    for (size_t gid = 0; gid < nl_.num_gates(); ++gid) {
        auto& g = nl_.gate(gid);
        if (g.type == GateType::INPUT || g.type == GateType::OUTPUT) continue;
        r.total_cells++;

        int depth = compute_depth(gid, depth_cache);
        VtType vt;

        if (depth >= critical_threshold) {
            // Critical path — use LVT for speed
            vt = VtType::LVT;
            r.lvt_cells++;
        } else if (depth < critical_threshold * 0.5) {
            // Far from critical — use HVT to save leakage
            vt = VtType::HVT;
            r.hvt_cells++;
        } else {
            // Medium criticality — keep SVT
            vt = VtType::SVT;
            r.svt_cells++;
        }

        r.assignments.push_back({(GateId)gid, g.name, vt});

        // Track leakage
        double base_leakage = 1.0; // normalized
        total_leakage_before += base_leakage;
        switch (vt) {
            case VtType::LVT: total_leakage_after += base_leakage * cfg_.lvt_leakage_factor; break;
            case VtType::HVT: total_leakage_after += base_leakage * cfg_.hvt_leakage_factor; break;
            case VtType::SVT: total_leakage_after += base_leakage; break;
        }
    }

    if (total_leakage_before > 0) {
        r.leakage_reduction_pct = (1.0 - total_leakage_after / total_leakage_before) * 100;
    }

    // Timing impact: LVT cells speed up, HVT slow down — net effect
    if (r.total_cells > 0) {
        double avg_speed = (r.lvt_cells * cfg_.lvt_speed_factor +
                           r.svt_cells * 1.0 +
                           r.hvt_cells * cfg_.hvt_speed_factor) / r.total_cells;
        r.timing_impact_pct = (avg_speed - 1.0) * 100;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = "Multi-Vt: " + std::to_string(r.lvt_cells) + " LVT, " +
                std::to_string(r.svt_cells) + " SVT, " +
                std::to_string(r.hvt_cells) + " HVT — leakage " +
                std::to_string((int)r.leakage_reduction_pct) + "% reduction";
    return r;
}

} // namespace sf
