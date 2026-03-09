// SiliconForge — Multi-Vt Optimizer Implementation
#include "synth/multi_vt.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace sf {

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

MultiVtResult MultiVtOptimizer::optimize() {
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
