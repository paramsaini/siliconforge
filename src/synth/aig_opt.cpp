// SiliconForge — AIG Optimization Implementation
#include "synth/aig_opt.hpp"
#include <algorithm>
#include <unordered_set>
#include <functional>
#include <unordered_map>
#include <queue>
#include <chrono>
#include <iostream>
#include <cassert>

namespace sf {

uint32_t AigOptimizer::compute_depth(const AigGraph& aig) {
    std::unordered_map<uint32_t, uint32_t> depth;
    depth[0] = 0;
    for (auto v : aig.inputs()) depth[v] = 0;

    uint32_t max_d = 0;
    for (uint32_t v = 1; v <= aig.max_var(); ++v) {
        if (!aig.is_and(v)) continue;
        const auto& nd = aig.and_node(v);
        uint32_t d0 = depth.count(aig_var(nd.fanin0)) ? depth[aig_var(nd.fanin0)] : 0;
        uint32_t d1 = depth.count(aig_var(nd.fanin1)) ? depth[aig_var(nd.fanin1)] : 0;
        depth[v] = std::max(d0, d1) + 1;
        max_d = std::max(max_d, depth[v]);
    }
    return max_d;
}

void AigOptimizer::sweep() {
    // Mark all nodes reachable from outputs
    std::unordered_set<uint32_t> reachable;

    std::function<void(AigLit)> mark = [&](AigLit lit) {
        uint32_t v = aig_var(lit);
        if (v == 0 || reachable.count(v)) return;
        reachable.insert(v);
        if (aig_.is_and(v)) {
            const auto& nd = aig_.and_node(v);
            mark(nd.fanin0);
            mark(nd.fanin1);
        }
    };

    for (auto olit : aig_.outputs()) mark(olit);
    for (auto& latch : aig_.latches()) mark(latch.next);

    // Count swept
    uint32_t swept = 0;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (aig_.is_and(v) && !reachable.count(v)) swept++;
    }
    if (swept > 0) {
        // Build new AIG with only reachable nodes
        AigGraph new_aig;
        std::unordered_map<uint32_t, AigLit> var_map;
        var_map[0] = AIG_FALSE;

        // Recreate inputs
        for (size_t i = 0; i < aig_.num_inputs(); ++i) {
            uint32_t old_v = aig_.inputs()[i];
            std::string iname = i < aig_.input_names().size() ? aig_.input_names()[i] : "";
            AigLit new_lit = new_aig.create_input(iname);
            var_map[old_v] = new_lit;
        }

        // Recreate reachable AND gates in order
        auto translate = [&](AigLit lit) -> AigLit {
            auto it = var_map.find(aig_var(lit));
            if (it == var_map.end()) return AIG_FALSE;
            return aig_sign(lit) ? aig_not(it->second) : it->second;
        };

        for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
            if (!aig_.is_and(v) || !reachable.count(v)) continue;
            const auto& nd = aig_.and_node(v);
            var_map[v] = new_aig.create_and(translate(nd.fanin0), translate(nd.fanin1));
        }

        // Recreate outputs
        for (size_t i = 0; i < aig_.num_outputs(); ++i)
            new_aig.add_output(translate(aig_.outputs()[i]));

        // Recreate latches
        for (auto& l : aig_.latches())
            new_aig.add_latch(translate(l.next), l.init);

        aig_ = std::move(new_aig);
    }
}

void AigOptimizer::balance() {
    // Collect AND chains and rebuild as balanced trees
    // For each multi-input AND tree, collect all leaves and rebuild balanced

    // Simple approach: for each output, find chains of same-polarity ANDs
    // and balance them. We do one pass.

    AigGraph new_aig;
    std::unordered_map<uint32_t, AigLit> var_map;
    var_map[0] = AIG_FALSE;

    for (size_t i = 0; i < aig_.num_inputs(); ++i) {
        uint32_t v = aig_.inputs()[i];
        std::string iname = i < aig_.input_names().size() ? aig_.input_names()[i] : "";
        var_map[v] = new_aig.create_input(iname);
    }

    // Collect AND chain leaves
    std::function<AigLit(AigLit)> rebuild;

    std::function<void(uint32_t, std::vector<AigLit>&)> collect_and_leaves;
    collect_and_leaves = [&](uint32_t v, std::vector<AigLit>& leaves) {
        if (!aig_.is_and(v)) {
            auto it = var_map.find(v);
            leaves.push_back(it != var_map.end() ? it->second : AIG_FALSE);
            return;
        }
        const auto& nd = aig_.and_node(v);
        // If fanin is non-inverted AND, recurse into it
        if (!aig_sign(nd.fanin0) && aig_.is_and(aig_var(nd.fanin0)))
            collect_and_leaves(aig_var(nd.fanin0), leaves);
        else {
            AigLit mapped = rebuild(nd.fanin0);
            leaves.push_back(mapped);
        }
        if (!aig_sign(nd.fanin1) && aig_.is_and(aig_var(nd.fanin1)))
            collect_and_leaves(aig_var(nd.fanin1), leaves);
        else {
            AigLit mapped = rebuild(nd.fanin1);
            leaves.push_back(mapped);
        }
    };

    rebuild = [&](AigLit lit) -> AigLit {
        uint32_t v = aig_var(lit);
        if (var_map.count(v)) {
            AigLit mapped = var_map[v];
            return aig_sign(lit) ? aig_not(mapped) : mapped;
        }

        if (!aig_.is_and(v)) {
            var_map[v] = AIG_FALSE;
            return aig_sign(lit) ? AIG_TRUE : AIG_FALSE;
        }

        // Collect all leaves of the AND tree
        std::vector<AigLit> leaves;
        collect_and_leaves(v, leaves);

        // Build balanced tree
        while (leaves.size() > 1) {
            std::vector<AigLit> next_level;
            for (size_t i = 0; i + 1 < leaves.size(); i += 2)
                next_level.push_back(new_aig.create_and(leaves[i], leaves[i+1]));
            if (leaves.size() & 1)
                next_level.push_back(leaves.back());
            leaves = std::move(next_level);
        }

        var_map[v] = leaves[0];
        return aig_sign(lit) ? aig_not(leaves[0]) : leaves[0];
    };

    // Process all AND gates in order
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v) || var_map.count(v)) continue;
        rebuild(aig_make(v));
    }

    for (size_t i = 0; i < aig_.num_outputs(); ++i)
        new_aig.add_output(rebuild(aig_.outputs()[i]));
    for (auto& l : aig_.latches())
        new_aig.add_latch(rebuild(l.next), l.init);

    aig_ = std::move(new_aig);
}

void AigOptimizer::rewrite() {
    // Simple local rewriting: scan for patterns and simplify
    // Pattern 1: AND(AND(a,b), AND(a,c)) → AND(a, AND(b,c)) — sharing
    // We just do a sweep + structural hash rebuild for now
    sweep();
}

void AigOptimizer::refactor() {
    // Collapse small sub-DAGs and rebuild
    // For now, equivalent to sweep + rebuild
    sweep();
}

OptStats AigOptimizer::optimize(int num_passes) {
    auto t0 = std::chrono::high_resolution_clock::now();

    OptStats stats;
    stats.initial_ands = aig_.num_ands();
    stats.initial_depth = compute_depth(aig_);

    for (int p = 0; p < num_passes; ++p) {
        rewrite();
        balance();
        refactor();
        stats.passes++;
    }

    stats.final_ands = aig_.num_ands();
    stats.final_depth = compute_depth(aig_);
    auto t1 = std::chrono::high_resolution_clock::now();
    stats.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    return stats;
}

} // namespace sf
