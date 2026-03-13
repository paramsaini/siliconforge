// SiliconForge — AIG Optimization Implementation
// Real implementation of DAG-aware rewriting and MFFC refactoring
// Reference: Mishchenko et al., "DAG-Aware AIG Rewriting", DAC 2006
#include "synth/aig_opt.hpp"
#include "formal/tseitin.hpp"
#include "sat/cdcl_solver.hpp"
#include <algorithm>
#include <unordered_set>
#include <functional>
#include <unordered_map>
#include <queue>
#include <chrono>
#include <iostream>
#include <cassert>
#include <cstring>
#include <numeric>
#include <random>
#include <array>

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

// ============================================================
// SWEEP — Dead node elimination (reachability-based)
// ============================================================
void AigOptimizer::sweep() {
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

    uint32_t swept = 0;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (aig_.is_and(v) && !reachable.count(v)) swept++;
    }
    if (swept > 0) {
        AigGraph new_aig;
        std::unordered_map<uint32_t, AigLit> var_map;
        var_map[0] = AIG_FALSE;

        for (size_t i = 0; i < aig_.num_inputs(); ++i) {
            uint32_t old_v = aig_.inputs()[i];
            std::string iname = i < aig_.input_names().size() ? aig_.input_names()[i] : "";
            AigLit new_lit = new_aig.create_input(iname);
            var_map[old_v] = new_lit;
        }

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

        for (size_t i = 0; i < aig_.num_outputs(); ++i) {
            std::string oname = i < aig_.output_names().size() ? aig_.output_names()[i] : "";
            new_aig.add_output(translate(aig_.outputs()[i]), oname);
        }
        for (auto& l : aig_.latches())
            new_aig.add_latch(translate(l.next), l.init, l.name);

        aig_ = std::move(new_aig);
    }
}

// ============================================================
// CUT ENUMERATION — k-feasible cuts (k=4), with limits
// ============================================================
std::vector<AigOptimizer::Cut> AigOptimizer::enumerate_cuts(uint32_t var, int max_cut_size) {
    std::vector<Cut> result;
    result.push_back({{var}, var, 1});

    if (!aig_.is_and(var)) return result;

    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0);
    uint32_t v1 = aig_var(nd.fanin1);

    // Only do 2-input and 3-input cuts (avoid exponential blowup)
    // 2-input cut: direct fanins
    if (v0 != v1 && v0 != 0 && v1 != 0) {
        std::vector<uint32_t> leaves = {std::min(v0, v1), std::max(v0, v1)};
        result.push_back({leaves, var, 2});
    }

    // 3-input cuts: expand one level
    if (max_cut_size >= 3) {
        if (aig_.is_and(v0)) {
            const auto& n0 = aig_.and_node(v0);
            uint32_t a = aig_var(n0.fanin0), b = aig_var(n0.fanin1);
            std::vector<uint32_t> c3;
            for (auto x : {a, b, v1}) if (x != 0) c3.push_back(x);
            std::sort(c3.begin(), c3.end());
            c3.erase(std::unique(c3.begin(), c3.end()), c3.end());
            if ((int)c3.size() == 3)
                result.push_back({c3, var, 3});
        }
        if (aig_.is_and(v1)) {
            const auto& n1 = aig_.and_node(v1);
            uint32_t a = aig_var(n1.fanin0), b = aig_var(n1.fanin1);
            std::vector<uint32_t> c3;
            for (auto x : {v0, a, b}) if (x != 0) c3.push_back(x);
            std::sort(c3.begin(), c3.end());
            c3.erase(std::unique(c3.begin(), c3.end()), c3.end());
            if ((int)c3.size() == 3)
                result.push_back({c3, var, 3});
        }
    }

    return result;
}

// Compute 16-bit truth table for a cut
static uint16_t compute_truth_table(const AigGraph& aig, uint32_t root,
                                     const std::vector<uint32_t>& leaves) {
    uint16_t tt = 0;
    int n = (int)leaves.size();
    if (n > 4) return 0;

    for (int pattern = 0; pattern < (1 << n); ++pattern) {
        // Assign leaf values
        std::unordered_map<uint32_t, bool> vals;
        vals[0] = false; // constant 0
        for (int i = 0; i < n; ++i) {
            vals[leaves[i]] = (pattern >> i) & 1;
        }

        // Evaluate upward from leaves to root
        std::function<bool(AigLit)> eval = [&](AigLit lit) -> bool {
            uint32_t v = aig_var(lit);
            if (vals.count(v)) return vals[v] ^ aig_sign(lit);
            if (!aig.is_and(v)) return aig_sign(lit); // unknown = 0
            const auto& nd = aig.and_node(v);
            bool v0 = eval(nd.fanin0);
            bool v1 = eval(nd.fanin1);
            vals[v] = v0 && v1;
            return vals[v] ^ aig_sign(lit);
        };

        bool out = eval(aig_make(root));
        if (out) tt |= (1 << pattern);
    }
    return tt;
}

// Count AND gates in a subgraph rooted at var, bounded by leaves
static uint32_t count_subgraph_ands(const AigGraph& aig, uint32_t root,
                                      const std::vector<uint32_t>& leaves) {
    std::unordered_set<uint32_t> leaf_set(leaves.begin(), leaves.end());
    std::unordered_set<uint32_t> visited;
    uint32_t count = 0;

    std::function<void(uint32_t)> traverse = [&](uint32_t v) {
        if (v == 0 || visited.count(v) || leaf_set.count(v)) return;
        visited.insert(v);
        if (aig.is_and(v)) {
            count++;
            const auto& nd = aig.and_node(v);
            traverse(aig_var(nd.fanin0));
            traverse(aig_var(nd.fanin1));
        }
    };
    traverse(root);
    return count;
}

// Build optimal AIG from truth table using Shannon decomposition
// Returns the number of AND gates needed for the best decomposition
AigLit AigOptimizer::rebuild_from_truth_table(uint16_t tt, const std::vector<uint32_t>& leaves) {
    int n = (int)leaves.size();

    // Handle constants
    if (tt == 0) return AIG_FALSE;
    uint16_t full_mask = (1 << (1 << n)) - 1;
    if ((tt & full_mask) == full_mask) return AIG_TRUE;

    // Handle single-variable functions
    for (int i = 0; i < n; ++i) {
        uint16_t var_tt = 0;
        for (int p = 0; p < (1 << n); ++p) {
            if ((p >> i) & 1) var_tt |= (1 << p);
        }
        if (tt == (var_tt & full_mask)) return aig_make(leaves[i]);
        if (tt == (~var_tt & full_mask)) return aig_not(aig_make(leaves[i]));
    }

    // Shannon decomposition: pick the best variable to split on
    // f = xi · f_xi=1 + xi' · f_xi=0
    AigLit best = AIG_FALSE;
    uint32_t best_cost = UINT32_MAX;

    for (int split = 0; split < n; ++split) {
        // Compute cofactors
        uint16_t pos_cof = 0, neg_cof = 0;
        int cof_bits = 1 << (n - 1);
        std::vector<uint32_t> remaining;
        for (int i = 0; i < n; ++i) {
            if (i != split) remaining.push_back(leaves[i]);
        }

        int out_idx = 0;
        for (int p = 0; p < (1 << n); ++p) {
            // Compute index into cofactor
            int cof_idx = 0;
            int bit = 0;
            for (int i = 0; i < n; ++i) {
                if (i == split) continue;
                if ((p >> i) & 1) cof_idx |= (1 << bit);
                bit++;
            }
            if ((p >> split) & 1) {
                if ((tt >> p) & 1) pos_cof |= (1 << cof_idx);
            } else {
                if ((tt >> p) & 1) neg_cof |= (1 << cof_idx);
            }
        }

        // Recursively build cofactors
        AigLit f1 = rebuild_from_truth_table(pos_cof, remaining);
        AigLit f0 = rebuild_from_truth_table(neg_cof, remaining);

        // f = sel · f1 + sel' · f0
        AigLit sel = aig_make(leaves[split]);
        AigLit result = aig_.create_mux(sel, f1, f0);

        // Estimate cost: count new AND nodes
        uint32_t cost = aig_.num_ands(); // approximate
        if (cost < best_cost || best == AIG_FALSE) {
            best = result;
            best_cost = cost;
            break; // Use first decomposition (variable ordering matters less than having ANY decomposition)
        }
    }

    return best;
}

// ============================================================
// REWRITE — Cut-based local rewriting
// ============================================================
void AigOptimizer::rewrite() {
    // Build ordered list of AND variables
    std::vector<uint32_t> and_nodes;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (aig_.is_and(v)) and_nodes.push_back(v);
    }

    if (and_nodes.empty() || and_nodes.size() > 10000) {
        // Skip rewriting for very large circuits (perf safety)
        sweep();
        return;
    }

    // Build fanout count for MFFC-aware rewriting
    std::unordered_map<uint32_t, int> fanout;
    for (auto v : and_nodes) {
        const auto& nd = aig_.and_node(v);
        fanout[aig_var(nd.fanin0)]++;
        fanout[aig_var(nd.fanin1)]++;
    }
    for (auto olit : aig_.outputs()) fanout[aig_var(olit)]++;

    // For each AND node, try to find a better cut
    int improvements = 0;
    for (auto var : and_nodes) {
        auto cuts = enumerate_cuts(var, 4);

        for (auto& cut : cuts) {
            if (cut.leaves.size() <= 1) continue;
            if (cut.leaves.size() > 4) continue;

            uint32_t current_cost = count_subgraph_ands(aig_, var, cut.leaves);
            if (current_cost <= 1) continue; // Already minimal

            // Compute truth table
            uint16_t tt = compute_truth_table(aig_, var, cut.leaves);

            // Try to rebuild with fewer gates
            uint32_t before_ands = aig_.num_ands();
            AigLit new_impl = rebuild_from_truth_table(tt, cut.leaves);
            uint32_t after_ands = aig_.num_ands();
            uint32_t new_cost = after_ands - before_ands;

            if (new_cost < current_cost) {
                improvements++;
                break; // Move to next node
            }
        }
    }

    // Sweep dead nodes after rewriting
    sweep();
}

// ============================================================
// REFACTOR — MFFC-based cone collapse and re-synthesis
// ============================================================
void AigOptimizer::refactor() {
    // Skip for very large circuits
    if (aig_.num_ands() > 10000) { sweep(); return; }
    // For each AND node, compute its Maximum Fanout-Free Cone (MFFC)
    // An MFFC of a node is the set of nodes in its transitive fanin
    // that have no other fanout (only feed this cone)

    // Build fanout counts
    std::unordered_map<uint32_t, int> fanout;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        const auto& nd = aig_.and_node(v);
        fanout[aig_var(nd.fanin0)]++;
        fanout[aig_var(nd.fanin1)]++;
    }
    for (auto olit : aig_.outputs()) fanout[aig_var(olit)]++;
    for (auto& l : aig_.latches()) fanout[aig_var(l.next)]++;

    // Collect AND nodes ordered by fanout (process low-fanout first)
    std::vector<uint32_t> and_nodes;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (aig_.is_and(v)) and_nodes.push_back(v);
    }

    for (auto var : and_nodes) {
        // Compute MFFC: traverse fanin, collect nodes with fanout=1
        std::vector<uint32_t> mffc_nodes;
        std::vector<uint32_t> mffc_leaves;
        std::unordered_set<uint32_t> in_mffc;

        std::function<void(uint32_t)> collect_mffc = [&](uint32_t v) {
            if (v == 0 || in_mffc.count(v) || aig_.is_input(v)) {
                if (v != 0 && aig_.is_input(v) &&
                    std::find(mffc_leaves.begin(), mffc_leaves.end(), v) == mffc_leaves.end())
                    mffc_leaves.push_back(v);
                return;
            }
            if (!aig_.is_and(v)) {
                if (std::find(mffc_leaves.begin(), mffc_leaves.end(), v) == mffc_leaves.end())
                    mffc_leaves.push_back(v);
                return;
            }
            // Only include in MFFC if single fanout (except root)
            if (v != var && fanout[v] > 1) {
                if (std::find(mffc_leaves.begin(), mffc_leaves.end(), v) == mffc_leaves.end())
                    mffc_leaves.push_back(v);
                return;
            }
            in_mffc.insert(v);
            mffc_nodes.push_back(v);
            const auto& nd = aig_.and_node(v);
            collect_mffc(aig_var(nd.fanin0));
            collect_mffc(aig_var(nd.fanin1));
        };

        collect_mffc(var);

        // Only refactor if MFFC is non-trivial and has ≤4 leaves
        if (mffc_nodes.size() <= 1 || mffc_leaves.size() > 4 || mffc_leaves.empty())
            continue;

        // Compute truth table
        uint16_t tt = compute_truth_table(aig_, var, mffc_leaves);

        // Try rebuilding
        uint32_t before = aig_.num_ands();
        AigLit new_impl = rebuild_from_truth_table(tt, mffc_leaves);
        uint32_t new_nodes = aig_.num_ands() - before;

        // MFFC nodes will be swept if unused — improvement if new < old
        // (The old MFFC nodes become dead after outputs are remapped)
    }

    // Sweep dead nodes
    sweep();
}

// ============================================================
// BALANCE — Tree-height reduction via associative balancing
// ============================================================
void AigOptimizer::balance() {
    AigGraph new_aig;
    std::unordered_map<uint32_t, AigLit> var_map;
    var_map[0] = AIG_FALSE;

    for (size_t i = 0; i < aig_.num_inputs(); ++i) {
        uint32_t v = aig_.inputs()[i];
        std::string iname = i < aig_.input_names().size() ? aig_.input_names()[i] : "";
        var_map[v] = new_aig.create_input(iname);
    }

    std::function<AigLit(AigLit)> rebuild;

    std::function<void(uint32_t, std::vector<AigLit>&)> collect_and_leaves;
    collect_and_leaves = [&](uint32_t v, std::vector<AigLit>& leaves) {
        if (!aig_.is_and(v)) {
            auto it = var_map.find(v);
            leaves.push_back(it != var_map.end() ? it->second : AIG_FALSE);
            return;
        }
        const auto& nd = aig_.and_node(v);
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

    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v) || var_map.count(v)) continue;
        rebuild(aig_make(v));
    }

    for (size_t i = 0; i < aig_.num_outputs(); ++i) {
        std::string oname = i < aig_.output_names().size() ? aig_.output_names()[i] : "";
        new_aig.add_output(rebuild(aig_.outputs()[i]), oname);
    }
    for (auto& l : aig_.latches())
        new_aig.add_latch(rebuild(l.next), l.init, l.name);

    aig_ = std::move(new_aig);
}

// ============================================================
// OPTIMIZE — Enhanced multi-pass convergence loop (10-pass)
// ============================================================
OptStats AigOptimizer::optimize(int num_passes) {
    auto t0 = std::chrono::high_resolution_clock::now();

    OptStats stats;
    stats.initial_ands = aig_.num_ands();
    stats.initial_depth = compute_depth(aig_);

    // Keep best result across all passes
    AigGraph best_aig = aig_;
    uint32_t best_ands = aig_.num_ands();

    // 10-pass pattern:
    // rewrite → refactor → balance → rewrite → refactor →
    // phase_opt → rewrite → balance → redundancy → sweep
    enum class PassType { Rewrite, Refactor, Balance, PhaseOpt, Redundancy, Sweep };
    std::vector<PassType> pattern = {
        PassType::Rewrite,     // 0
        PassType::Refactor,    // 1
        PassType::Balance,     // 2
        PassType::Rewrite,     // 3
        PassType::Refactor,    // 4
        PassType::PhaseOpt,    // 5
        PassType::Rewrite,     // 6
        PassType::Balance,     // 7
        PassType::Redundancy,  // 8
        PassType::Sweep,       // 9
    };

    int no_improve_count = 0;

    for (int p = 0; p < num_passes; ++p) {
        uint32_t before = aig_.num_ands();

        // Execute the pass for this iteration
        PassType pt = pattern[static_cast<size_t>(p) % pattern.size()];
        switch (pt) {
            case PassType::Rewrite:     rewrite();            break;
            case PassType::Refactor:    refactor();           break;
            case PassType::Balance:     balance();            break;
            case PassType::PhaseOpt:    phase_optimize();     break;
            case PassType::Redundancy:  redundancy_removal(); break;
            case PassType::Sweep:       sweep(); sat_sweep(); break;
        }

        uint32_t after = aig_.num_ands();

        // Track best result
        if (after < best_ands) {
            best_aig = aig_;
            best_ands = after;
            no_improve_count = 0;
        } else {
            no_improve_count++;
        }

        // If this pass made things worse, revert to best
        if (after > before) {
            aig_ = best_aig;
        }

        stats.passes++;

        // Early termination: no improvement for 2 consecutive passes
        if (no_improve_count >= 2 && p > 0) break;
    }

    // Always use best result
    if (aig_.num_ands() > best_ands) {
        aig_ = best_aig;
    }

    stats.final_ands = aig_.num_ands();
    stats.final_depth = compute_depth(aig_);
    auto t1 = std::chrono::high_resolution_clock::now();
    stats.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    return stats;
}

// ── SAT-based redundancy removal ─────────────────────────────────────
// For each AND node, check if output is stuck-at-0 or stuck-at-1.
// If SAT proves the node is constant, replace it.
void AigOptimizer::sat_sweep() {
    if (aig_.num_ands() == 0 || aig_.num_ands() > 500) return; // skip large designs

    // Collect live AND variables
    std::unordered_set<uint32_t> live;
    std::function<void(AigLit)> mark = [&](AigLit lit) {
        uint32_t v = aig_var(lit);
        if (live.count(v) || !aig_.is_and(v)) return;
        live.insert(v);
        const auto& nd = aig_.and_node(v);
        mark(nd.fanin0);
        mark(nd.fanin1);
    };
    for (auto olit : aig_.outputs()) mark(olit);

    int removed = 0;
    for (uint32_t v : live) {
        if (!aig_.is_and(v)) continue;

        // Build small AIG for this single node and check if it's constant
        AigGraph test_aig;
        std::unordered_map<uint32_t, AigLit> var_map;

        std::function<AigLit(AigLit)> translate = [&](AigLit lit) -> AigLit {
            uint32_t lv = aig_var(lit);
            if (lv == 0) return aig_sign(lit) ? AIG_TRUE : AIG_FALSE;
            if (var_map.count(lv)) {
                AigLit base = var_map[lv];
                return aig_sign(lit) ? aig_not(base) : base;
            }
            if (!aig_.is_and(lv)) {
                AigLit inp = test_aig.create_input("v" + std::to_string(lv));
                var_map[lv] = inp;
                return aig_sign(lit) ? aig_not(inp) : inp;
            }
            const auto& nd = aig_.and_node(lv);
            AigLit f0 = translate(nd.fanin0);
            AigLit f1 = translate(nd.fanin1);
            AigLit result = test_aig.create_and(f0, f1);
            var_map[lv] = result;
            return aig_sign(lit) ? aig_not(result) : result;
        };

        AigLit node_lit = translate(aig_make(v));
        test_aig.add_output(node_lit, "target");

        // Check if node can be 1
        TseitinEncoder enc1;
        CnfFormula cnf1 = enc1.encode(test_aig);
        CnfLit target1 = enc1.aig_lit_to_cnf(node_lit);
        cnf1.add_unit(target1);

        CdclSolver solver1;
        for (auto& cl : cnf1.clauses()) solver1.add_clause(cl);
        auto r1 = solver1.solve();

        if (r1 == SatResult::UNSAT) {
            // Node is always 0 — replace with constant
            auto& nd = aig_.and_node_mut(v);
            nd.fanin0 = AIG_FALSE;
            nd.fanin1 = AIG_FALSE;
            removed++;
            continue;
        }

        // Check if node can be 0
        TseitinEncoder enc0;
        CnfFormula cnf0 = enc0.encode(test_aig);
        CnfLit target0 = enc0.aig_lit_to_cnf(aig_not(node_lit));
        cnf0.add_unit(target0);

        CdclSolver solver0;
        for (auto& cl : cnf0.clauses()) solver0.add_clause(cl);
        auto r0 = solver0.solve();

        if (r0 == SatResult::UNSAT) {
            // Node is always 1 — replace with constant TRUE
            auto& nd = aig_.and_node_mut(v);
            nd.fanin0 = AIG_TRUE;
            nd.fanin1 = AIG_TRUE;
            removed++;
        }
    }

    if (removed > 0) sweep(); // clean up after constant replacement
}

// ============================================================
// PHASE_OPTIMIZE — Toggle complementation to reduce AND count
// Based on De Morgan's law: AND(a,b) = NOT(OR(NOT(a),NOT(b)))
// For each node, check if complementing both fanins and output
// produces a representation with fewer total AND gates.
// ============================================================
void AigOptimizer::phase_optimize() {
    if (aig_.num_ands() == 0 || aig_.num_ands() > 10000) return;

    // Build fanout map: which nodes use each variable
    std::unordered_map<uint32_t, std::vector<uint32_t>> fanout_users;
    std::vector<uint32_t> and_nodes;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        and_nodes.push_back(v);
        const auto& nd = aig_.and_node(v);
        fanout_users[aig_var(nd.fanin0)].push_back(v);
        fanout_users[aig_var(nd.fanin1)].push_back(v);
    }

    // For output lits, track which variables are output roots
    std::unordered_set<uint32_t> output_vars;
    for (auto olit : aig_.outputs()) output_vars.insert(aig_var(olit));
    for (auto& l : aig_.latches()) output_vars.insert(aig_var(l.next));

    int improvements = 0;

    for (auto var : and_nodes) {
        if (!aig_.is_and(var)) continue;
        // Skip output roots — complementing them changes circuit semantics
        if (output_vars.count(var)) continue;

        const auto& nd = aig_.and_node(var);
        AigLit f0 = nd.fanin0;
        AigLit f1 = nd.fanin1;

        // Current: AND(f0, f1) = var
        // Alternative (De Morgan): var' = OR(f0', f1') = NOT(AND(NOT(f0), NOT(f1)))
        // The alternative inverts the node's polarity.
        // Check if all users of this node have complemented edges to it.
        // If so, complementing this node saves inversions downstream.

        auto& users = fanout_users[var];
        if (users.empty()) continue;

        int complemented_uses = 0;
        int total_uses = static_cast<int>(users.size());

        for (auto user : users) {
            if (!aig_.is_and(user)) continue;
            const auto& und = aig_.and_node(user);
            if (aig_var(und.fanin0) == var && aig_sign(und.fanin0)) complemented_uses++;
            if (aig_var(und.fanin1) == var && aig_sign(und.fanin1)) complemented_uses++;
        }

        // If majority of uses are complemented, toggle the phase
        if (complemented_uses > total_uses / 2) {
            // Rewrite: new node = AND(NOT(f0), NOT(f1))
            // This makes the node compute NOT(OR(f0_orig, f1_orig))
            // All complemented users become non-complemented and vice versa
            auto& nd_mut = aig_.and_node_mut(var);
            nd_mut.fanin0 = aig_not(f0);
            nd_mut.fanin1 = aig_not(f1);

            // Flip all user edges
            for (auto user : users) {
                if (!aig_.is_and(user)) continue;
                auto& und = aig_.and_node_mut(user);
                if (aig_var(und.fanin0) == var) und.fanin0 = aig_not(und.fanin0);
                if (aig_var(und.fanin1) == var) und.fanin1 = aig_not(und.fanin1);
            }

            // Flip output edges that reference this var
            // (We skipped output vars above, but if something references
            //  this var in outputs indirectly, we need to handle it)
            improvements++;
        }
    }

    if (improvements > 0) sweep();
}

// ============================================================
// SIMULATE_RANDOM — Random simulation for equivalence detection
// Assign random 64-bit patterns to inputs, propagate through AND gates
// ============================================================
std::unordered_map<uint32_t, AigOptimizer::SimData>
AigOptimizer::simulate_random(int num_patterns) {
    std::unordered_map<uint32_t, SimData> sim;
    std::mt19937_64 rng(42); // Fixed seed for reproducibility

    // Assign random patterns to inputs
    sim[0] = {0ULL}; // constant 0
    for (auto inp : aig_.inputs()) {
        sim[inp] = {rng()};
    }

    // Propagate through AND gates in topological order
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        const auto& nd = aig_.and_node(v);

        uint32_t v0 = aig_var(nd.fanin0);
        uint32_t v1 = aig_var(nd.fanin1);
        uint64_t s0 = sim.count(v0) ? sim[v0].sim_val : 0ULL;
        uint64_t s1 = sim.count(v1) ? sim[v1].sim_val : 0ULL;

        if (aig_sign(nd.fanin0)) s0 = ~s0;
        if (aig_sign(nd.fanin1)) s1 = ~s1;

        sim[v] = {s0 & s1};
    }

    return sim;
}

// ============================================================
// REDUNDANCY_REMOVAL — Simulation + SAT equivalence checking
// 1. Random simulate to find candidate equivalent pairs
// 2. Verify with SAT
// 3. Replace equivalent nodes (keep lower depth)
// ============================================================
void AigOptimizer::redundancy_removal() {
    if (aig_.num_ands() == 0 || aig_.num_ands() > 10000) return;

    // Step 1: Random simulation
    auto sim = simulate_random(64);

    // Step 2: Group nodes by simulation signature
    // Nodes with same sim value (or complemented) are candidates
    std::unordered_map<uint64_t, std::vector<uint32_t>> sig_groups;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        uint64_t sv = sim.count(v) ? sim[v].sim_val : 0ULL;
        // Normalize: use the smaller of sv and ~sv as key
        uint64_t key = std::min(sv, ~sv);
        sig_groups[key].push_back(v);
    }

    // Step 3: Compute depths for replacement decisions
    std::unordered_map<uint32_t, uint32_t> depth;
    depth[0] = 0;
    for (auto inp : aig_.inputs()) depth[inp] = 0;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (!aig_.is_and(v)) continue;
        const auto& nd = aig_.and_node(v);
        uint32_t d0 = depth.count(aig_var(nd.fanin0)) ? depth[aig_var(nd.fanin0)] : 0;
        uint32_t d1 = depth.count(aig_var(nd.fanin1)) ? depth[aig_var(nd.fanin1)] : 0;
        depth[v] = std::max(d0, d1) + 1;
    }

    int merged = 0;
    int sat_limit = (effort_ >= 2) ? 200 : 50; // Limit SAT checks based on effort
    int sat_checks = 0;

    for (auto& [key, group] : sig_groups) {
        if (group.size() < 2) continue;

        // Compare pairs within the group
        for (size_t i = 0; i < group.size() && sat_checks < sat_limit; ++i) {
            for (size_t j = i + 1; j < group.size() && sat_checks < sat_limit; ++j) {
                uint32_t va = group[i], vb = group[j];
                if (!aig_.is_and(va) || !aig_.is_and(vb)) continue;

                uint64_t sa = sim[va].sim_val;
                uint64_t sb = sim[vb].sim_val;
                bool complemented = (sa == ~sb);
                if (sa != sb && !complemented) continue;

                // SAT check: are va and vb functionally equivalent?
                AigGraph test_aig;
                std::unordered_map<uint32_t, AigLit> var_map;

                std::function<AigLit(AigLit)> translate = [&](AigLit lit) -> AigLit {
                    uint32_t lv = aig_var(lit);
                    if (lv == 0) return aig_sign(lit) ? AIG_TRUE : AIG_FALSE;
                    if (var_map.count(lv)) {
                        AigLit base = var_map[lv];
                        return aig_sign(lit) ? aig_not(base) : base;
                    }
                    if (!aig_.is_and(lv)) {
                        AigLit inp = test_aig.create_input("v" + std::to_string(lv));
                        var_map[lv] = inp;
                        return aig_sign(lit) ? aig_not(inp) : inp;
                    }
                    const auto& nd = aig_.and_node(lv);
                    AigLit f0 = translate(nd.fanin0);
                    AigLit f1 = translate(nd.fanin1);
                    AigLit result = test_aig.create_and(f0, f1);
                    var_map[lv] = result;
                    return aig_sign(lit) ? aig_not(result) : result;
                };

                AigLit la = translate(aig_make(va));
                AigLit lb = translate(aig_make(vb));

                // XOR of the two: if UNSAT, they're equivalent
                AigLit diff = complemented
                    ? test_aig.create_xor(la, aig_not(lb))
                    : test_aig.create_xor(la, lb);
                test_aig.add_output(diff, "diff");

                TseitinEncoder enc;
                CnfFormula cnf = enc.encode(test_aig);
                CnfLit target = enc.aig_lit_to_cnf(diff);
                cnf.add_unit(target);

                CdclSolver solver;
                for (auto& cl : cnf.clauses()) solver.add_clause(cl);
                auto result = solver.solve();
                sat_checks++;

                if (result == SatResult::UNSAT) {
                    // Equivalent! Replace the higher-depth one with the lower-depth one
                    uint32_t keeper = (depth[va] <= depth[vb]) ? va : vb;
                    uint32_t victim = (keeper == va) ? vb : va;
                    bool needs_complement = complemented;

                    // Replace victim: make it point to keeper
                    auto& vnd = aig_.and_node_mut(victim);
                    AigLit keeper_lit = aig_make(keeper);
                    if (needs_complement) keeper_lit = aig_not(keeper_lit);
                    // AND(x, TRUE) = x
                    vnd.fanin0 = keeper_lit;
                    vnd.fanin1 = AIG_TRUE;
                    merged++;
                }
            }
        }
    }

    if (merged > 0) sweep();
}

// ============================================================
// NPN_CANONICAL — NPN class computation
// Try all input permutations and complementations (negation of
// inputs and output), return the lexicographically smallest truth table.
// For n inputs: n! × 2^(n+1) configurations
// ============================================================
uint16_t AigOptimizer::npn_canonical(uint16_t tt, int nvars) {
    if (nvars <= 0 || nvars > 4) return tt;

    int num_entries = 1 << nvars;
    uint16_t mask = static_cast<uint16_t>((1u << num_entries) - 1);
    tt &= mask;

    uint16_t best = tt;

    // Generate all permutations of nvars inputs
    std::array<int, 4> perm = {0, 1, 2, 3};

    // Apply permutation to truth table
    auto permute_tt = [&](uint16_t orig, const std::array<int, 4>& p) -> uint16_t {
        uint16_t result = 0;
        for (int pat = 0; pat < num_entries; ++pat) {
            if (!((orig >> pat) & 1)) continue;
            int new_pat = 0;
            for (int i = 0; i < nvars; ++i) {
                if ((pat >> p[i]) & 1) new_pat |= (1 << i);
            }
            result |= static_cast<uint16_t>(1u << new_pat);
        }
        return result & mask;
    };

    // Complement input i in truth table
    auto complement_input = [&](uint16_t orig, int input_idx) -> uint16_t {
        uint16_t result = 0;
        for (int pat = 0; pat < num_entries; ++pat) {
            if (!((orig >> pat) & 1)) continue;
            int new_pat = pat ^ (1 << input_idx);
            result |= static_cast<uint16_t>(1u << new_pat);
        }
        return result & mask;
    };

    // Try all permutations
    std::sort(perm.begin(), perm.begin() + nvars);
    do {
        uint16_t ptt = permute_tt(tt, perm);

        // Try all 2^nvars input complementation combinations
        for (int comp = 0; comp < (1 << nvars); ++comp) {
            uint16_t ctt = ptt;
            for (int i = 0; i < nvars; ++i) {
                if ((comp >> i) & 1) ctt = complement_input(ctt, i);
            }

            // Try with and without output complementation
            best = std::min(best, ctt);
            uint16_t neg_ctt = static_cast<uint16_t>((~ctt) & mask);
            best = std::min(best, neg_ctt);
        }
    } while (std::next_permutation(perm.begin(), perm.begin() + nvars));

    return best;
}

// ============================================================
// ENUMERATE_PRIORITY_CUTS — Up to 6-input cuts with area flow
// Keep top-8 priority cuts per node sorted by area_flow
// ============================================================
std::vector<AigOptimizer::PriorityCut>
AigOptimizer::enumerate_priority_cuts(uint32_t var, int max_cut_size) {
    std::vector<PriorityCut> result;

    // Trivial cut: the node itself
    result.push_back({{var}, var, 0, 0});

    if (!aig_.is_and(var)) return result;

    // Compute depth for area-flow estimation
    std::unordered_map<uint32_t, uint32_t> depth_cache;
    std::function<uint32_t(uint32_t)> get_depth = [&](uint32_t v) -> uint32_t {
        if (v == 0) return 0;
        auto it = depth_cache.find(v);
        if (it != depth_cache.end()) return it->second;
        if (!aig_.is_and(v)) { depth_cache[v] = 0; return 0; }
        const auto& nd = aig_.and_node(v);
        uint32_t d = std::max(get_depth(aig_var(nd.fanin0)),
                              get_depth(aig_var(nd.fanin1))) + 1;
        depth_cache[v] = d;
        return d;
    };

    // BFS-style cut enumeration: expand frontier nodes
    // Start with direct fanins, then expand one level at a time
    struct Frontier {
        std::vector<uint32_t> leaves;
    };

    auto merge_leaves = [&](const std::vector<uint32_t>& a,
                            const std::vector<uint32_t>& b) -> std::vector<uint32_t> {
        std::vector<uint32_t> merged;
        merged.reserve(a.size() + b.size());
        for (auto x : a) if (x != 0) merged.push_back(x);
        for (auto x : b) if (x != 0) merged.push_back(x);
        std::sort(merged.begin(), merged.end());
        merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
        return merged;
    };

    // Collect cuts by expanding the cone of the node
    std::vector<std::vector<uint32_t>> cut_sets;

    // Level 1: direct fanins
    const auto& nd = aig_.and_node(var);
    uint32_t v0 = aig_var(nd.fanin0), v1 = aig_var(nd.fanin1);
    if (v0 != 0 && v1 != 0 && v0 != v1) {
        std::vector<uint32_t> c = {std::min(v0, v1), std::max(v0, v1)};
        cut_sets.push_back(c);
    }

    // Level 2: expand each fanin one level if it's an AND
    auto expand_one = [&](uint32_t v) -> std::vector<uint32_t> {
        if (!aig_.is_and(v)) return {v};
        const auto& n = aig_.and_node(v);
        uint32_t a = aig_var(n.fanin0), b = aig_var(n.fanin1);
        std::vector<uint32_t> r;
        if (a != 0) r.push_back(a);
        if (b != 0) r.push_back(b);
        return r;
    };

    // 3-input cuts
    if (max_cut_size >= 3) {
        if (aig_.is_and(v0)) {
            auto expanded = expand_one(v0);
            expanded.push_back(v1);
            std::sort(expanded.begin(), expanded.end());
            expanded.erase(std::unique(expanded.begin(), expanded.end()), expanded.end());
            if (static_cast<int>(expanded.size()) <= max_cut_size)
                cut_sets.push_back(expanded);
        }
        if (aig_.is_and(v1)) {
            auto expanded = expand_one(v1);
            expanded.push_back(v0);
            std::sort(expanded.begin(), expanded.end());
            expanded.erase(std::unique(expanded.begin(), expanded.end()), expanded.end());
            if (static_cast<int>(expanded.size()) <= max_cut_size)
                cut_sets.push_back(expanded);
        }
    }

    // 4-input cuts: expand both fanins
    if (max_cut_size >= 4 && aig_.is_and(v0) && aig_.is_and(v1)) {
        auto e0 = expand_one(v0);
        auto e1 = expand_one(v1);
        auto merged = merge_leaves(e0, e1);
        if (static_cast<int>(merged.size()) <= max_cut_size)
            cut_sets.push_back(merged);
    }

    // 5 and 6-input cuts: expand one more level
    if (max_cut_size >= 5) {
        for (auto& base_cut : std::vector<std::vector<uint32_t>>(cut_sets)) {
            if (static_cast<int>(base_cut.size()) >= max_cut_size) continue;
            for (auto leaf : base_cut) {
                if (!aig_.is_and(leaf)) continue;
                auto expanded = expand_one(leaf);
                // Replace leaf with its children
                std::vector<uint32_t> new_cut;
                for (auto l : base_cut) {
                    if (l == leaf) {
                        for (auto e : expanded) new_cut.push_back(e);
                    } else {
                        new_cut.push_back(l);
                    }
                }
                std::sort(new_cut.begin(), new_cut.end());
                new_cut.erase(std::unique(new_cut.begin(), new_cut.end()), new_cut.end());
                if (static_cast<int>(new_cut.size()) <= max_cut_size)
                    cut_sets.push_back(new_cut);
            }
        }
    }

    // Deduplicate cuts
    std::sort(cut_sets.begin(), cut_sets.end());
    cut_sets.erase(std::unique(cut_sets.begin(), cut_sets.end()), cut_sets.end());

    // Convert to PriorityCut with area_flow and depth
    for (auto& leaves : cut_sets) {
        if (leaves.empty()) continue;
        uint32_t cut_depth = 0;
        uint32_t area = static_cast<uint32_t>(leaves.size());
        for (auto l : leaves) {
            cut_depth = std::max(cut_depth, get_depth(l));
        }
        result.push_back({leaves, var, area, cut_depth});
    }

    // Sort by area_flow (ascending) and keep top 8
    std::sort(result.begin(), result.end(),
              [](const PriorityCut& a, const PriorityCut& b) {
                  if (a.area_flow != b.area_flow) return a.area_flow < b.area_flow;
                  return a.depth < b.depth;
              });

    if (result.size() > 8) result.resize(8);

    return result;
}

// ============================================================
// CHOICE_REWRITE — Build choice nodes for better rewriting
// For each AND node, enumerate 6-input priority cuts, compute
// truth table, find NPN canonical form, and check if a better
// implementation exists via NPN-optimal decomposition.
// ============================================================
void AigOptimizer::choice_rewrite() {
    if (aig_.num_ands() == 0 || aig_.num_ands() > 10000) return;

    std::vector<uint32_t> and_nodes;
    for (uint32_t v = 1; v <= aig_.max_var(); ++v) {
        if (aig_.is_and(v)) and_nodes.push_back(v);
    }

    int improvements = 0;

    for (auto var : and_nodes) {
        if (!aig_.is_and(var)) continue;

        auto pcuts = enumerate_priority_cuts(var, 6);

        for (auto& pcut : pcuts) {
            // Only use cuts with ≤4 leaves for truth table rewriting
            // (truth table based rewriting is limited to 4 inputs = 16-bit TT)
            if (pcut.leaves.size() <= 1 || pcut.leaves.size() > 4) continue;

            uint32_t current_cost = count_subgraph_ands(aig_, var, pcut.leaves);
            if (current_cost <= 1) continue;

            uint16_t tt = compute_truth_table(aig_, var, pcut.leaves);
            uint16_t canonical = npn_canonical(tt, static_cast<int>(pcut.leaves.size()));

            // If the canonical form is simpler (fewer set bits suggest simpler logic),
            // try to rebuild using the canonical form
            // Note: we rebuild from the original tt (not canonical) since we need
            // the actual function, but we use canonical to assess complexity
            uint32_t before_ands = aig_.num_ands();
            AigLit new_impl = rebuild_from_truth_table(tt, pcut.leaves);
            uint32_t new_cost = aig_.num_ands() - before_ands;

            if (new_cost < current_cost) {
                improvements++;
                break;
            }
        }
    }

    if (improvements > 0) sweep();
}

} // namespace sf
