// SiliconForge — k-Induction Implementation
#include "formal/k_induction.hpp"
#include "sat/cdcl_solver.hpp"
#include <unordered_map>
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace sf {

KIndResult KInduction::check(const AigGraph& aig, int max_k) {
    if (aig.num_latches() == 0 || aig.num_outputs() == 0)
        return {KIndResult::UNKNOWN, 0, {}, "ERROR: need latches and outputs"};

    AigLit bad_output = aig.outputs().back();

    // Find latch variables
    std::vector<uint32_t> latch_vars;
    for (uint32_t v = 1; v <= aig.max_var(); ++v)
        if (!aig.is_input(v) && !aig.is_and(v))
            latch_vars.push_back(v);

    for (int k = 0; k <= max_k; ++k) {
        // =============================================
        // BASE CASE: I(s₀) ∧ T(s₀,s₁) ∧ ... ∧ ¬P(sₖ)
        // Same as BMC — check if bad state reachable from init in k steps
        // =============================================
        {
            CdclSolver base_solver;
            std::vector<std::unordered_map<uint32_t, int>> vmaps;

            auto get_var = [&](int f, uint32_t av) -> int {
                while ((int)vmaps.size() <= f) vmaps.emplace_back();
                auto it = vmaps[f].find(av);
                if (it != vmaps[f].end()) return it->second;
                int cv = base_solver.new_var();
                vmaps[f][av] = cv;
                return cv;
            };
            auto get_lit = [&](int f, AigLit al) -> CnfLit {
                int cv = get_var(f, aig_var(al));
                return aig_sign(al) ? -cv : cv;
            };
            auto encode_frame = [&](int f) {
                base_solver.add_clause({-get_var(f, 0)});
                for (uint32_t v = 1; v <= aig.max_var(); ++v) {
                    if (!aig.is_and(v)) continue;
                    const auto& nd = aig.and_node(v);
                    int c = get_var(f, v);
                    CnfLit a = get_lit(f, nd.fanin0), b = get_lit(f, nd.fanin1);
                    base_solver.add_clause({-c, a});
                    base_solver.add_clause({-c, b});
                    base_solver.add_clause({c, -a, -b});
                }
            };

            for (int f = 0; f <= k; ++f) {
                encode_frame(f);
                if (f == 0) {
                    for (size_t i = 0; i < latch_vars.size() && i < aig.latches().size(); ++i) {
                        int cv = get_var(0, latch_vars[i]);
                        base_solver.add_clause(aig.latches()[i].init == AIG_FALSE ?
                            std::vector<CnfLit>{-cv} : std::vector<CnfLit>{cv});
                    }
                } else {
                    for (size_t i = 0; i < latch_vars.size() && i < aig.latches().size(); ++i) {
                        int curr = get_var(f, latch_vars[i]);
                        CnfLit nxt = get_lit(f - 1, aig.latches()[i].next);
                        base_solver.add_clause({-curr, nxt});
                        base_solver.add_clause({curr, -nxt});
                    }
                }
            }
            // Assert bad at frame k
            base_solver.add_clause({get_lit(k, bad_output)});

            if (base_solver.solve() == SatResult::SAT) {
                KIndResult r;
                r.status = KIndResult::FAILED;
                r.depth = k;
                r.message = "UNSAFE — counterexample at depth " + std::to_string(k);
                for (int f = 0; f <= k; ++f) {
                    std::vector<bool> iv(aig.num_inputs());
                    for (size_t i = 0; i < aig.num_inputs(); ++i)
                        iv[i] = base_solver.model_value(get_var(f, aig.inputs()[i]));
                    r.trace.push_back(iv);
                }
                return r;
            }
        }

        // =============================================
        // INDUCTIVE STEP: P(s₀)∧...∧P(sₖ₋₁) ∧ T^k ∧ ¬P(sₖ)
        // No initial state constraint — states are free
        // =============================================
        {
            CdclSolver step_solver;
            std::vector<std::unordered_map<uint32_t, int>> vmaps;

            auto get_var = [&](int f, uint32_t av) -> int {
                while ((int)vmaps.size() <= f) vmaps.emplace_back();
                auto it = vmaps[f].find(av);
                if (it != vmaps[f].end()) return it->second;
                int cv = step_solver.new_var();
                vmaps[f][av] = cv;
                return cv;
            };
            auto get_lit = [&](int f, AigLit al) -> CnfLit {
                int cv = get_var(f, aig_var(al));
                return aig_sign(al) ? -cv : cv;
            };
            auto encode_frame = [&](int f) {
                step_solver.add_clause({-get_var(f, 0)});
                for (uint32_t v = 1; v <= aig.max_var(); ++v) {
                    if (!aig.is_and(v)) continue;
                    const auto& nd = aig.and_node(v);
                    int c = get_var(f, v);
                    CnfLit a = get_lit(f, nd.fanin0), b = get_lit(f, nd.fanin1);
                    step_solver.add_clause({-c, a});
                    step_solver.add_clause({-c, b});
                    step_solver.add_clause({c, -a, -b});
                }
            };

            for (int f = 0; f <= k; ++f) {
                encode_frame(f);
                // Transition (no init constraint — free states)
                if (f > 0) {
                    for (size_t i = 0; i < latch_vars.size() && i < aig.latches().size(); ++i) {
                        int curr = get_var(f, latch_vars[i]);
                        CnfLit nxt = get_lit(f - 1, aig.latches()[i].next);
                        step_solver.add_clause({-curr, nxt});
                        step_solver.add_clause({curr, -nxt});
                    }
                }
                // P holds at frames 0..k-1
                if (f < k) {
                    step_solver.add_clause({-get_lit(f, bad_output)}); // NOT bad
                }
            }
            // Assert bad at frame k
            step_solver.add_clause({get_lit(k, bad_output)});

            if (step_solver.solve() == SatResult::UNSAT) {
                return {KIndResult::PROVEN, k, {},
                    "PROVEN SAFE — " + std::to_string(k) + "-induction (unbounded proof)"};
            }
        }
    }

    return {KIndResult::UNKNOWN, max_k, {},
        "UNKNOWN — neither proven nor disproven up to k=" + std::to_string(max_k)};
}

} // namespace sf
