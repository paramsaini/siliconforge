// SiliconForge — Bounded Model Checking Implementation
// Unrolls the sequential AIG k times and encodes as a SAT problem.
//
// Mathematical formulation:
//   φ = I(s₀) ∧ T(s₀,s₁) ∧ T(s₁,s₂) ∧ ... ∧ T(sₖ₋₁,sₖ) ∧ (Bad₀ ∨ Bad₁ ∨ ... ∨ Badₖ)
// If SAT → counterexample trace found (property violation)
// If UNSAT for all k up to max_depth → property holds (bounded proof)

#include "formal/bmc.hpp"
#include "formal/tseitin.hpp"
#include "sat/cdcl_solver.hpp"
#include <iostream>
#include <unordered_map>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace sf {

BmcResult BmcEngine::check(const AigGraph& aig, int max_depth) {
    if (aig.num_latches() == 0) {
        return {false, 0, {}, "ERROR: BMC requires sequential circuit (latches)"};
    }
    if (aig.num_outputs() == 0) {
        return {false, 0, {}, "ERROR: No property output defined"};
    }

    // The last output is the bad-state signal (1 = property violated)
    AigLit bad_output = aig.outputs().back();

    CnfFormula cnf;
    CdclSolver solver;

    // For each time frame, we create fresh CNF variables for all AIG variables
    // var_map[frame][aig_var] = cnf_var
    std::vector<std::unordered_map<uint32_t, int>> var_maps;

    auto get_var = [&](int frame, uint32_t av) -> int {
        while ((int)var_maps.size() <= frame)
            var_maps.emplace_back();
        auto it = var_maps[frame].find(av);
        if (it != var_maps[frame].end()) return it->second;
        int cv = solver.new_var();
        var_maps[frame][av] = cv;
        return cv;
    };

    auto get_lit = [&](int frame, AigLit alit) -> CnfLit {
        int cv = get_var(frame, aig_var(alit));
        return aig_sign(alit) ? -cv : cv;
    };

    // Encode one time frame's combinational logic
    auto encode_frame = [&](int frame) {
        // Constant FALSE
        int const_v = get_var(frame, 0);
        solver.add_clause({-const_v});

        // AND gates: Tseitin encoding
        for (uint32_t v = 1; v <= aig.max_var(); ++v) {
            if (!aig.is_and(v)) continue;
            const auto& node = aig.and_node(v);
            int c = get_var(frame, v);
            CnfLit a = get_lit(frame, node.fanin0);
            CnfLit b = get_lit(frame, node.fanin1);
            solver.add_clause({-c, a});
            solver.add_clause({-c, b});
            solver.add_clause({c, -a, -b});
        }
    };

    // Initial state: all latches get their init values
    auto encode_init = [&]() {
        for (size_t i = 0; i < aig.latches().size(); ++i) {
            int lv = get_var(0, aig.inputs().size() + 1 + i); // latch var estimate
            // Actually, latch variables aren't inputs — we need their actual var indices
        }
        // Better approach: set latch state variables at frame 0 to their init values
    };

    // We need to know latch variable indices. In the AIG, latches are created
    // with add_latch which assigns them variable indices. Let's track them.
    // The latch at index i has variable latch_vars_[i] (private).
    // We can infer: latch vars are allocated after inputs and AND gates.
    // Actually, we need to iterate and use a different approach.

    // Let's use a cleaner unrolling: for each frame, we have variables for
    // every AIG node. Latches connect frames: latch state at frame k+1 =
    // latch next-state function at frame k.

    // The AIG stores latches with their next-state function (combinational AIG literal).
    // We treat each latch as having a "current state" variable per frame.
    // At frame 0, current state = init value.
    // At frame k+1, current state = next-state function evaluated at frame k.

    // Since latch vars are created by add_latch and stored internally,
    // we know they're sequential after inputs. Let's compute them.
    // Input vars: 1, 2, ..., num_inputs
    // Latch vars come from add_latch which uses next_var_++
    // We can figure them out: they appear in the AIG but aren't inputs or AND gates.

    // Better: encode latches explicitly. The latch "current value" at frame f
    // is what we constrain.

    // For simplicity: latch variable indices are num_inputs + 1, num_inputs + 2, etc.
    // But this isn't guaranteed by the AIG API. Let's scan for them.

    // The AIG graph has latches with .next (AigLit) and .init (AigLit).
    // The latch variables are allocated by add_latch. We can find them by
    // looking at which variables are neither 0, inputs, nor AND gates.
    std::vector<uint32_t> latch_vars;
    for (uint32_t v = 1; v <= aig.max_var(); ++v) {
        if (!aig.is_input(v) && !aig.is_and(v))
            latch_vars.push_back(v);
    }

    // Collect bad-state literals across all frames
    std::vector<CnfLit> bad_lits;

    for (int frame = 0; frame <= max_depth; ++frame) {
        // Encode combinational logic for this frame
        encode_frame(frame);

        // Set latch values for this frame
        if (frame == 0) {
            // Initial state
            for (size_t i = 0; i < latch_vars.size() && i < aig.latches().size(); ++i) {
                int cv = get_var(0, latch_vars[i]);
                if (aig.latches()[i].init == AIG_FALSE)
                    solver.add_clause({-cv});
                else
                    solver.add_clause({cv});
            }
        } else {
            // Transition: latch[frame] = next_state_function[frame-1]
            for (size_t i = 0; i < latch_vars.size() && i < aig.latches().size(); ++i) {
                int curr = get_var(frame, latch_vars[i]);
                CnfLit next_val = get_lit(frame - 1, aig.latches()[i].next);
                // curr ↔ next_val: (curr → next_val) ∧ (next_val → curr)
                solver.add_clause({-curr, next_val});
                solver.add_clause({curr, -next_val});
            }
        }

        // Collect bad-state literal for this frame
        CnfLit bad_lit = get_lit(frame, bad_output);
        bad_lits.push_back(bad_lit);

        // Check: is the property violated at any frame 0..frame?
        // Add activation literal for incremental solving
        int act = solver.new_var();
        // act → (bad_0 ∨ bad_1 ∨ ... ∨ bad_frame)
        std::vector<CnfLit> clause = {-act};
        for (auto bl : bad_lits) clause.push_back(bl);
        solver.add_clause(clause);

        SatResult result = solver.solve({act});

        if (result == SatResult::SAT) {
            // Counterexample found!
            BmcResult br;
            br.safe = false;
            br.depth = frame;
            br.message = "UNSAFE — property violated at cycle " + std::to_string(frame);

            // Extract trace
            for (int f = 0; f <= frame; ++f) {
                std::vector<bool> input_vals(aig.num_inputs());
                for (size_t i = 0; i < aig.num_inputs(); ++i) {
                    int cv = get_var(f, aig.inputs()[i]);
                    input_vals[i] = solver.model_value(cv);
                }
                br.trace.push_back(input_vals);
            }
            return br;
        }
    }

    return {true, max_depth, {}, "SAFE — property holds for " +
            std::to_string(max_depth + 1) + " cycles (bounded proof)"};
}

} // namespace sf
