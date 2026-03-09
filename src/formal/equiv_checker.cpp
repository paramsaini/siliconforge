// SiliconForge — Equivalence Checker Implementation
#include "formal/equiv_checker.hpp"
#include "formal/tseitin.hpp"
#include "sat/cdcl_solver.hpp"
#include <iostream>

namespace sf {

EquivResult EquivChecker::check_outputs(const AigGraph& aig, AigLit out_a, AigLit out_b) {
    // Build miter: XOR of the two outputs. If SAT → not equivalent.
    // We encode the AIG into CNF, then add a clause forcing XOR(a,b) = TRUE.
    TseitinEncoder enc;
    CnfFormula cnf = enc.encode(aig);

    CnfLit ca = enc.aig_lit_to_cnf(out_a);
    CnfLit cb = enc.aig_lit_to_cnf(out_b);

    // XOR(a,b) = (a ∧ ¬b) ∨ (¬a ∧ b)
    // We create a new variable x for the miter output
    int x = cnf.new_var();
    // x ↔ XOR(ca, cb):
    //   x → (ca ∨ cb) and x → (¬ca ∨ ¬cb)  [x implies they differ]
    //   ¬x → (ca ↔ cb)
    // Simpler: just add clauses that force "ca ≠ cb"
    // Clause 1: (ca ∨ cb)  — at least one is true
    // Clause 2: (¬ca ∨ ¬cb) — at least one is false
    // But these together mean exactly one is true = XOR = 1
    // However, we want to check if XOR CAN be 1and is satisfiable.
    // So we add the constraint that ca and cb differ:
    cnf.add_clause({ca, cb});       // not both false
    cnf.add_clause({-ca, -cb});     // not both true

    CdclSolver solver(cnf);
    SatResult result = solver.solve();

    EquivResult er;
    if (result == SatResult::UNSAT) {
        er.equivalent = true;
        er.message = "EQUIVALENT — miter is UNSAT (mathematically proven)";
    } else {
        er.equivalent = false;
        er.message = "NOT EQUIVALENT — counterexample found";
        er.counterexample.resize(aig.num_inputs());
        for (size_t i = 0; i < aig.num_inputs(); ++i) {
            int cv = enc.aig_to_cnf(aig.inputs()[i]);
            er.counterexample[i] = solver.model_value(cv);
        }
    }
    return er;
}

EquivResult EquivChecker::check(const AigGraph& circuit_a, const AigGraph& circuit_b) {
    if (circuit_a.num_inputs() != circuit_b.num_inputs() ||
        circuit_a.num_outputs() != circuit_b.num_outputs()) {
        return {false, {}, "MISMATCH — different number of inputs/outputs"};
    }

    // Build combined AIG: shared inputs, both circuits, miter outputs
    AigGraph miter;
    size_t n_in = circuit_a.num_inputs();
    size_t n_out = circuit_a.num_outputs();

    // Create shared inputs
    std::vector<AigLit> shared_inputs(n_in);
    for (size_t i = 0; i < n_in; ++i)
        shared_inputs[i] = miter.create_input("in" + std::to_string(i));

    // Helper: deep-copy an AIG's logic into the miter graph
    auto copy_circuit = [&](const AigGraph& src) -> std::vector<AigLit> {
        // Map src AIG var -> miter AIG lit
        std::vector<AigLit> var_map(src.max_var() + 1, AIG_FALSE);

        // Map inputs
        for (size_t i = 0; i < src.num_inputs(); ++i)
            var_map[src.inputs()[i]] = shared_inputs[i];

        // Rebuild AND gates (they're in topological order)
        for (uint32_t v = 1; v <= src.max_var(); ++v) {
            if (!src.is_and(v)) continue;
            const auto& node = src.and_node(v);
            // Translate fanin literals
            auto translate = [&](AigLit lit) -> AigLit {
                AigLit mapped = var_map[aig_var(lit)];
                return aig_sign(lit) ? aig_not(mapped) : mapped;
            };
            var_map[v] = miter.create_and(translate(node.fanin0), translate(node.fanin1));
        }

        // Collect output literals
        std::vector<AigLit> outs(n_out);
        for (size_t i = 0; i < n_out; ++i) {
            AigLit olit = src.outputs()[i];
            AigLit mapped = var_map[aig_var(olit)];
            outs[i] = aig_sign(olit) ? aig_not(mapped) : mapped;
        }
        return outs;
    };

    auto outs_a = copy_circuit(circuit_a);
    auto outs_b = copy_circuit(circuit_b);

    // Build miter: OR of all XOR(out_a[i], out_b[i])
    AigLit miter_out = AIG_FALSE;
    for (size_t i = 0; i < n_out; ++i) {
        AigLit diff = miter.create_xor(outs_a[i], outs_b[i]);
        miter_out = miter.create_or(miter_out, diff);
    }
    miter.add_output(miter_out, "miter");

    // Encode and solve
    TseitinEncoder enc;
    CnfFormula cnf = enc.encode(miter);
    // Force miter output = TRUE (we want to find a distinguishing input)
    cnf.add_unit(enc.aig_lit_to_cnf(miter_out));

    CdclSolver solver(cnf);
    SatResult result = solver.solve();

    EquivResult er;
    if (result == SatResult::UNSAT) {
        er.equivalent = true;
        er.message = "EQUIVALENT — all " + std::to_string(n_out) +
                     " outputs proven identical for all inputs";
    } else {
        er.equivalent = false;
        er.message = "NOT EQUIVALENT — distinguishing input found";
        er.counterexample.resize(n_in);
        for (size_t i = 0; i < n_in; ++i) {
            int cv = enc.aig_to_cnf(miter.inputs()[i]);
            er.counterexample[i] = solver.model_value(cv);
        }
    }
    return er;
}

} // namespace sf
