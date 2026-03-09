#pragma once
// SiliconForge — k-Induction Prover
// Strengthens BMC with an inductive step to provide unbounded proofs.
//
// Base case (BMC):  I(s₀) ∧ T^k ∧ ¬P(sₖ) — is there a bad trace from init?
// Inductive step:   P(s₀)∧...∧P(sₖ₋₁) ∧ T^k ∧ ¬P(sₖ) — can k safe steps lead to unsafe?
//
// If base UNSAT AND step UNSAT → property holds UNCONDITIONALLY (unbounded proof)

#include "core/aig.hpp"
#include "sat/solver.hpp"
#include <string>
#include <vector>

namespace sf {

struct KIndResult {
    enum Status { PROVEN, FAILED, UNKNOWN };
    Status status = UNKNOWN;
    int depth = 0;
    std::vector<std::vector<bool>> trace; // counterexample if FAILED
    std::string message;
};

class KInduction {
public:
    // Run k-induction on sequential AIG.
    // Last output = bad-state signal (1 = property violation).
    static KIndResult check(const AigGraph& aig, int max_k = 50);
};

} // namespace sf
