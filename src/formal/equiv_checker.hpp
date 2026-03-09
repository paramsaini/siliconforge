#pragma once
// SiliconForge — Combinational Equivalence Checker
// Builds a miter circuit (XOR of outputs) and checks via SAT.
// UNSAT = circuits are equivalent. SAT = counterexample found.

#include "core/aig.hpp"
#include "sat/solver.hpp"
#include <vector>
#include <memory>

namespace sf {

struct EquivResult {
    bool equivalent = false;
    std::vector<bool> counterexample; // Input values that distinguish (if not equivalent)
    std::string message;
};

class EquivChecker {
public:
    // Check if two AIG circuits with same number of inputs/outputs are equivalent
    static EquivResult check(const AigGraph& circuit_a, const AigGraph& circuit_b);

    // Check if two output literals within the SAME AIG are equivalent
    static EquivResult check_outputs(const AigGraph& aig, AigLit out_a, AigLit out_b);
};

} // namespace sf
