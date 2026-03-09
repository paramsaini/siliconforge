#pragma once
// SiliconForge — Bounded Model Checking (BMC)
// Unrolls sequential circuit for k cycles and checks safety properties via SAT.
// Reference: A. Biere et al., "Symbolic Model Checking without BDDs", TACAS 1999

#include "core/aig.hpp"
#include "sat/solver.hpp"
#include <vector>
#include <string>

namespace sf {

struct BmcResult {
    bool safe = false;          // true if property holds up to depth k
    int depth = 0;              // depth checked (or depth of counterexample)
    std::vector<std::vector<bool>> trace; // state trace if counterexample found
    std::string message;
};

class BmcEngine {
public:
    // Run BMC on a sequential AIG circuit.
    // The property output (last output) should be TRUE when property is VIOLATED.
    // (Bad-state signal: 1 = property violation)
    // max_depth: maximum number of clock cycles to unroll
    static BmcResult check(const AigGraph& aig, int max_depth = 20);
};

} // namespace sf
