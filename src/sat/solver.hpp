#pragma once
// SiliconForge — SAT Solver Abstract Interface

#include "cnf.hpp"
#include <vector>

namespace sf {

enum class SatResult { SAT, UNSAT, UNKNOWN };

class SatSolver {
public:
    virtual ~SatSolver() = default;
    virtual SatResult solve() = 0;
    virtual SatResult solve(const std::vector<CnfLit>& assumptions) = 0;
    virtual bool model_value(int var) const = 0;
    virtual void add_clause(const std::vector<CnfLit>& clause) = 0;
    virtual int new_var() = 0;
    virtual int num_vars() const = 0;
};

} // namespace sf
