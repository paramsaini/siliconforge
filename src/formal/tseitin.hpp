#pragma once
// SiliconForge — Tseitin Transformation (AIG → CNF)
// Each AND gate becomes 3 clauses. Complement edges handled via literal negation.
// Reference: G. Tseitin, "On the complexity of derivation in propositional calculus", 1968

#include "core/aig.hpp"
#include "sat/cnf.hpp"
#include <unordered_map>

namespace sf {

class TseitinEncoder {
public:
    // Encode an AIG into CNF. Returns the CNF formula.
    // cnf_var_map maps AIG variable -> CNF variable (1-indexed).
    CnfFormula encode(const AigGraph& aig);

    // Get the CNF variable for an AIG variable (after encoding)
    int aig_to_cnf(uint32_t aig_var) const;

    // Get the CNF literal for an AIG literal (handles complement)
    CnfLit aig_lit_to_cnf(AigLit lit) const;

private:
    std::unordered_map<uint32_t, int> var_map_;

    int get_or_create(uint32_t aig_var, CnfFormula& cnf);
    CnfLit lit_convert(AigLit aig_lit) const;
};

} // namespace sf
