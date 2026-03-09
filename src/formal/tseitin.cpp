// SiliconForge — Tseitin Transformation Implementation
#include "formal/tseitin.hpp"

namespace sf {

int TseitinEncoder::get_or_create(uint32_t aig_var, CnfFormula& cnf) {
    auto it = var_map_.find(aig_var);
    if (it != var_map_.end()) return it->second;
    int cv = cnf.new_var();
    var_map_[aig_var] = cv;
    return cv;
}

CnfLit TseitinEncoder::lit_convert(AigLit aig_lit) const {
    int cv = var_map_.at(aig_var(aig_lit));
    return aig_sign(aig_lit) ? -cv : cv;
}

int TseitinEncoder::aig_to_cnf(uint32_t av) const { return var_map_.at(av); }
CnfLit TseitinEncoder::aig_lit_to_cnf(AigLit lit) const { return lit_convert(lit); }

CnfFormula TseitinEncoder::encode(const AigGraph& aig) {
    CnfFormula cnf;
    var_map_.clear();

    // Create CNF variable for constant FALSE (AIG var 0)
    int const_var = cnf.new_var();
    var_map_[0] = const_var;
    cnf.add_unit(-const_var); // Force FALSE = 0

    // Create CNF variables for all inputs
    for (auto iv : aig.inputs())
        get_or_create(iv, cnf);

    // Create CNF variables for all latches
    for (size_t i = 0; i < aig.latches().size(); ++i) {
        // Latch variables are implicit in the AIG
        // They need CNF vars but we handle them via the latch var indices
    }

    // Encode each AND gate: output = fanin0 AND fanin1
    // Tseitin clauses for c = a AND b:
    //   (¬c ∨ a)        — if c is true, a must be true
    //   (¬c ∨ b)        — if c is true, b must be true
    //   (c ∨ ¬a ∨ ¬b)   — if both a,b are true, c must be true
    for (uint32_t v = 1; v <= aig.max_var(); ++v) {
        if (!aig.is_and(v)) continue;

        const auto& node = aig.and_node(v);
        int c = get_or_create(v, cnf);

        // Get or create vars for fanins
        get_or_create(aig_var(node.fanin0), cnf);
        get_or_create(aig_var(node.fanin1), cnf);

        CnfLit a = lit_convert(node.fanin0);
        CnfLit b = lit_convert(node.fanin1);

        cnf.add_clause({-c, a});          // ¬c → a
        cnf.add_clause({-c, b});          // ¬c → b
        cnf.add_clause({c, -a, -b});      // (a ∧ b) → c
    }

    return cnf;
}

} // namespace sf
