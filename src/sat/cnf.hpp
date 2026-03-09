#pragma once
// SiliconForge — CNF Formula Representation
// DIMACS-compatible: variables 1-indexed, literals signed integers

#include <cstdint>
#include <vector>
#include <string>
#include <sstream>

namespace sf {

using CnfLit = int32_t;

inline int     cnf_var(CnfLit lit) { return lit > 0 ? lit : -lit; }
inline bool    cnf_sign(CnfLit lit) { return lit < 0; }
inline CnfLit  cnf_neg(CnfLit lit) { return -lit; }

class CnfFormula {
public:
    CnfFormula() = default;

    int new_var() { return ++num_vars_; }
    int num_vars() const { return num_vars_; }
    size_t num_clauses() const { return clauses_.size(); }

    void add_clause(std::vector<CnfLit> lits) {
        clauses_.push_back(std::move(lits));
    }
    void add_clause(std::initializer_list<CnfLit> lits) {
        clauses_.emplace_back(lits);
    }
    void add_unit(CnfLit lit) { clauses_.push_back({lit}); }

    // Force variable count (useful when building from AIG with known var count)
    void set_num_vars(int n) { if (n > num_vars_) num_vars_ = n; }

    const std::vector<std::vector<CnfLit>>& clauses() const { return clauses_; }

    std::string to_dimacs() const {
        std::ostringstream ss;
        ss << "p cnf " << num_vars_ << " " << clauses_.size() << "\n";
        for (auto& c : clauses_) {
            for (auto l : c) ss << l << " ";
            ss << "0\n";
        }
        return ss.str();
    }

private:
    int num_vars_ = 0;
    std::vector<std::vector<CnfLit>> clauses_;
};

} // namespace sf
