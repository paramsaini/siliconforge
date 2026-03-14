#pragma once
// SiliconForge — CDCL SAT Solver
// Conflict-Driven Clause Learning with:
//   - 2-watched-literal propagation
//   - First-UIP conflict analysis
//   - Non-chronological backjumping
//   - VSIDS variable activity heuristic

#include "solver.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <limits>

namespace sf {

class CdclSolver : public SatSolver {
public:
    CdclSolver();
    explicit CdclSolver(const CnfFormula& formula);

    SatResult solve() override;
    SatResult solve(const std::vector<CnfLit>& assumptions) override;
    bool model_value(int var) const override;
    void add_clause(const std::vector<CnfLit>& clause) override;
    int new_var() override;
    int num_vars() const override { return num_vars_; }

    struct Stats {
        uint64_t decisions = 0, propagations = 0, conflicts = 0, restarts = 0;
    };
    const Stats& stats() const { return stats_; }

    // VSIDS activity-based branching
    void enable_vsids(bool enable = true);

    // LBD-based clause deletion
    struct ClauseDbConfig {
        int max_learned_clauses;
        int lbd_threshold;          // delete clauses with LBD > this
        double decay_factor;
    };
    void set_clause_db_config(const ClauseDbConfig& cfg);
    void reduce_clause_db();

    // Restart policy
    enum class RestartPolicy { LUBY, GEOMETRIC, GLUCOSE };
    void set_restart_policy(RestartPolicy policy);

    // Phase saving
    void enable_phase_saving(bool enable = true);

    // Preprocessing
    struct PreprocessResult {
        int unit_propagated;
        int pure_literals;
        int subsumptions;
        int self_subsumptions;
    };
    PreprocessResult preprocess();

    // Statistics
    struct SolverStats {
        uint64_t decisions;
        uint64_t propagations;
        uint64_t conflicts;
        uint64_t learned_clauses;
        uint64_t deleted_clauses;
        uint64_t restarts;
        double time_ms;
    };
    SolverStats get_stats() const;

private:
    enum class LBool : uint8_t { UNDEF = 0, TRUE = 1, FALSE = 2 };

    struct Clause {
        std::vector<int> lits; // internal lit indices
        bool learned = false;
    };

    // Internal literal encoding: var v -> pos=2v, neg=2v+1
    static int to_ilit(CnfLit l) { return l > 0 ? 2*l : 2*(-l)+1; }
    static int neg(int il) { return il ^ 1; }
    static int ivar(int il) { return il >> 1; }

    int num_vars_ = 0;
    std::vector<Clause> clauses_;
    std::vector<LBool> assigns_;
    std::vector<int> trail_;
    std::vector<int> trail_lim_;
    std::vector<int> reason_;
    std::vector<int> level_;
    std::vector<std::vector<int>> watches_;
    std::vector<double> activity_;
    std::vector<bool> seen_;
    double var_inc_ = 1.0;
    int qhead_ = 0;
    Stats stats_;
    bool ok_ = true;

    bool vsids_enabled_ = true;
    ClauseDbConfig clause_db_cfg_ = {2000, 6, 0.95};
    RestartPolicy restart_policy_ = RestartPolicy::GEOMETRIC;
    bool phase_saving_ = false;
    std::vector<bool> saved_phase_;
    uint64_t learned_count_ = 0;
    uint64_t deleted_count_ = 0;

    void ensure_var(int v);
    int decision_level() const { return (int)trail_lim_.size(); }
    LBool val(int ilit) const;
    void assign_lit(int ilit, int reason_cl);
    int propagate(); // returns conflict clause index or -1
    void analyze(int confl, std::vector<int>& learnt, int& bt_level);
    void backtrack(int level);
    int pick_branch();
    void bump_activity(int var);
    void decay_activity();
    int add_clause_internal(std::vector<int> lits, bool learned);
};

// Portfolio SAT solver: runs N CDCL instances with different configurations
// in parallel. First to return SAT/UNSAT wins. Uses ThreadPool for execution.
struct PortfolioResult {
    SatResult result = SatResult::UNKNOWN;
    int winning_config = -1;
    double time_ms = 0;
    std::vector<CdclSolver::SolverStats> per_config_stats;
};

PortfolioResult portfolio_solve(const CnfFormula& formula, int num_configs = 0);

} // namespace sf
