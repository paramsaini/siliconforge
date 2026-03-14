// SiliconForge — CDCL SAT Solver Implementation
#include "sat/cdcl_solver.hpp"
#include "core/thread_pool.hpp"
#include <iostream>
#include <chrono>
#include <atomic>
#include <mutex>
#include <thread>

namespace sf {

CdclSolver::CdclSolver() {
    ok_ = true;
    // Reserve var 0 as unused (DIMACS convention: vars are 1-indexed)
    assigns_.push_back(LBool::UNDEF);
    reason_.push_back(-1);
    level_.push_back(-1);
    activity_.push_back(0.0);
    seen_.push_back(false);
    watches_.resize(2); // lits 0 and 1 unused
}

CdclSolver::CdclSolver(const CnfFormula& formula) : CdclSolver() {
    for (int i = 0; i < formula.num_vars(); ++i) new_var();
    for (auto& cl : formula.clauses()) add_clause(cl);
}

int CdclSolver::new_var() {
    ++num_vars_;
    assigns_.push_back(LBool::UNDEF);
    reason_.push_back(-1);
    level_.push_back(-1);
    activity_.push_back(0.0);
    seen_.push_back(false);
    watches_.resize(2 * (num_vars_ + 1));
    return num_vars_;
}

void CdclSolver::ensure_var(int v) {
    while (v > num_vars_) new_var();
}

void CdclSolver::add_clause(const std::vector<CnfLit>& clause) {
    std::vector<int> ilits;
    for (auto l : clause) {
        ensure_var(cnf_var(l));
        ilits.push_back(to_ilit(l));
    }
    add_clause_internal(std::move(ilits), false);
}

// ok_ is now a member variable in the header

int CdclSolver::add_clause_internal(std::vector<int> lits, bool learned) {
    // Remove duplicates
    std::sort(lits.begin(), lits.end());
    lits.erase(std::unique(lits.begin(), lits.end()), lits.end());
    // Check tautology
    for (size_t i = 1; i < lits.size(); ++i)
        if (lits[i] == neg(lits[i-1])) return -1;

    if (lits.empty()) { ok_ = false; return -1; }

    int ci = (int)clauses_.size();
    clauses_.push_back({std::move(lits), learned});

    auto& cl = clauses_[ci].lits;
    if (cl.size() >= 2) {
        // Ensure watched literals are not already-false when possible.
        // Move non-false literals to the first two positions so that the
        // two-watched-literal scheme can fire correctly during propagation.
        for (int w = 0; w < 2; w++) {
            if (val(cl[w]) == LBool::FALSE) {
                for (size_t j = w + 1; j < cl.size(); j++) {
                    if (val(cl[j]) != LBool::FALSE) {
                        std::swap(cl[w], cl[j]);
                        break;
                    }
                }
            }
        }
        watches_[cl[0]].push_back(ci);
        watches_[cl[1]].push_back(ci);

        // After reordering, check if clause is effectively unit or conflict
        if (val(cl[0]) == LBool::FALSE) {
            // All literals are false — conflict at current level
            ok_ = false;
        } else if (val(cl[1]) == LBool::FALSE && val(cl[0]) == LBool::UNDEF) {
            // Only cl[0] is unassigned — clause is unit, propagate
            assign_lit(cl[0], ci);
        }
    } else if (cl.size() == 1) {
        if (val(cl[0]) == LBool::FALSE) {
            ok_ = false; // Conflict at level 0
        } else if (val(cl[0]) == LBool::UNDEF) {
            assign_lit(cl[0], ci);
        }
        // If already TRUE, clause is satisfied — nothing to do
    }
    return ci;
}

CdclSolver::LBool CdclSolver::val(int ilit) const {
    LBool v = assigns_[ivar(ilit)];
    if (v == LBool::UNDEF) return LBool::UNDEF;
    bool pos = !(ilit & 1);
    bool is_true = (v == LBool::TRUE);
    return (pos == is_true) ? LBool::TRUE : LBool::FALSE;
}

void CdclSolver::assign_lit(int ilit, int reason_cl) {
    int v = ivar(ilit);
    assigns_[v] = (ilit & 1) ? LBool::FALSE : LBool::TRUE;
    reason_[v] = reason_cl;
    level_[v] = decision_level();
    trail_.push_back(v);
}

int CdclSolver::propagate() {
    while (qhead_ < (int)trail_.size()) {
        int v = trail_[qhead_++];
        stats_.propagations++;
        // The false literal for this assignment
        int false_lit = (assigns_[v] == LBool::TRUE) ? (2*v+1) : (2*v);
        auto& ws = watches_[false_lit];
        int i = 0, j = 0;
        while (i < (int)ws.size()) {
            int ci = ws[i];
            auto& cl = clauses_[ci].lits;

            if (cl.size() == 1) {
                ws[j++] = ws[i++];
                return ci; // conflict
            }

            // Ensure false_lit is at position 1
            if (cl[0] == false_lit) std::swap(cl[0], cl[1]);
            assert(cl[1] == false_lit);

            // If other watched lit is true, clause satisfied
            if (val(cl[0]) == LBool::TRUE) {
                ws[j++] = ws[i++];
                continue;
            }

            // Find new literal to watch
            bool found = false;
            for (size_t k = 2; k < cl.size(); ++k) {
                if (val(cl[k]) != LBool::FALSE) {
                    std::swap(cl[1], cl[k]);
                    watches_[cl[1]].push_back(ci);
                    found = true;
                    break;
                }
            }
            if (found) { ++i; continue; }

            // No replacement — unit or conflict
            ws[j++] = ws[i++];
            if (val(cl[0]) == LBool::FALSE) {
                // Conflict — copy remaining watches
                while (i < (int)ws.size()) ws[j++] = ws[i++];
                ws.resize(j);
                return ci;
            }
            // Unit propagation
            assign_lit(cl[0], ci);
        }
        ws.resize(j);
    }
    return -1;
}

void CdclSolver::analyze(int confl, std::vector<int>& learnt, int& bt_level) {
    learnt.clear();
    bt_level = 0;
    learnt.push_back(-1); // placeholder for asserting literal

    int counter = 0;
    int trail_idx = (int)trail_.size() - 1;
    int p = -1;

    auto process_clause = [&](int ci, int skip_var) {
        auto& cl = clauses_[ci].lits;
        for (int lit : cl) {
            int v = ivar(lit);
            if (v == skip_var) continue;
            if (seen_[v] || level_[v] == 0) continue;
            seen_[v] = true;
            bump_activity(v);
            if (level_[v] == decision_level())
                counter++;
            else {
                learnt.push_back(lit);
                bt_level = std::max(bt_level, level_[v]);
            }
        }
    };

    process_clause(confl, -1);

    while (counter > 0) {
        // Find next assigned variable on trail that was seen
        while (!seen_[trail_[trail_idx]]) trail_idx--;
        p = trail_[trail_idx];
        seen_[p] = false;
        counter--;
        if (counter > 0) {
            assert(reason_[p] >= 0);
            process_clause(reason_[p], p);
        }
        trail_idx--;
    }

    // The asserting literal: negate the last UIP's assignment
    int assert_lit = (assigns_[p] == LBool::TRUE) ? (2*p+1) : (2*p);
    learnt[0] = assert_lit;

    // Clear seen flags
    for (size_t i = 1; i < learnt.size(); ++i) seen_[ivar(learnt[i])] = false;

    if (learnt.size() == 1) bt_level = 0;
}

void CdclSolver::backtrack(int target_level) {
    if (decision_level() <= target_level) return;
    int stop = trail_lim_[target_level];
    for (int i = (int)trail_.size() - 1; i >= stop; --i) {
        int v = trail_[i];
        assigns_[v] = LBool::UNDEF;
        reason_[v] = -1;
        level_[v] = -1;
    }
    trail_.resize(stop);
    trail_lim_.resize(target_level);
    qhead_ = (int)trail_.size();
}

int CdclSolver::pick_branch() {
    int best = -1;
    double best_act = -1.0;
    for (int v = 1; v <= num_vars_; ++v) {
        if (assigns_[v] == LBool::UNDEF && activity_[v] > best_act) {
            best_act = activity_[v];
            best = v;
        }
    }
    return best;
}

void CdclSolver::bump_activity(int var) {
    activity_[var] += var_inc_;
    if (activity_[var] > 1e100) {
        for (int i = 1; i <= num_vars_; ++i) activity_[i] *= 1e-100;
        var_inc_ *= 1e-100;
    }
}

void CdclSolver::decay_activity() { var_inc_ /= 0.95; }

SatResult CdclSolver::solve() { return solve({}); }

SatResult CdclSolver::solve(const std::vector<CnfLit>& assumptions) {
    // Check if we already found contradiction during clause addition
    if (!ok_) return SatResult::UNSAT;

    // Initial propagation of unit clauses from construction
    {
        int confl = propagate();
        if (confl >= 0) return SatResult::UNSAT;
    }

    // Apply assumptions as decisions at the earliest decision levels.
    // Track the assumption boundary so we can detect when conflict analysis
    // forces backtracking past it (MiniSat-style assumption handling).
    int assumption_level = decision_level(); // level before assumptions (usually 0)
    for (auto lit : assumptions) {
        int il = to_ilit(lit);
        if (val(il) == LBool::FALSE) {
            backtrack(assumption_level);
            return SatResult::UNSAT;
        }
        if (val(il) == LBool::UNDEF) {
            trail_lim_.push_back((int)trail_.size());
            assign_lit(il, -1);
            int confl = propagate();
            if (confl >= 0) {
                backtrack(assumption_level);
                return SatResult::UNSAT;
            }
        }
    }
    int search_root = decision_level(); // level after all assumptions applied

    int restart_limit = 100;
    int conflicts_since_restart = 0;

    while (true) {
        int confl = propagate();
        if (confl >= 0) {
            stats_.conflicts++;
            conflicts_since_restart++;
            if (decision_level() <= assumption_level) {
                backtrack(assumption_level);
                return SatResult::UNSAT;
            }

            std::vector<int> learnt;
            int bt_level;
            analyze(confl, learnt, bt_level);

            // If conflict analysis requires backtracking into or below
            // assumption levels, the formula is UNSAT under these assumptions.
            if (bt_level < search_root) {
                backtrack(assumption_level);
                // Still add the learned clause — it's a valid consequence
                add_clause_internal(std::move(learnt), true);
                return SatResult::UNSAT;
            }

            backtrack(bt_level);
            int ci = add_clause_internal(std::move(learnt), true);
            if (ci >= 0 && clauses_[ci].lits.size() == 1) {
                // Already assigned by add_clause_internal
            } else if (ci >= 0) {
                assign_lit(clauses_[ci].lits[0], ci);
            }
            decay_activity();

            // Restart check — don't backtrack past assumption levels
            if (conflicts_since_restart >= restart_limit) {
                stats_.restarts++;
                backtrack(search_root);
                conflicts_since_restart = 0;
                restart_limit = (int)(restart_limit * 1.5);
            }
        } else {
            // No conflict — pick a decision variable
            int v = pick_branch();
            if (v < 0) return SatResult::SAT; // All assigned

            stats_.decisions++;
            trail_lim_.push_back((int)trail_.size());
            assign_lit(2 * v, -1); // Decide positive
        }
    }
}

bool CdclSolver::model_value(int var) const {
    assert(var >= 1 && var <= num_vars_);
    return assigns_[var] == LBool::TRUE;
}

// --- New CDCL enhancement methods ---

void CdclSolver::enable_vsids(bool enable) {
    vsids_enabled_ = enable;
}

void CdclSolver::set_clause_db_config(const ClauseDbConfig& cfg) {
    clause_db_cfg_ = cfg;
}

void CdclSolver::reduce_clause_db() {
    // Compute LBD for each learned clause, remove those above threshold.
    // LBD = number of distinct decision levels among the clause's literals.
    auto compute_lbd = [&](const Clause& cl) -> int {
        std::vector<int> levels;
        for (int lit : cl.lits) {
            int v = ivar(lit);
            int lv = level_[v];
            if (lv >= 0) {
                bool found = false;
                for (int l : levels) {
                    if (l == lv) { found = true; break; }
                }
                if (!found) levels.push_back(lv);
            }
        }
        return (int)levels.size();
    };

    // Build set of reason clauses (must not be deleted)
    std::vector<bool> is_reason(clauses_.size(), false);
    for (int v = 1; v <= num_vars_; ++v) {
        if (reason_[v] >= 0 && reason_[v] < (int)clauses_.size()) {
            is_reason[reason_[v]] = true;
        }
    }

    // Identify clauses to delete
    std::vector<bool> to_delete(clauses_.size(), false);
    for (int ci = 0; ci < (int)clauses_.size(); ++ci) {
        auto& cl = clauses_[ci];
        if (!cl.learned) continue;
        if (is_reason[ci]) continue;
        if (cl.lits.empty()) continue;
        int lbd = compute_lbd(cl);
        if (lbd > clause_db_cfg_.lbd_threshold) {
            to_delete[ci] = true;
            deleted_count_++;
        }
    }

    // Clean up watch lists: remove references to deleted clauses
    for (auto& ws : watches_) {
        int j = 0;
        for (int i = 0; i < (int)ws.size(); ++i) {
            if (!to_delete[ws[i]]) {
                ws[j++] = ws[i];
            }
        }
        ws.resize(j);
    }

    // Clear deleted clauses (empty their lits to mark them dead)
    for (int ci = 0; ci < (int)clauses_.size(); ++ci) {
        if (to_delete[ci]) {
            clauses_[ci].lits.clear();
        }
    }
}

void CdclSolver::set_restart_policy(RestartPolicy policy) {
    restart_policy_ = policy;
}

void CdclSolver::enable_phase_saving(bool enable) {
    phase_saving_ = enable;
    if (enable) {
        saved_phase_.resize(num_vars_ + 1, false);
    }
}

CdclSolver::PreprocessResult CdclSolver::preprocess() {
    PreprocessResult result{0, 0, 0, 0};

    // --- Unit propagation: find unit clauses and propagate ---
    for (int ci = 0; ci < (int)clauses_.size(); ++ci) {
        auto& cl = clauses_[ci].lits;
        if (cl.size() == 1 && val(cl[0]) == LBool::UNDEF) {
            assign_lit(cl[0], ci);
            result.unit_propagated++;
        }
    }
    if (result.unit_propagated > 0) {
        int confl = propagate();
        if (confl >= 0) {
            ok_ = false;
            return result;
        }
    }

    // --- Pure literal detection ---
    // For each variable, track whether it appears positive, negative, or both
    // in currently non-satisfied clauses.
    std::vector<uint8_t> polarity(num_vars_ + 1, 0); // bit 0 = pos, bit 1 = neg
    for (auto& clause : clauses_) {
        if (clause.lits.empty()) continue;
        // Check if clause is already satisfied
        bool satisfied = false;
        for (int lit : clause.lits) {
            if (val(lit) == LBool::TRUE) { satisfied = true; break; }
        }
        if (satisfied) continue;
        for (int lit : clause.lits) {
            int v = ivar(lit);
            if (assigns_[v] != LBool::UNDEF) continue;
            if (lit & 1) polarity[v] |= 2; // negative
            else         polarity[v] |= 1; // positive
        }
    }
    for (int v = 1; v <= num_vars_; ++v) {
        if (assigns_[v] != LBool::UNDEF) continue;
        if (polarity[v] == 1) {
            // appears only positive
            assign_lit(2 * v, -1);
            result.pure_literals++;
        } else if (polarity[v] == 2) {
            // appears only negative
            assign_lit(2 * v + 1, -1);
            result.pure_literals++;
        }
    }
    if (result.pure_literals > 0) {
        int confl = propagate();
        if (confl >= 0) {
            ok_ = false;
            return result;
        }
    }

    // --- Subsumption: remove clauses that are supersets of other clauses ---
    // Simple O(n^2) approach: for each pair, check if smaller subsumes larger.
    int n = (int)clauses_.size();
    std::vector<bool> removed(n, false);
    for (int i = 0; i < n; ++i) {
        if (removed[i] || clauses_[i].lits.empty()) continue;
        for (int j = i + 1; j < n; ++j) {
            if (removed[j] || clauses_[j].lits.empty()) continue;
            auto& small = (clauses_[i].lits.size() <= clauses_[j].lits.size())
                              ? clauses_[i].lits : clauses_[j].lits;
            auto& large = (clauses_[i].lits.size() <= clauses_[j].lits.size())
                              ? clauses_[j].lits : clauses_[i].lits;
            int larger_idx = (clauses_[i].lits.size() <= clauses_[j].lits.size()) ? j : i;
            if (small.size() == large.size() && i != larger_idx) continue; // skip equal-size unless i is smaller

            // Check if all lits in small appear in large
            bool subsumes = true;
            for (int lit : small) {
                bool found = false;
                for (int lit2 : large) {
                    if (lit == lit2) { found = true; break; }
                }
                if (!found) { subsumes = false; break; }
            }
            if (subsumes && small.size() < large.size()) {
                removed[larger_idx] = true;
                result.subsumptions++;
            }
        }
    }

    // Clean up watch lists for removed clauses and clear them
    if (result.subsumptions > 0) {
        for (auto& ws : watches_) {
            int j = 0;
            for (int i = 0; i < (int)ws.size(); ++i) {
                if (!removed[ws[i]]) ws[j++] = ws[i];
            }
            ws.resize(j);
        }
        for (int ci = 0; ci < n; ++ci) {
            if (removed[ci]) clauses_[ci].lits.clear();
        }
    }

    return result;
}

CdclSolver::SolverStats CdclSolver::get_stats() const {
    return SolverStats{
        stats_.decisions,
        stats_.propagations,
        stats_.conflicts,
        learned_count_,
        deleted_count_,
        stats_.restarts,
        0.0
    };
}

// ============================================================================
// Portfolio SAT Solver — parallel multi-strategy solving
// ============================================================================

PortfolioResult portfolio_solve(const CnfFormula& formula, int num_configs) {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Default: use hardware concurrency (capped at 4 for SAT)
    if (num_configs <= 0)
        num_configs = std::min(4u, std::max(1u, std::thread::hardware_concurrency()));

    // Define diverse solver configurations
    struct SolverConfig {
        CdclSolver::RestartPolicy restart;
        bool vsids;
        bool phase_saving;
        CdclSolver::ClauseDbConfig clause_cfg;
    };

    std::vector<SolverConfig> configs = {
        {CdclSolver::RestartPolicy::GEOMETRIC, true,  false, {2000, 6, 0.95}},
        {CdclSolver::RestartPolicy::LUBY,      true,  true,  {3000, 8, 0.90}},
        {CdclSolver::RestartPolicy::GLUCOSE,    true,  true,  {1500, 4, 0.95}},
        {CdclSolver::RestartPolicy::GEOMETRIC,  false, false, {5000, 10, 0.99}},
    };
    while ((int)configs.size() < num_configs)
        configs.push_back(configs[configs.size() % 4]);
    configs.resize(num_configs);

    // Shared result state
    std::atomic<bool> done{false};
    std::mutex result_mutex;
    PortfolioResult portfolio;
    portfolio.per_config_stats.resize(num_configs);

    // Launch solvers using ThreadPool
    sf::ThreadPool pool(num_configs);
    std::vector<std::future<void>> futures;

    for (int ci = 0; ci < num_configs; ++ci) {
        futures.push_back(pool.submit([&, ci]() {
            CdclSolver solver(formula);
            solver.set_restart_policy(configs[ci].restart);
            solver.enable_vsids(configs[ci].vsids);
            solver.enable_phase_saving(configs[ci].phase_saving);
            solver.set_clause_db_config(configs[ci].clause_cfg);

            SatResult res = solver.solve();

            portfolio.per_config_stats[ci] = solver.get_stats();

            if (!done.exchange(true)) {
                std::lock_guard<std::mutex> lock(result_mutex);
                portfolio.result = res;
                portfolio.winning_config = ci;
            }
        }));
    }

    for (auto& f : futures) f.get();

    auto t1 = std::chrono::high_resolution_clock::now();
    portfolio.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    return portfolio;
}

} // namespace sf
