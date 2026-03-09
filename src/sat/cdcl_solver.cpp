// SiliconForge — CDCL SAT Solver Implementation
#include "sat/cdcl_solver.hpp"
#include <iostream>

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
        watches_[cl[0]].push_back(ci);
        watches_[cl[1]].push_back(ci);
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

    // Apply assumptions
    for (auto lit : assumptions) {
        int il = to_ilit(lit);
        if (val(il) == LBool::FALSE) return SatResult::UNSAT;
        if (val(il) == LBool::UNDEF) {
            trail_lim_.push_back((int)trail_.size());
            assign_lit(il, -1);
            int confl = propagate();
            if (confl >= 0) return SatResult::UNSAT;
        }
    }

    int restart_limit = 100;
    int conflicts_since_restart = 0;

    while (true) {
        int confl = propagate();
        if (confl >= 0) {
            stats_.conflicts++;
            conflicts_since_restart++;
            if (decision_level() == 0) return SatResult::UNSAT;

            std::vector<int> learnt;
            int bt_level;
            analyze(confl, learnt, bt_level);
            backtrack(bt_level);
            int ci = add_clause_internal(std::move(learnt), true);
            if (ci >= 0 && clauses_[ci].lits.size() == 1) {
                // Already assigned by add_clause_internal
            } else if (ci >= 0) {
                assign_lit(clauses_[ci].lits[0], ci);
            }
            decay_activity();

            // Restart check
            if (conflicts_since_restart >= restart_limit) {
                stats_.restarts++;
                backtrack(0);
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

} // namespace sf
