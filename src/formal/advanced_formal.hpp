#pragma once
// SiliconForge — Advanced Formal Verification Engine
// Phase 44: IC3/PDR, LTL Model Checking, CEGAR, Craig Interpolation
//
// References:
//   - Bradley, "SAT-Based Model Checking without Unrolling", VMCAI 2011 (IC3)
//   - Een et al., "Efficient Implementation of Property Directed Reachability", FMCAD 2011
//   - Clarke et al., "Counterexample-Guided Abstraction Refinement", CAV 2000
//   - McMillan, "Interpolation and SAT-Based Model Checking", CAV 2003

#include "formal/bmc.hpp"
#include "formal/k_induction.hpp"
#include "formal/tseitin.hpp"
#include "sat/cdcl_solver.hpp"
#include "sat/cnf.hpp"
#include "core/aig.hpp"
#include <vector>
#include <string>
#include <set>
#include <map>
#include <functional>
#include <memory>

namespace sf {

// ═════════════════════════════════════════════════════════════════════════════
//  IC3/PDR — Property-Directed Reachability
// ═════════════════════════════════════════════════════════════════════════════

// A cube is a conjunction of literals (partial state assignment)
using Cube = std::vector<int>; // CNF literals (positive = var true, negative = var false)

struct Ic3Config {
    int max_frames = 200;          // max frame depth
    int max_cti_depth = 1000;      // max CTI (counterexample-to-induction) queue
    bool generalize = true;        // enable cube generalization
    bool propagate = true;         // enable clause propagation
    bool verbose = false;
};

struct Ic3Result {
    enum Status { PROVEN, REFUTED, UNKNOWN };
    Status status = UNKNOWN;
    int frames_used = 0;
    int clauses_learned = 0;
    int fixed_point_frame = -1;    // frame where F_i == F_{i+1}
    std::vector<std::vector<bool>> counterexample; // if refuted
    double time_ms = 0;
    std::string message;
};

class Ic3Engine {
public:
    explicit Ic3Engine(const AigGraph& aig, const Ic3Config& cfg = {});

    // Run IC3/PDR algorithm
    Ic3Result check();

    // Query frames
    int num_frames() const { return (int)frames_.size(); }
    int clauses_in_frame(int f) const;

private:
    const AigGraph& aig_;
    Ic3Config cfg_;

    // Frames: F_0, F_1, ..., F_N
    // Each frame is a set of blocking clauses (negated cubes)
    std::vector<std::vector<Cube>> frames_;

    // Internal operations
    bool init_check();                        // Check if init state is bad
    bool is_blocked(const Cube& cube, int frame); // Is cube blocked in frame?
    bool sat_relative(const Cube& cube, int frame, Cube& predecessor);
    Cube generalize(const Cube& cube, int frame); // MIC (minimal inductive clause)
    bool propagate_clauses();                  // Push clauses forward
    bool check_fixed_point();                  // F_i == F_{i+1}?

    // SAT helpers
    bool is_initial(const Cube& cube);         // Does cube intersect initial states?
    void block_cube(const Cube& cube, int frame);

    // Proof obligation queue
    struct ProofObligation {
        Cube cube;
        int frame;
        int depth;
        bool operator>(const ProofObligation& o) const { return frame > o.frame; }
    };
};

// ═════════════════════════════════════════════════════════════════════════════
//  LTL Model Checking
// ═════════════════════════════════════════════════════════════════════════════

// LTL formula AST
enum class LtlOp {
    ATOM,       // propositional variable (e.g., signal name mapped to AIG)
    NOT,        // ¬φ
    AND,        // φ₁ ∧ φ₂
    OR,         // φ₁ ∨ φ₂
    IMPLIES,    // φ₁ → φ₂
    NEXT,       // Xφ (next state)
    GLOBALLY,   // Gφ (always)
    FINALLY,    // Fφ (eventually)
    UNTIL,      // φ₁ U φ₂
    RELEASE,    // φ₁ R φ₂
};

struct LtlFormula {
    LtlOp op = LtlOp::ATOM;
    std::string atom_name;        // for ATOM nodes
    int atom_signal = -1;         // AIG output index or latch index
    std::shared_ptr<LtlFormula> left;   // first operand
    std::shared_ptr<LtlFormula> right;  // second operand (for binary ops)
};

// Builder helpers
std::shared_ptr<LtlFormula> ltl_atom(const std::string& name, int signal = -1);
std::shared_ptr<LtlFormula> ltl_not(std::shared_ptr<LtlFormula> f);
std::shared_ptr<LtlFormula> ltl_and(std::shared_ptr<LtlFormula> a, std::shared_ptr<LtlFormula> b);
std::shared_ptr<LtlFormula> ltl_or(std::shared_ptr<LtlFormula> a, std::shared_ptr<LtlFormula> b);
std::shared_ptr<LtlFormula> ltl_globally(std::shared_ptr<LtlFormula> f);
std::shared_ptr<LtlFormula> ltl_finally(std::shared_ptr<LtlFormula> f);
std::shared_ptr<LtlFormula> ltl_next(std::shared_ptr<LtlFormula> f);
std::shared_ptr<LtlFormula> ltl_until(std::shared_ptr<LtlFormula> a, std::shared_ptr<LtlFormula> b);

struct LtlResult {
    enum Status { HOLDS, VIOLATED, UNKNOWN };
    Status status = UNKNOWN;
    int bound_checked = 0;         // BMC depth for bounded check
    std::vector<std::vector<bool>> counterexample; // lasso-shaped trace
    double time_ms = 0;
    std::string message;
};

class LtlChecker {
public:
    explicit LtlChecker(const AigGraph& aig);

    // Check LTL property via bounded model checking
    LtlResult check(std::shared_ptr<LtlFormula> property, int max_bound = 30);

    // Check safety property: G(p) — always p
    LtlResult check_safety(int output_index, int max_bound = 30);

    // Check liveness property: GF(p) — infinitely often p
    LtlResult check_liveness(int output_index, int max_bound = 20);

private:
    const AigGraph& aig_;

    // Convert LTL to safety check via monitor construction
    bool is_safety_formula(const std::shared_ptr<LtlFormula>& f) const;
    int formula_depth(const std::shared_ptr<LtlFormula>& f) const;
};

// ═════════════════════════════════════════════════════════════════════════════
//  CEGAR — Counterexample-Guided Abstraction Refinement
// ═════════════════════════════════════════════════════════════════════════════

struct CegarConfig {
    int max_refinements = 50;      // max abstraction-refinement iterations
    int bmc_depth = 20;            // BMC depth for abstract model check
    double initial_abstraction = 0.3; // fraction of latches to include initially
    bool use_cone_of_influence = true; // start with COI latches
};

struct CegarResult {
    enum Status { PROVEN, REFUTED, UNKNOWN };
    Status status = UNKNOWN;
    int refinement_iterations = 0;
    int abstract_latches = 0;      // latches in final abstraction
    int concrete_latches = 0;      // total latches in original design
    int spurious_cex_count = 0;    // number of spurious counterexamples
    double time_ms = 0;
    std::string message;
};

class CegarEngine {
public:
    explicit CegarEngine(const AigGraph& aig, const CegarConfig& cfg = {});

    // Run CEGAR loop
    CegarResult check();

    // Query abstraction state
    int num_visible_latches() const { return (int)visible_latches_.size(); }
    int num_total_latches() const;

private:
    const AigGraph& aig_;
    CegarConfig cfg_;
    std::set<int> visible_latches_; // currently visible latch indices

    // Build abstract model (visible latches kept, others freed)
    AigGraph build_abstract_model() const;

    // Check if counterexample is spurious
    bool is_spurious(const std::vector<std::vector<bool>>& trace) const;

    // Refine abstraction by adding relevant latches
    void refine(const std::vector<std::vector<bool>>& trace);

    // Cone-of-influence analysis
    std::set<int> compute_coi() const;
};

// ═════════════════════════════════════════════════════════════════════════════
//  Craig Interpolation
// ═════════════════════════════════════════════════════════════════════════════

struct InterpolationResult {
    bool unsat = false;              // true if A ∧ B is UNSAT
    CnfFormula interpolant;          // I such that A ⊢ I and I ∧ B is UNSAT
    int interpolant_vars = 0;        // variables in interpolant
    int interpolant_clauses = 0;     // clauses in interpolant
    double time_ms = 0;
    std::string message;
};

class CraigInterpolator {
public:
    // Compute interpolant from A ∧ B where A ∧ B is UNSAT
    // Variables in `shared_vars` are common to A and B
    static InterpolationResult interpolate(
        const CnfFormula& A,
        const CnfFormula& B,
        const std::vector<int>& shared_vars);

    // McMillan's interpolation from resolution proof
    // Uses the UNSAT proof from CDCL solver
    static InterpolationResult proof_based_interpolation(
        const CnfFormula& A,
        const CnfFormula& B);

private:
    // Classify variables as A-local, B-local, or shared
    struct VarClassification {
        std::set<int> a_local;
        std::set<int> b_local;
        std::set<int> shared;
    };
    static VarClassification classify_vars(const CnfFormula& A, const CnfFormula& B,
                                            const std::vector<int>& shared);
};

// ═════════════════════════════════════════════════════════════════════════════
//  Unified Advanced Formal Engine
// ═════════════════════════════════════════════════════════════════════════════

struct AdvFormalResult {
    Ic3Result ic3;
    LtlResult ltl;
    CegarResult cegar;
    InterpolationResult interpolation;
    double total_time_ms = 0;
    std::string summary;
};

class AdvancedFormalEngine {
public:
    explicit AdvancedFormalEngine(const AigGraph& aig);

    // Run all advanced verification techniques
    AdvFormalResult verify_all(int output_index = 0);

    // Individual engines
    Ic3Result run_ic3(const Ic3Config& cfg = {});
    LtlResult run_ltl(std::shared_ptr<LtlFormula> property, int bound = 30);
    CegarResult run_cegar(const CegarConfig& cfg = {});
    InterpolationResult run_interpolation(const CnfFormula& A, const CnfFormula& B);

private:
    const AigGraph& aig_;
};

} // namespace sf
