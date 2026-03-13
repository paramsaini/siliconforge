#pragma once
// SiliconForge — Logic Equivalence Checking (LEC)
// Verifies structural/functional equivalence between two netlists
// (e.g., pre-synthesis vs post-synthesis, or pre-ECO vs post-ECO).
// Uses AIG-based miter construction + SAT solving.
// Reference: Brand, "Verification of Large Synthesized Designs", ICCAD 1993

#include "core/netlist.hpp"
#include "core/aig.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace sf {

struct LecResult {
    bool equivalent = false;
    int key_points_compared = 0;
    int matched = 0;
    int mismatched = 0;
    int aborted = 0;
    double time_ms = 0;

    struct Mismatch {
        std::string point_name;
        std::string detail;
    };
    std::vector<Mismatch> mismatches;
    std::string message;
};

// Structural matching key-point identification
struct KeyPoint {
    int net_idx = -1;
    std::string name;
    enum Type { PRIMARY_IO, REGISTER, INTERNAL_CUT } type = PRIMARY_IO;
    bool matched = false;
    int match_idx = -1;
};

struct StructuralMatch {
    int total_key_points = 0;
    int matched = 0;
    int unmatched = 0;
    std::vector<std::pair<int,int>> match_pairs;
    std::vector<int> unmatched_ref;
    std::vector<int> unmatched_impl;
};

// Cone-of-influence reduction
struct ConeReduction {
    std::vector<int> relevant_gates;
    int original_size = 0;
    int reduced_size = 0;
    double reduction_pct = 0;
};

// Sequential equivalence checking
struct SeqEquivResult {
    bool equivalent = false;
    int unrolling_depth = 0;
    int registers_matched = 0;
    std::string message;
};

// Counter-example trace
struct LecCex {
    std::vector<std::pair<std::string,bool>> input_values;
    std::string output_name;
    bool ref_value = false;
    bool impl_value = false;
    std::vector<std::pair<std::string,bool>> internal_diffs;
};

class LecEngine {
public:
    LecEngine(const Netlist& golden, const Netlist& revised)
        : golden_(golden), revised_(revised) {}

    LecResult check();

    // Structural matching: identify key points in a netlist
    std::vector<KeyPoint> identify_key_points(const Netlist& nl);

    // Structural comparison between golden and revised
    StructuralMatch structural_compare();

    // Cone-of-influence reduction for a single output
    ConeReduction reduce_cone(int output_idx);

    // Sequential equivalence checking
    SeqEquivResult check_sequential(int max_depth = 10);

    // Get counterexample trace for last failure
    LecCex get_counterexample();

    // Enhanced LEC flow: structural + COI + sequential + CEX
    LecResult run_enhanced();

private:
    const Netlist& golden_;
    const Netlist& revised_;

    // Cached last-failure info for get_counterexample()
    LecCex last_cex_;
    bool has_cex_ = false;

    // Build AIG from netlist for a single output
    AigLit build_cone(AigGraph& aig, const Netlist& nl, NetId output,
                      std::unordered_map<NetId, AigLit>& net_map);

    // Compare single output cones using SAT
    bool compare_output(const std::string& name, NetId g_out, NetId r_out,
                        LecResult& result);

    // Simulation-based equivalence check (robust to structural changes)
    bool sim_compare(LecResult& result);

    // ── Tier 2: Hierarchical LEC ────────────────────────────────────────
public:
    // Hierarchical equivalence checking: decompose designs by module boundaries,
    // verify each module independently, compose results.
    struct HierLecConfig {
        bool enabled = false;
        int max_module_size = 500; // max gates per module before splitting
    };
    struct ModuleMatch {
        std::string module_name;
        int golden_start, golden_end;     // gate ID ranges
        int revised_start, revised_end;
        bool equivalent = false;
    };
    struct HierLecResult {
        bool equivalent = false;
        int modules_compared = 0;
        int modules_matched = 0;
        int modules_failed = 0;
        std::vector<ModuleMatch> module_results;
        std::string message;
    };
    HierLecResult hierarchical_check(const HierLecConfig& cfg);

private:
    // Partition netlist into modules by connectivity clustering
    std::vector<std::pair<int,int>> partition_into_modules(const Netlist& nl, int max_size);
};

} // namespace sf
