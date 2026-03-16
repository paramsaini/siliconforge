#pragma once
// SiliconForge — NPN Boolean Matching Engine
// Supports up to 6-input truth tables (uint64_t) with NPN canonical form.
// NPN equivalence: two Boolean functions are NPN-equivalent if one can be
// obtained from the other by Negating inputs (N), Permuting inputs (P),
// and Negating the output (N).
//
// NPN canonical form reduces a 6-input truth table to a representative
// member of its NPN equivalence class, enabling O(1) cell library matching
// instead of exhaustive structural pattern matching.
//
// References:
//   Hinsberger & Kolla, "Boolean Matching for Large Libraries", DAC 1998
//   Debnath & Sasao, "Efficient Computation of Canonical Form for Boolean
//                     Matching in Technology Mapping", ASP-DAC 2004
//   Soeken et al., "Exact Synthesis of Majority-Inverter Graphs", IEEE TCAD 2017

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <string>
#include <array>

namespace sf {

// 6-input truth table type (2^6 = 64 bits)
using TruthTable6 = uint64_t;

// NPN canonical form result
struct NpnCanonical {
    TruthTable6 canonical_tt = 0;     // canonical truth table
    uint8_t input_perm[6] = {};       // input permutation: canonical_input[i] = original_input[perm[i]]
    uint8_t input_neg = 0;            // bitmask: bit i=1 means input i was negated
    bool output_neg = false;           // true if output was negated
    int num_inputs = 0;
};

// NPN matching database entry
struct NpnCellEntry {
    std::string cell_name;
    TruthTable6 original_tt = 0;
    TruthTable6 canonical_tt = 0;
    int num_inputs = 0;
    double area = 0;
    double delay = 0;
    NpnCanonical npn_form;
};

class NpnMatcher {
public:
    // ── Truth table computation ──────────────────────────────────────────

    // Compute truth table for N-input function (N <= 6)
    // Pattern evaluation on AIG subgraph
    static TruthTable6 compute_tt(int num_inputs,
                                   const std::vector<bool>& pattern_results);

    // Build truth table from Boolean function string
    // Supports: A-F as inputs, &, |, ^, !, (, ), 0, 1
    static TruthTable6 func_to_tt6(const std::string& func, int num_inputs);

    // ── NPN canonical form computation ───────────────────────────────────

    // Compute NPN canonical form (exact, up to 6 inputs)
    // For <= 4 inputs: exhaustive enumeration (fast, ~50K permutations)
    // For 5-6 inputs: heuristic + partial enumeration
    static NpnCanonical compute_canonical(TruthTable6 tt, int num_inputs);

    // Fast NPN canonical for <= 4 inputs (exhaustive)
    static NpnCanonical compute_canonical_exact4(TruthTable6 tt, int num_inputs);

    // Heuristic NPN canonical for 5-6 inputs
    // Uses cofactor signature sorting + greedy column swap
    static NpnCanonical compute_canonical_heuristic(TruthTable6 tt, int num_inputs);

    // ── Truth table manipulation ─────────────────────────────────────────

    // Negate input variable i in truth table
    static TruthTable6 negate_input(TruthTable6 tt, int var, int num_inputs);

    // Swap input variables i and j
    static TruthTable6 swap_inputs(TruthTable6 tt, int i, int j, int num_inputs);

    // Negate output
    static TruthTable6 negate_output(TruthTable6 tt, int num_inputs) {
        uint64_t mask = (num_inputs < 6) ? ((1ULL << (1 << num_inputs)) - 1) : ~0ULL;
        return (~tt) & mask;
    }

    // Cofactor: set variable to 0 or 1
    static TruthTable6 cofactor(TruthTable6 tt, int var, bool value, int num_inputs);

    // Count ones in truth table
    static int popcount(TruthTable6 tt, int num_inputs) {
        uint64_t mask = (num_inputs < 6) ? ((1ULL << (1 << num_inputs)) - 1) : ~0ULL;
        return __builtin_popcountll(tt & mask);
    }

    // Cofactor signature: (popcount(f|xi=0), popcount(f|xi=1)) for each input
    struct CofactorSig {
        int count0;  // popcount of negative cofactor
        int count1;  // popcount of positive cofactor
    };
    static std::vector<CofactorSig> cofactor_signatures(TruthTable6 tt, int num_inputs);

    // ── NPN cell library database ────────────────────────────────────────

    // Build NPN database from cell library
    // For each cell, computes truth table -> NPN canonical -> store mapping
    void build_library(const std::vector<NpnCellEntry>& cells);

    // Look up a truth table in the NPN database
    // Returns matching cell entries (may be multiple cells with same NPN class)
    struct MatchResult {
        const NpnCellEntry* cell = nullptr;
        NpnCanonical transform;    // how to transform cell to match target
        double area = 0;
        double delay = 0;
    };
    std::vector<MatchResult> lookup(TruthTable6 tt, int num_inputs) const;

    // Best match by area or delay
    MatchResult best_match_area(TruthTable6 tt, int num_inputs) const;
    MatchResult best_match_delay(TruthTable6 tt, int num_inputs) const;

    // Statistics
    int num_npn_classes() const { return (int)npn_db_.size(); }
    int num_cells_mapped() const { return (int)all_cells_.size(); }

private:
    // NPN canonical TT -> list of cells that implement this class
    std::unordered_map<TruthTable6, std::vector<NpnCellEntry>> npn_db_;
    std::vector<NpnCellEntry> all_cells_;

    // Precomputed NPN masks for swap operations
    static constexpr uint64_t swap_mask_[6][6] = {};
};

// ── 6-input AIG truth table computation ──────────────────────────────────

// Compute a 6-input truth table from an AIG subgraph
// root: AIG variable to evaluate
// leaves: input variables (up to 6)
// Returns: TruthTable6 where bit i = f(x5..x0) for minterm i
class AigGraph;  // forward decl
TruthTable6 compute_aig_tt6(const AigGraph& aig, uint32_t root,
                             const std::vector<uint32_t>& leaves);

} // namespace sf
