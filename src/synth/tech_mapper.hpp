#pragma once
// SiliconForge — Technology Mapper
// Maps AIG to standard cells from a Liberty library.
// Uses cut-enumeration and area/delay-optimal covering.
// Reference: Cong & Ding, "FlowMap", IEEE TCAD, 1994

#include "core/aig.hpp"
#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>

namespace sf {

struct MapStats {
    uint32_t num_cells = 0;
    double total_area = 0.0;
    uint32_t depth = 0;
    double time_ms = 0.0;
};

class TechMapper {
public:
    // Mapping modes
    enum class MapGoal { AREA, DELAY, POWER };

    TechMapper(const AigGraph& aig, const LibertyLibrary& lib)
        : aig_(aig), lib_(lib) {}

    // Set mapping objective
    void set_goal(MapGoal goal) { goal_ = goal; }

    // Map AIG to gate-level netlist using cells from the library
    Netlist map(bool optimize_area = true);

    const MapStats& stats() const { return stats_; }

    // Load-dependent delay modeling
    struct LoadModel {
        double input_cap = 0.01;       // pF
        double load_cap = 0.0;         // computed from fanout
        double intrinsic_delay = 0.1;  // ns
        double transition_delay = 0.05; // ns/pF
    };

private:
    const AigGraph& aig_;
    const LibertyLibrary& lib_;
    MapStats stats_;
    MapGoal goal_ = MapGoal::DELAY;

    // Simple structural matching: match AIG subgraph patterns to cells
    struct CellMatch {
        const LibertyCell* cell;
        std::vector<AigLit> inputs; // mapped AIG inputs to cell inputs
        AigLit output;
    };

    // Extended cell matching with complex gates
    struct ExtendedMatch {
        const LibertyCell* cell = nullptr;
        std::vector<AigLit> inputs;
        AigLit output = AIG_FALSE;
        double area = 0.0;
        double delay = 0.0;       // load-dependent
        double power = 0.0;       // estimated switching power
        int priority = 999;       // match quality (lower = better)
    };

    // Match cells to AIG nodes (original fallback)
    CellMatch match_node(uint32_t var);

    // Build netlist from matches
    Netlist build_netlist(const std::vector<CellMatch>& matches);

    // Post-mapping buffer insertion for high-fanout nets
    void insert_buffers(Netlist& nl, int max_fanout = 8);

    // Complex gate pattern recognition
    ExtendedMatch match_aoi21(uint32_t var);    // AND-OR-INVERT: !(a&b | c)
    ExtendedMatch match_oai21(uint32_t var);    // OR-AND-INVERT: !((a|b) & c)
    ExtendedMatch match_aoi22(uint32_t var);    // !(a&b | c&d)
    ExtendedMatch match_mux2(uint32_t var);     // MUX: s?a:b
    ExtendedMatch match_xor2(uint32_t var);     // XOR: a^b
    ExtendedMatch match_xnor2(uint32_t var);    // XNOR: !(a^b)
    ExtendedMatch match_nand3(uint32_t var);    // 3-input NAND
    ExtendedMatch match_nor3(uint32_t var);     // 3-input NOR
    ExtendedMatch match_and3(uint32_t var);     // 3-input AND
    ExtendedMatch match_or3(uint32_t var);      // 3-input OR
    ExtendedMatch match_majority(uint32_t var); // MAJ(a,b,c)

    // Multi-match selection (Pareto-optimal)
    std::vector<ExtendedMatch> find_all_matches(uint32_t var);
    ExtendedMatch select_best_match(const std::vector<ExtendedMatch>& matches);

    // Load-aware delay computation
    double compute_load_delay(const LibertyCell* cell, double load_cap) const;

    // Technology mapping with area recovery
    void area_recovery_pass(Netlist& nl, const std::vector<CellMatch>& matches);

    // Generic 3-input matcher using truth table comparison
    ExtendedMatch match_3input_generic(uint32_t var, uint16_t target_tt,
                                       const std::string& name_pattern);

    // Helper: find a library cell by name substring (case-insensitive)
    const LibertyCell* find_cell_by_pattern(const std::string& pattern) const;

    // Helper: find cells matching a truth table with given input count
    const LibertyCell* find_cell_by_tt(uint16_t target_tt, int num_inputs) const;

    // Helper: compute fanout count for an AIG variable
    int compute_fanout(uint32_t var) const;

    // Helper: convert ExtendedMatch to CellMatch
    CellMatch to_cell_match(const ExtendedMatch& em) const;

    // ── Tier 2: ILP-style optimal tech mapping ──────────────────────────
    // Dynamic programming optimal covering that considers area+delay tradeoff.
    // Approximates ILP by evaluating all match candidates per node and
    // selecting the globally optimal combination via Lagrangian relaxation.
public:
    struct IlpMapConfig {
        double area_weight = 0.5;
        double delay_weight = 0.5;
        int lagrangian_iterations = 10;
        double lambda_step = 0.1;
    };
    Netlist map_optimal(const IlpMapConfig& cfg);
};

} // namespace sf
