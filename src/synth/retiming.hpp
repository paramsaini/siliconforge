#pragma once
// SiliconForge — Register Retiming via Leiserson-Saxe Algorithm
// Computes optimal DFF placement to minimize clock period (maximize Fmax).
// Reference: Leiserson & Saxe, "Retiming Synchronous Circuitry", Algorithmica 1991
// Reference: Pan, "An Efficient Retiming Algorithm Under Setup and Hold
//            Constraints", DAC 1999

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace sf {

// Industrial retiming configuration
// Reference: Synopsys Design Compiler optimize_registers, Cadence RTL Compiler
struct RetimingConfig {
    // Constraint modes
    bool preserve_clock_gating = true;   // don't retime through ICG cells
    bool preserve_reset_paths = true;    // don't retime across async reset boundaries
    bool honor_multicycle_paths = true;  // respect multi-cycle path exceptions
    bool hold_aware = true;              // consider hold time in retiming constraints

    // Resource constraints
    int max_register_increase = -1;      // -1 = unlimited; else max DFFs that can be added
    double area_budget = -1;             // -1 = unlimited; else max area increase (um²)

    // Timing
    double hold_margin = 0.05;           // hold time margin (ns) — prevents short-path violations
    double setup_margin = 0.0;           // setup time margin (ns) — extra pessimism

    // Liberty integration
    bool use_liberty_delays = true;      // use real cell delays from Liberty lib

    // Optimization control
    int binary_search_steps = 12;        // precision of period search
    double improvement_threshold = 0.01; // min 1% improvement to apply retiming
};

// Multi-cycle path exception for retiming
struct MulticyclePath {
    GateId from_gate = -1;    // source gate (-1 = any)
    GateId to_gate = -1;      // destination gate (-1 = any)
    int cycles = 2;           // number of clock cycles allowed
};

struct RetimingResult {
    double critical_path_before = 0;
    double critical_path_after = 0;
    int registers_moved = 0;
    int dffs_inserted = 0;
    int dffs_removed = 0;
    bool improved = false;
    std::string message;

    // Industrial metrics
    int clock_gating_preserved = 0;    // ICG cells skipped
    int reset_paths_preserved = 0;     // reset-connected gates skipped
    int multicycle_paths_honored = 0;  // MCP exceptions applied
    int register_count_before = 0;     // DFF count before retiming
    int register_count_after = 0;      // DFF count after retiming
    double area_before = 0;            // estimated area before
    double area_after = 0;             // estimated area after
    double fmax_before = 0;            // 1/critical_path_before (GHz)
    double fmax_after = 0;             // 1/critical_path_after (GHz)
    double fmax_improvement_pct = 0;   // percentage Fmax improvement
    int iterations = 0;                // binary search iterations used
};

class RetimingEngine {
public:
    // Analyzes the Netlist and repositions DFFs to minimize clock period.
    // Returns true if the netlist was modified.
    bool optimize(Netlist& nl);

    // Full optimize with result reporting
    RetimingResult optimize_with_result(Netlist& nl);

    // Industrial configuration
    void set_config(const RetimingConfig& cfg) { config_ = cfg; }
    RetimingConfig& config() { return config_; }

    // Liberty library for accurate cell delays
    void set_liberty(const LibertyLibrary* lib) { lib_ = lib; }

    // Constraint API
    void add_dont_retime(GateId gid) { dont_retime_.insert(gid); }
    void clear_dont_retime() { dont_retime_.clear(); }
    void add_multicycle_path(const MulticyclePath& mcp) { multicycle_paths_.push_back(mcp); }
    void clear_multicycle_paths() { multicycle_paths_.clear(); }

    // Mark clock gating cells (ICGs) — will be auto-detected if not set
    void add_clock_gating_cell(GateId gid) { clock_gating_cells_.insert(gid); }

private:
    RetimingConfig config_;
    const LibertyLibrary* lib_ = nullptr;
    std::unordered_set<GateId> dont_retime_;
    std::unordered_set<GateId> clock_gating_cells_;
    std::vector<MulticyclePath> multicycle_paths_;

    // Leiserson-Saxe graph representation
    struct RetimeNode {
        GateId gid;
        double delay;
        int r_val = 0; // retiming label: registers moved backward through node
        bool frozen = false; // industrial: can't be retimed
    };

    struct RetimeEdge {
        int u, v;       // indices into nodes_
        int weight;     // register count on edge (0 = combinational, ≥1 = through DFF)
        double delay;   // combinational delay of this edge's path
        int multicycle_factor = 1; // MCP relaxation factor
    };

    std::vector<RetimeNode> nodes_;
    std::vector<RetimeEdge> edges_;
    std::unordered_map<GateId, int> gid_to_idx_;

    // Graph construction
    void build_graph(const Netlist& nl);
    double get_gate_delay(GateType type) const;
    double get_liberty_delay(GateType type) const;

    // Industrial: detect and freeze clock gating / reset cells
    void detect_clock_gating(const Netlist& nl);
    void detect_reset_paths(const Netlist& nl);
    void apply_multicycle_relaxation();
    void freeze_dont_retime_cells();

    // Critical path analysis (longest zero-weight path via Bellman-Ford)
    double compute_critical_path() const;

    // Leiserson-Saxe retiming: constraint-graph + Bellman-Ford
    // Returns true if a valid retiming with shorter clock period was found.
    bool compute_retiming_values(double target_period);

    // Structural netlist modification: insert/remove DFFs per r-values
    int apply_retiming(Netlist& nl);

    // Industrial: validate retiming against resource constraints
    bool validate_resource_constraints() const;
};

} // namespace sf
