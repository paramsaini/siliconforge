#pragma once
// SiliconForge — Layout vs Schematic (LVS) Checker
// Compares physical layout netlist against schematic (logical) netlist.
// Reference: IC Design LVS — Mentor Calibre approach

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct LvsResult {
    bool match = false;
    int schematic_cells = 0;
    int layout_cells = 0;
    int matched_cells = 0;
    int unmatched_schematic = 0;
    int unmatched_layout = 0;
    int net_mismatches = 0;
    double time_ms = 0;
    std::string message;

    struct Mismatch {
        std::string type;  // "missing_cell", "extra_cell", "net_mismatch"
        std::string detail;
    };
    std::vector<Mismatch> mismatches;
};

class LvsChecker {
public:
    LvsChecker(const Netlist& schematic, const PhysicalDesign& layout)
        : schem_(schematic), layout_(layout) {}

    LvsResult check();

    // Device extraction from layout
    struct ExtractedDevice {
        enum Type { MOSFET, RESISTOR, CAPACITOR, DIODE } type;
        std::string name;
        int layer;              // gate layer for MOS, body layer for R/C
        double width, length;   // W/L for MOS, dimensions for R/C
        std::string drain, gate, source, bulk;  // terminal nets
    };
    struct ExtractionResult {
        std::vector<ExtractedDevice> devices;
        int mos_count = 0;
        int res_count = 0;
        int cap_count = 0;
    };
    ExtractionResult extract_devices();

    // Subcircuit matching (graph isomorphism)
    struct SubcircuitMatch {
        std::string schematic_name;
        std::string layout_name;
        bool matched = false;
        std::vector<std::pair<std::string,std::string>> pin_mapping;
        std::string mismatch_reason;
    };
    std::vector<SubcircuitMatch> match_subcircuits();

    // Net tracing through vias
    struct NetTrace {
        int net_idx;
        std::string net_name;
        std::vector<int> layers_visited;
        std::vector<std::pair<double,double>> via_locations;
        bool connected;     // full connectivity verified
    };
    std::vector<NetTrace> trace_all_nets();
    NetTrace trace_net(int net_idx);

    // Pin swapping tolerance
    struct PinSwapGroup {
        std::string cell_type;
        std::vector<std::vector<std::string>> swappable_pins;  // groups of interchangeable pins
    };
    void add_pin_swap_group(const PinSwapGroup& group);

    // Cross-hierarchy comparison
    struct HierarchyLevel {
        std::string instance_name;
        std::string cell_name;
        std::vector<int> child_indices;
        int parent = -1;
    };
    struct HierarchyCompare {
        std::vector<HierarchyLevel> schematic_hier;
        std::vector<HierarchyLevel> layout_hier;
        bool hierarchies_match = false;
        std::vector<std::string> mismatches;
    };
    HierarchyCompare compare_hierarchy();

    // LVS debug reporting
    struct LvsDebugReport {
        std::vector<std::string> unmatched_schematic_nets;
        std::vector<std::string> unmatched_layout_nets;
        std::vector<std::string> unmatched_schematic_cells;
        std::vector<std::string> unmatched_layout_cells;
        std::vector<std::pair<std::string,std::string>> net_name_suggestions;  // layout_net -> likely_schematic_net
        std::string formatted_report;
    };
    LvsDebugReport generate_debug_report();

    // Enhanced LVS run
    LvsResult run_enhanced();

private:
    const Netlist& schem_;
    const PhysicalDesign& layout_;

    // Extract connectivity from layout
    struct LayoutNet {
        std::string name;
        std::vector<int> cell_ids;
    };

    std::vector<PinSwapGroup> pin_swap_groups_;

    // Graph-based matching helpers
    struct NetGraph {
        std::vector<std::string> nodes;   // cell names
        std::vector<std::string> edges;   // net names
        std::vector<std::vector<int>> adjacency;
    };
    NetGraph build_schematic_graph();
    NetGraph build_layout_graph();
    bool graphs_isomorphic(const NetGraph& a, const NetGraph& b,
                           std::vector<int>& mapping);

    // Net name correlation (fuzzy matching)
    double net_similarity(const std::string& a, const std::string& b) const;
};

} // namespace sf
