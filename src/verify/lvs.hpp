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

private:
    const Netlist& schem_;
    const PhysicalDesign& layout_;

    // Extract connectivity from layout
    struct LayoutNet {
        std::string name;
        std::vector<int> cell_ids;
    };
};

} // namespace sf
