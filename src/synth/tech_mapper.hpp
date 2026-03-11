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
    TechMapper(const AigGraph& aig, const LibertyLibrary& lib)
        : aig_(aig), lib_(lib) {}

    // Map AIG to gate-level netlist using cells from the library
    Netlist map(bool optimize_area = true);

    const MapStats& stats() const { return stats_; }

private:
    const AigGraph& aig_;
    const LibertyLibrary& lib_;
    MapStats stats_;

    // Simple structural matching: match AIG subgraph patterns to cells
    struct CellMatch {
        const LibertyCell* cell;
        std::vector<AigLit> inputs; // mapped AIG inputs to cell inputs
        AigLit output;
    };

    // Match cells to AIG nodes
    CellMatch match_node(uint32_t var);

    // Build netlist from matches
    Netlist build_netlist(const std::vector<CellMatch>& matches);

    // Post-mapping buffer insertion for high-fanout nets
    void insert_buffers(Netlist& nl, int max_fanout = 8);
};

} // namespace sf
