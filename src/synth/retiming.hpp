#pragma once
// SiliconForge — Register Retiming via Leiserson-Saxe Graph Optimization
// Mathematically shifts Flip-Flops across combinational logic to minimize clock period (maximize Fmax)

#include "core/netlist.hpp"
#include <vector>
#include <map>

namespace sf {

class RetimingEngine {
public:
    // Analyzes the Netlist and repositions DFFs to balance combinational delays
    // Returns true if the graph was optimized successfully.
    bool optimize(Netlist& nl);

private:
    // Leiserson-Saxe Graph representation
    struct RetimeNode {
        GateId gid;
        double delay;
        int r_val; // Retiming variable r(v): number of registers pushed BACKWARDS across the node
    };
    
    struct RetimeEdge {
        GateId u;     // Source
        GateId v;     // Destination
        int weight;   // Number of registers on the edge (typically 0 or 1)
        double delay; // Delay of the path
    };

    std::vector<RetimeNode> nodes_;
    std::vector<RetimeEdge> edges_;
    
    void build_graph(const Netlist& nl);
    bool compute_retiming_values();
    void apply_retiming(Netlist& nl);
    
    // Estimates the combinational delay of a gate type (in arbitrary time units)
    double get_gate_delay(GateType type);
};

} // namespace sf
