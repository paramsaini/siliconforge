#pragma once
// SiliconForge — SVA Formal Engine
// Translates parsed SVA properties into assertions inside the gate-level Netlist.

#include "core/netlist.hpp"
#include "frontend/sva_parser.hpp"
#include <vector>
#include <string>

namespace sf {

struct SvaMonitorResult {
    int properties_synthesized = 0;
    int assert_monitors = 0;
    int cover_monitors = 0;
    int assume_monitors = 0;
    int delay_registers = 0;
    int logic_gates = 0;
    std::vector<std::string> monitor_outputs;
    std::string report;
};

class SvaEngine {
public:
    // Legacy entry point — backward compatible
    static void synthesize_assertions(Netlist& nl, const std::vector<SvaProperty>& props);

    // Enhanced: returns detailed monitor synthesis metrics
    static SvaMonitorResult synthesize_enhanced(Netlist& nl,
                                                const std::vector<SvaProperty>& props);

private:
    static NetId find_net_by_name(const Netlist& nl, const std::string& name);
    static NetId synthesize_node(Netlist& nl, const SvaNode* node, NetId clk,
                                 const std::string& prefix, int& counter);
};

} // namespace sf
