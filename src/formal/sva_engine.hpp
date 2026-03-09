#pragma once
// SiliconForge — SVA Formal Engine
// Translates parsed SVA properties into assertions inside the gate-level Netlist.

#include "core/netlist.hpp"
#include "frontend/sva_parser.hpp"
#include <vector>
#include <string>

namespace sf {

class SvaEngine {
public:
    // Takes an existing netlist and a list of parsed SVA properties.
    // Synthesizes the properties as monitor logic into the netlist.
    // Creates a new output pin for each property starting with "sva_fail_".
    // A value of 1 on these outputs indicates a property violation.
    static void synthesize_assertions(Netlist& nl, const std::vector<SvaProperty>& props);

private:
    static NetId find_net_by_name(const Netlist& nl, const std::string& name);
};

} // namespace sf
