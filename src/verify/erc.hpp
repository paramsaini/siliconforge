#pragma once
// SiliconForge — Electrical Rule Checking (ERC)
// Checks for floating inputs, unconnected outputs, multi-driven nets,
// missing tie-off, power/ground connectivity.
// Reference: Cadence Voltus ERC, Mentor Calibre ERC methodology

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct ErcViolation {
    enum Type {
        FLOATING_INPUT, UNCONNECTED_OUTPUT, MULTI_DRIVEN,
        MISSING_TIEOFF, SHORTED_NETS, MISSING_POWER, MISSING_GROUND,
        FLOATING_GATE, TRISTATE_CONFLICT
    };
    Type type;
    std::string net_name;
    std::string message;
};

struct ErcResult {
    int total_checks = 0;
    int violations   = 0;
    int warnings     = 0;
    std::vector<ErcViolation> details;
    bool clean = false;
    std::string message;
};

class ErcEngine {
public:
    ErcEngine(const Netlist& nl, const PhysicalDesign& pd)
        : nl_(nl), pd_(pd) {}
    ErcResult run();

private:
    const Netlist& nl_;
    const PhysicalDesign& pd_;
    void check_floating_inputs(ErcResult& res);
    void check_unconnected_outputs(ErcResult& res);
    void check_multi_driven(ErcResult& res);
    void check_missing_tieoff(ErcResult& res);
    void check_power_ground(ErcResult& res);
};

} // namespace sf
