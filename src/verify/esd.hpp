#pragma once
// SiliconForge — ESD (Electrostatic Discharge) Verification
// Checks IO pad protection, power clamps, and discharge paths.
// Reference: ESD Association STM5.5.1, Voldman "ESD: Physics and Devices", Wiley 2004

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>

namespace sf {

struct EsdViolation {
    enum Type { MISSING_CLAMP, NO_DISCHARGE_PATH, HIGH_RESISTANCE_PATH,
                MISSING_SECONDARY, NO_POWER_CLAMP };
    Type type;
    std::string pad_name;
    std::string message;
};

struct EsdResult {
    int total_pads = 0;
    int protected_pads = 0;
    int violations = 0;
    int warnings = 0;
    std::vector<EsdViolation> details;
    bool clean = false;
    double worst_resistance_ohm = 0;
    std::string message;
};

class EsdChecker {
public:
    EsdChecker(const Netlist& nl, const PhysicalDesign& pd) : nl_(nl), pd_(pd) {}
    EsdResult check();

private:
    const Netlist& nl_;
    const PhysicalDesign& pd_;
    void check_io_clamps(EsdResult& res);
    void check_power_clamps(EsdResult& res);
    void check_discharge_paths(EsdResult& res);
};

} // namespace sf
