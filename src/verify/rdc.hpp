#pragma once
// SiliconForge — Reset Domain Crossing (RDC) Analyzer
// Detects signals crossing between reset domains and checks for synchronizers.
// Reference: Cummings, "Correct Methods for Adding Delays to Verilog Models", SNUG 2008

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>

namespace sf {

struct RdcViolation {
    enum Type { ASYNC_RESET_CROSSING, MISSING_SYNCHRONIZER, GLITCH_PRONE };
    Type type;
    std::string source_domain;
    std::string dest_domain;
    std::string path;
    std::string message;
};

struct RdcResult {
    int reset_domains = 0;
    int crossings = 0;
    int violations = 0;
    int warnings = 0;
    std::vector<RdcViolation> details;
    bool clean = false;
    std::string message;
};

class RdcAnalyzer {
public:
    RdcAnalyzer(const Netlist& nl) : nl_(nl) {}
    RdcResult analyze();

private:
    const Netlist& nl_;
    std::unordered_map<int, std::string> detect_reset_domains();
};

} // namespace sf
