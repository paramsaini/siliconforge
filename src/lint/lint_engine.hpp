#pragma once
// SiliconForge — Lint Engine (Static Design Rule Checking)
// Analyzes gate-level netlist for common design issues WITHOUT simulation.

#include "core/netlist.hpp"
#include <string>
#include <vector>

namespace sf {

struct LintViolation {
    enum Severity { INFO, WARNING, ERROR };
    Severity severity;
    std::string rule;
    std::string message;
    NetId net = -1;
    GateId gate = -1;
};

class LintEngine {
public:
    explicit LintEngine(const Netlist& nl) : nl_(nl) {}

    std::vector<LintViolation> run_all();

    // Individual rules
    std::vector<LintViolation> check_undriven_nets();
    std::vector<LintViolation> check_multi_driven_nets();
    std::vector<LintViolation> check_floating_outputs();
    std::vector<LintViolation> check_combinational_loops();
    std::vector<LintViolation> check_unconnected_inputs();
    std::vector<LintViolation> check_constant_outputs();

private:
    const Netlist& nl_;
};

} // namespace sf
