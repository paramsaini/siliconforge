#pragma once
// SiliconForge — Lint Engine (Static Design Rule Checking)
// Analyzes gate-level netlist for common design issues WITHOUT simulation.
// Industrial: 30+ rules across 8 categories matching Synopsys SpyGlass / Cadence HAL.
// References:
//   Synopsys SpyGlass Lint Rule Reference
//   Cadence HAL — RTL Design Rule Checking
//   IEEE 1364-2005 / IEEE 1800-2017 coding guidelines

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace sf {

// Lint rule categories (industrial classification)
enum class LintCategory {
    CONNECTIVITY,   // undriven, multi-driven, floating
    CLOCK,          // clock domain, gating, async reset
    TIMING,         // combinational loops, long chains, reconvergent fanout
    NAMING,         // naming conventions, reserved words
    STRUCTURE,      // fanout violations, high-fanin, constant propagation
    REDUNDANCY,     // dead logic, unreachable gates, duplicate gates
    POWER,          // clock gating opportunities, unnecessary toggling
    DFT             // scan chain issues, non-scannable FFs
};

inline const char* lint_category_str(LintCategory c) {
    switch (c) {
        case LintCategory::CONNECTIVITY: return "connectivity";
        case LintCategory::CLOCK: return "clock";
        case LintCategory::TIMING: return "timing";
        case LintCategory::NAMING: return "naming";
        case LintCategory::STRUCTURE: return "structure";
        case LintCategory::REDUNDANCY: return "redundancy";
        case LintCategory::POWER: return "power";
        case LintCategory::DFT: return "dft";
        default: return "unknown";
    }
}

struct LintViolation {
    enum Severity { INFO, WARNING, ERROR };
    Severity severity;
    std::string rule;
    std::string message;
    NetId net = -1;
    GateId gate = -1;
    LintCategory category = LintCategory::CONNECTIVITY;
};

// Lint configuration
struct LintConfig {
    // Category enables
    bool check_connectivity = true;
    bool check_clock = true;
    bool check_timing = true;
    bool check_naming = true;
    bool check_structure = true;
    bool check_redundancy = true;
    bool check_power = true;
    bool check_dft = true;

    // Thresholds
    int max_fanout = 32;              // LINT-S01: max fanout before warning
    int max_fanin = 8;                // LINT-S02: max fanin before warning
    int max_combo_depth = 20;         // LINT-T02: max combinational logic depth
    int min_name_length = 2;          // LINT-N01: minimum signal/gate name length

    // Naming conventions
    std::string clock_pattern = "clk"; // expected clock signal naming
    std::string reset_pattern = "rst"; // expected reset signal naming

    // Disabled rules
    std::unordered_set<std::string> disabled_rules;
};

// Lint summary
struct LintSummary {
    int total_violations = 0;
    int errors = 0;
    int warnings = 0;
    int infos = 0;
    std::unordered_map<std::string, int> violations_by_rule;
    std::unordered_map<LintCategory, int> violations_by_category;
    int rules_checked = 0;
};

class LintEngine {
public:
    explicit LintEngine(const Netlist& nl) : nl_(nl) {}

    // Industrial: configurable lint
    void set_config(const LintConfig& cfg) { config_ = cfg; }
    LintConfig& config() { return config_; }

    std::vector<LintViolation> run_all();
    LintSummary summarize(const std::vector<LintViolation>& violations) const;

    // ── Category: CONNECTIVITY (LINT-001 to LINT-006) ──
    std::vector<LintViolation> check_undriven_nets();       // LINT-001
    std::vector<LintViolation> check_multi_driven_nets();   // LINT-002
    std::vector<LintViolation> check_floating_outputs();    // LINT-003
    std::vector<LintViolation> check_combinational_loops(); // LINT-004
    std::vector<LintViolation> check_unconnected_inputs();  // LINT-005
    std::vector<LintViolation> check_constant_outputs();    // LINT-006

    // ── Category: CLOCK (LINT-C01 to LINT-C05) ──
    std::vector<LintViolation> check_multiple_clocks();     // LINT-C01: DFFs driven by different clocks
    std::vector<LintViolation> check_gated_clocks();        // LINT-C02: AND/OR gates on clock paths
    std::vector<LintViolation> check_async_reset_usage();   // LINT-C03: async reset consistency
    std::vector<LintViolation> check_clock_as_data();       // LINT-C04: clock net used as data
    std::vector<LintViolation> check_data_as_clock();       // LINT-C05: data net used as clock

    // ── Category: TIMING (LINT-T01 to LINT-T03) ──
    std::vector<LintViolation> check_reconvergent_fanout(); // LINT-T01: reconvergent paths
    std::vector<LintViolation> check_combo_depth();         // LINT-T02: combinational depth exceeds limit
    std::vector<LintViolation> check_latch_inference();     // LINT-T03: transparent latch detected

    // ── Category: NAMING (LINT-N01 to LINT-N03) ──
    std::vector<LintViolation> check_short_names();         // LINT-N01: names too short
    std::vector<LintViolation> check_naming_conventions();  // LINT-N02: clock/reset naming
    std::vector<LintViolation> check_duplicate_names();     // LINT-N03: duplicate gate names

    // ── Category: STRUCTURE (LINT-S01 to LINT-S04) ──
    std::vector<LintViolation> check_high_fanout();         // LINT-S01: fanout exceeds limit
    std::vector<LintViolation> check_high_fanin();          // LINT-S02: fanin exceeds limit
    std::vector<LintViolation> check_tristate_bus();        // LINT-S03: multiple tristate drivers
    std::vector<LintViolation> check_dangling_nets();       // LINT-S04: nets with no driver and no fanout

    // ── Category: REDUNDANCY (LINT-R01 to LINT-R03) ──
    std::vector<LintViolation> check_dead_logic();          // LINT-R01: gates with no path to output
    std::vector<LintViolation> check_redundant_inverters(); // LINT-R02: double inversion
    std::vector<LintViolation> check_constant_propagation();// LINT-R03: gates with all-constant inputs

    // ── Category: POWER (LINT-P01 to LINT-P02) ──
    std::vector<LintViolation> check_clock_gating_opportunity(); // LINT-P01
    std::vector<LintViolation> check_unnecessary_buffers(); // LINT-P02

    // ── Category: DFT (LINT-D01 to LINT-D02) ──
    std::vector<LintViolation> check_non_scannable_ffs();   // LINT-D01: DFFs without scan mux
    std::vector<LintViolation> check_dff_reset_missing();   // LINT-D02: DFFs without reset

private:
    const Netlist& nl_;
    LintConfig config_;

    bool is_rule_enabled(const std::string& rule) const {
        return config_.disabled_rules.find(rule) == config_.disabled_rules.end();
    }
};

} // namespace sf
