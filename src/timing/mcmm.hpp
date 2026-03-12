#pragma once
// SiliconForge — Multi-Corner Multi-Mode (MCMM) Analyzer
// Runs STA/power across N user-defined PVT corners × M functional modes.
// Supports per-corner CornerDerate, OCV/AOCV/POCV, per-mode clock constraints.
// Industrial: scenario classification, pruning, per-scenario constraints,
//             signoff reporting, active scenario sets, sensitivity analysis.
// Reference: Synopsys "MCMM Analysis", PrimeTime User Guide
// Reference: Cadence Tempus "Multi-Mode Multi-Corner Timing Analysis"

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include "timing/power.hpp"
#include <string>
#include <vector>
#include <array>
#include <functional>
#include <unordered_set>

namespace sf {

// Scenario signoff classification
// Reference: Cadence Tempus — each scenario has a signoff purpose
enum class SignoffType { SETUP, HOLD, LEAKAGE, DYNAMIC_POWER, TRANSITION, ALL };

inline const char* signoff_type_str(SignoffType t) {
    switch (t) {
        case SignoffType::SETUP: return "setup";
        case SignoffType::HOLD: return "hold";
        case SignoffType::LEAKAGE: return "leakage";
        case SignoffType::DYNAMIC_POWER: return "dynamic_power";
        case SignoffType::TRANSITION: return "transition";
        case SignoffType::ALL: return "all";
        default: return "all";
    }
}

// PVT corner: defines process/voltage/temperature + STA derating
struct PvtCorner {
    std::string name;
    double voltage = 1.8;
    double temperature_c = 25;
    enum Process { FAST, TYPICAL, SLOW } process = TYPICAL;

    // Proper corner derating (replaces crude delay_scale)
    CornerDerate derate;

    // OCV/AOCV/POCV per corner
    OcvMode ocv_mode = OcvMode::NONE;
    AocvTable aocv_table;
    PocvTable pocv_table;   // POCV per-cell sigma (industrial)

    // CPPR per corner
    bool cppr_enabled = false;

    // Power scaling
    double power_scale = 1.0;

    // Backward-compatible fields
    double delay_scale = 1.0;

    // Industrial: signoff type — what this corner is used for
    SignoffType signoff = SignoffType::ALL;
};

// Functional mode: different operating conditions (func/scan/standby)
struct FunctionalMode {
    std::string name;
    double clock_freq_mhz = 500;
    double clock_period_ns = 0;     // explicit period (overrides freq if >0)
    double switching_activity = 0.1;
    double setup_uncertainty = 0;   // per-mode setup uncertainty
    double hold_uncertainty = 0;    // per-mode hold uncertainty
    std::string description;
    SignoffType signoff = SignoffType::ALL; // mode-level signoff filter
};

// Per-scenario timing exception
struct ScenarioConstraint {
    std::string scenario_pattern; // glob pattern: "ss*/func" or "*"
    double clock_period_override = -1; // override clock for specific scenario
    double max_transition = -1;    // max slew constraint
    bool is_active = true;         // can disable a scenario
};

struct McmmScenario {
    PvtCorner corner;
    FunctionalMode mode;
    StaResult sta;
    PowerResult power;
    std::string scenario_name;
    bool active = true;            // industrial: can be disabled
    SignoffType signoff = SignoffType::ALL;
    bool dominated = false;        // industrial: flagged as dominated by another
};

// MCMM configuration for industrial analysis
struct McmmConfig {
    bool enable_scenario_pruning = false;  // remove dominated scenarios
    bool enable_pocv_per_corner = true;    // use POCV if corner has it
    bool enable_cppr_per_corner = true;    // use CPPR if corner has it
    bool enable_sensitivity = false;       // compute per-scenario sensitivity
    double pruning_slack_margin = 0.05;    // ns margin for pruning dominance check
    int max_paths_per_scenario = 10;       // STA path limit per scenario
};

// Signoff summary per category
struct SignoffSummary {
    std::string worst_scenario;
    double worst_wns = 0;
    double worst_tns = 0;
    int total_violations = 0;
    int scenario_count = 0;
};

struct McmmResult {
    int corners = 0;
    int modes = 0;
    int scenarios = 0;
    double worst_setup_wns = 0;
    double worst_setup_tns = 0;
    double worst_hold_wns = 0;
    double worst_hold_tns = 0;
    std::string worst_setup_scenario;
    std::string worst_hold_scenario;
    int total_setup_violations = 0;
    int total_hold_violations = 0;
    double max_power_mw = 0;
    std::string max_power_scenario;
    std::vector<McmmScenario> details;
    double time_ms = 0;
    std::string message;

    // Backward compatibility
    double worst_wns = 0;
    double worst_tns = 0;
    std::string worst_corner;

    // Industrial signoff summary
    SignoffSummary setup_signoff;
    SignoffSummary hold_signoff;
    SignoffSummary leakage_signoff;
    SignoffSummary power_signoff;

    // Industrial metrics
    int active_scenarios = 0;      // scenarios actually analyzed
    int pruned_scenarios = 0;      // scenarios skipped by pruning
    int cppr_scenarios = 0;        // scenarios using CPPR
    int pocv_scenarios = 0;        // scenarios using POCV

    // Per-scenario slack sensitivity (optional)
    // Maps scenario_name → (voltage_sensitivity, temp_sensitivity)
    struct Sensitivity { double voltage_sens = 0; double temp_sens = 0; };
    std::vector<std::pair<std::string, Sensitivity>> sensitivities;
};

class McmmAnalyzer {
public:
    McmmAnalyzer(const Netlist& nl,
                 const LibertyLibrary* lib = nullptr,
                 const PhysicalDesign* pd = nullptr)
        : nl_(nl), lib_(lib), pd_(pd) {}

    void add_corner(const PvtCorner& corner) { corners_.push_back(corner); }
    void add_mode(const FunctionalMode& mode) { modes_.push_back(mode); }
    void clear_corners() { corners_.clear(); }
    void clear_modes() { modes_.clear(); }

    // Load standard corners (3 default OR extended foundry set)
    void load_default_corners();
    void load_foundry_corners();   // 7 standard foundry corners
    void load_default_modes(double base_freq_mhz = 500);

    // Industrial configuration
    void set_config(const McmmConfig& cfg) { config_ = cfg; }
    McmmConfig& config() { return config_; }

    // Scenario constraints
    void add_scenario_constraint(const ScenarioConstraint& sc) {
        constraints_.push_back(sc);
    }

    // Active scenario set: only analyze specified scenarios
    void set_active_scenarios(const std::unordered_set<std::string>& active) {
        active_set_ = active;
        use_active_set_ = true;
    }
    void clear_active_set() { active_set_.clear(); use_active_set_ = false; }

    McmmResult analyze();

    // Industrial: get corners/modes for inspection
    const std::vector<PvtCorner>& corners() const { return corners_; }
    const std::vector<FunctionalMode>& modes() const { return modes_; }

private:
    const Netlist& nl_;
    const LibertyLibrary* lib_;
    const PhysicalDesign* pd_;
    std::vector<PvtCorner> corners_;
    std::vector<FunctionalMode> modes_;
    McmmConfig config_;
    std::vector<ScenarioConstraint> constraints_;
    std::unordered_set<std::string> active_set_;
    bool use_active_set_ = false;

    // Run STA for one scenario
    StaResult run_scenario_sta(const PvtCorner& corner, const FunctionalMode& mode);

    // Industrial: scenario pruning (remove dominated scenarios)
    void prune_dominated_scenarios(std::vector<McmmScenario>& scenarios);

    // Industrial: check if scenario is active
    bool is_scenario_active(const std::string& name) const;

    // Industrial: compute sensitivity (how WNS changes with V/T)
    McmmResult::Sensitivity compute_sensitivity(const PvtCorner& corner,
                                                 const FunctionalMode& mode,
                                                 double base_wns);
};

} // namespace sf
