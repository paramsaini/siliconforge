#pragma once
// SiliconForge — Multi-Corner Multi-Mode (MCMM) Analyzer
// Runs STA/power across multiple PVT corners and functional modes.
// Reference: Synopsys "MCMM Analysis", PrimeTime User Guide

#include "core/netlist.hpp"
#include "timing/sta.hpp"
#include "timing/power.hpp"
#include <string>
#include <vector>

namespace sf {

struct PvtCorner {
    std::string name;
    double voltage = 1.8;
    double temperature_c = 25;
    enum Process { FAST, TYPICAL, SLOW } process = TYPICAL;
    double delay_scale = 1.0;  // multiplier on gate delays
    double power_scale = 1.0;
};

struct FunctionalMode {
    std::string name;
    double clock_freq_mhz = 500;
    double switching_activity = 0.1;
    std::string description;
};

struct McmmScenario {
    PvtCorner corner;
    FunctionalMode mode;
    StaResult sta;
    PowerResult power;
};

struct McmmResult {
    int corners = 0;
    int modes = 0;
    int scenarios = 0;
    double worst_wns = 0;
    double worst_tns = 0;
    std::string worst_corner;
    double max_power_mw = 0;
    std::string max_power_scenario;
    std::vector<McmmScenario> details;
    double time_ms = 0;
    std::string message;
};

class McmmAnalyzer {
public:
    explicit McmmAnalyzer(const Netlist& nl) : nl_(nl) {}

    void add_corner(const PvtCorner& corner) { corners_.push_back(corner); }
    void add_mode(const FunctionalMode& mode) { modes_.push_back(mode); }

    // Load standard corners
    void load_default_corners();
    void load_default_modes(double base_freq_mhz = 500);

    McmmResult analyze();

private:
    const Netlist& nl_;
    std::vector<PvtCorner> corners_;
    std::vector<FunctionalMode> modes_;
};

} // namespace sf
