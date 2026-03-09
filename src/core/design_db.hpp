#pragma once
// SiliconForge — Design Database
// Unified design state manager: ties together netlist, physical design,
// constraints, timing results, and verification state into a single object.
// Provides save/load and design snapshot capabilities.

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "frontend/sdc_parser.hpp"
#include "timing/sta.hpp"
#include "timing/power.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <sstream>

namespace sf {

struct DesignState {
    enum Stage {
        INIT, RTL_PARSED, SYNTHESIZED, PLACED, ROUTED, VERIFIED, SIGNED_OFF
    } stage = INIT;

    std::string design_name;
    Netlist netlist;
    PhysicalDesign physical;
    SdcConstraints constraints;
    StaResult timing;
    PowerResult power;

    // Metadata
    std::string technology;
    double target_freq_mhz = 0;
    int version = 0;
    std::vector<std::string> log;
};

struct DesignDbStats {
    int designs = 0;
    int total_gates = 0;
    int total_nets = 0;
    int total_cells = 0;
    std::string message;
};

class DesignDatabase {
public:
    // Create / manage designs
    void create(const std::string& name, const std::string& tech = "generic");
    DesignState& get(const std::string& name);
    const DesignState& get(const std::string& name) const;
    bool exists(const std::string& name) const;
    void remove(const std::string& name);

    // Advance stage
    void set_stage(const std::string& name, DesignState::Stage stage);

    // Add log entry
    void log(const std::string& name, const std::string& msg);

    // Snapshot — save current state as version
    int snapshot(const std::string& name);

    // Stats
    DesignDbStats stats() const;

    // List all designs
    std::vector<std::string> list() const;

    // Export design state summary
    std::string summary(const std::string& name) const;

private:
    std::unordered_map<std::string, DesignState> designs_;
    static DesignState dummy_; // for const get fallback
};

} // namespace sf
