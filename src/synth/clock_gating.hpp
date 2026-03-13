#pragma once
// SiliconForge — Clock Gating Insertion
// Automatically inserts Integrated Clock Gating (ICG) cells
// to reduce dynamic power by gating clock to idle flip-flops.
// Reference: Benini & De Micheli, "Dynamic Power Management", Springer 1998

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

namespace sf {

struct ClockGatingResult {
    int total_ffs = 0;
    int gated_ffs = 0;
    int icg_cells_inserted = 0;
    double power_reduction_pct = 0;
    double time_ms = 0;

    struct IcgCell {
        std::string name;
        NetId clk_in;
        NetId enable;
        NetId clk_out;
        std::vector<GateId> gated_ffs;
    };
    std::vector<IcgCell> cells;
    std::string message;
};

class ClockGatingEngine {
public:
    explicit ClockGatingEngine(Netlist& nl) : nl_(nl) {}

    // Set minimum group size for gating (don't gate single FFs)
    void set_min_group(int n) { min_group_ = n; }

    // Set assumed activity factor (lower = more savings)
    void set_activity(double act) { activity_ = act; }

    ClockGatingResult insert();

    // ── Multi-level clock gating ────────────────────────────────────────────
    struct ClockTree {
        NetId root_clock;
        struct GatingLevel {
            int level;              // 0 = top, 1 = sub-group, etc.
            NetId enable;
            NetId gated_clock;
            std::vector<GateId> ffs;
            std::vector<int> children; // indices of sub-levels
        };
        std::vector<GatingLevel> levels;
    };

    // ── Activity-based analysis ─────────────────────────────────────────────
    struct ActivityProfile {
        std::unordered_map<NetId, double> signal_activity; // switching activity per net
        double default_activity = 0.1;
    };

    // ── Configuration ───────────────────────────────────────────────────────
    struct ClockGatingConfig {
        int min_group = 1;
        double activity_threshold = 0.3; // don't gate if activity > 30%
        bool enable_multi_level = true;
        bool enable_fine_grain = true;   // per-register gating
        int max_levels = 3;
        double target_power_reduction = 0.0; // 0 = maximize
    };

    void set_config(const ClockGatingConfig& cfg) { cfg_ = cfg; }
    void set_activity(const ActivityProfile& ap) { activity_profile_ = ap; }

    // Enhanced insertion with multi-level support
    ClockGatingResult insert_hierarchical();

    // ── Fine-grain: per-register enable analysis ────────────────────────────
    struct RegisterEnableInfo {
        GateId ff_id;
        NetId data_net;
        NetId enable_condition; // -1 if always active
        double estimated_activity;
        bool worth_gating;      // activity below threshold
    };
    std::vector<RegisterEnableInfo> analyze_register_enables();

    // ── Power estimation for gating decisions ───────────────────────────────
    struct PowerEstimate {
        double ungated_power;    // mW
        double gated_power;      // mW
        double icg_overhead;     // mW (ICG cell leakage + switching)
        double net_savings;      // mW
        bool profitable;         // net_savings > 0
    };
    PowerEstimate estimate_gating_power(const std::vector<GateId>& ffs, double activity);

    // ── ICG cell selection from library ─────────────────────────────────────
    struct IcgCellType {
        std::string name;
        double area;
        double leakage_power;
        double clk_to_q_delay;
        int max_fanout;
    };
    static std::vector<IcgCellType> get_icg_library();
    IcgCellType select_icg_cell(int fanout);

private:
    Netlist& nl_;
    int min_group_ = 1;
    double activity_ = 0.1; // 10% default switching
    ClockGatingConfig cfg_;
    ActivityProfile activity_profile_;

    // Find groups of FFs sharing the same enable condition
    struct FfGroup {
        NetId clock;
        NetId enable; // -1 if no enable
        std::vector<GateId> ffs;
    };
    std::vector<FfGroup> find_groups();

    // Multi-level tree building
    ClockTree build_clock_tree(NetId root_clk, const std::vector<GateId>& ffs);
    void partition_ffs(const std::vector<GateId>& ffs,
                       std::vector<std::vector<GateId>>& groups);

    // Improved enable condition extraction
    NetId extract_enable_deep(GateId ff_id);
};

// ── Phase C: Clock Gating Verification ──────────────────────────────────────

struct ClockGateVerifyResult {
    int total_icg_cells = 0;
    int latch_based = 0;
    int and_gate_based = 0;      // glitch-prone
    int enable_timing_ok = 0;
    int enable_timing_fail = 0;
    int glitch_risk = 0;
    bool clean = true;
    std::string message;

    struct Issue {
        std::string icg_name;
        std::string type;        // "AND_GATE_GATING", "ENABLE_TIMING", "GLITCH_RISK"
        std::string detail;
    };
    std::vector<Issue> issues;
};

class ClockGateVerifier {
public:
    explicit ClockGateVerifier(const Netlist& nl) : nl_(nl) {}

    ClockGateVerifyResult verify();

private:
    const Netlist& nl_;

    // Check that enable is stable during active clock edge
    bool check_enable_timing(GateId icg_gate) const;

    // Check for potential glitches on gated clock output
    bool check_glitch_free(GateId icg_gate) const;

    // Verify latch-based gating (not bare AND gate)
    bool is_latch_based(GateId icg_gate) const;
};

} // namespace sf
