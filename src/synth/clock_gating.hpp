#pragma once
// SiliconForge — Clock Gating Insertion
// Automatically inserts Integrated Clock Gating (ICG) cells
// to reduce dynamic power by gating clock to idle flip-flops.
// Reference: Benini & De Micheli, "Dynamic Power Management", Springer 1998

#include "core/netlist.hpp"
#include <string>
#include <vector>

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

private:
    Netlist& nl_;
    int min_group_ = 1;
    double activity_ = 0.1; // 10% default switching

    // Find groups of FFs sharing the same enable condition
    struct FfGroup {
        NetId clock;
        NetId enable; // -1 if no enable
        std::vector<GateId> ffs;
    };
    std::vector<FfGroup> find_groups();
};

} // namespace sf
