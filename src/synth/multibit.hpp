#pragma once
// SiliconForge — Multi-Bit Flip-Flop Banking
// Groups spatially close DFFs sharing clock/reset into multi-bit cells
// for area savings (~20% per banked group).

#include "core/netlist.hpp"
#include <string>
#include <vector>

namespace sf {

struct MultiBitConfig {
    int max_bank_width = 4;      // max FFs per multi-bit cell
    double max_distance = 5.0;   // max spatial distance for banking
    bool enable = true;
};

struct MultiBitResult {
    int original_ffs = 0;
    int banked_groups = 0;
    int banked_ffs = 0;
    int remaining_ffs = 0;
    double area_savings_pct = 0;
    std::string message;
};

class MultiBitOptimizer {
public:
    explicit MultiBitOptimizer(Netlist& nl) : nl_(nl) {}

    MultiBitResult optimize(const MultiBitConfig& cfg = {});

private:
    Netlist& nl_;

    // Find groups of DFFs that can be banked together
    std::vector<std::vector<GateId>> find_bankable_groups(const MultiBitConfig& cfg);

    // Merge a group of DFFs into a single multi-bit cell
    void merge_group(const std::vector<GateId>& group);
};

} // namespace sf
