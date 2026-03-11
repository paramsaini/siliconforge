#pragma once
// SiliconForge — AIG Optimization Passes
// Implements DAG-aware rewriting and AIG balancing to reduce circuit size/depth.
// Reference: Mishchenko et al., "DAG-Aware AIG Rewriting", DAC 2006

#include "core/aig.hpp"
#include <string>
#include <vector>

namespace sf {

struct OptStats {
    uint32_t initial_ands = 0;
    uint32_t final_ands = 0;
    uint32_t initial_depth = 0;
    uint32_t final_depth = 0;
    int passes = 0;
    double time_ms = 0.0;
};

class AigOptimizer {
public:
    explicit AigOptimizer(AigGraph& aig) : aig_(aig) {}

    // Run all optimization passes
    OptStats optimize(int num_passes = 3);

    // Individual passes
    void rewrite();       // Local rewriting with 4-input cuts
    void refactor();      // Collapse and re-derive sub-circuits
    void balance();       // Tree-height reduction via associative balancing

    // Constant propagation and dead node elimination
    void sweep();

    // SAT-based redundancy removal
    void sat_sweep();

    // Compute AIG depth (longest path from input to output)
    static uint32_t compute_depth(const AigGraph& aig);

private:
    AigGraph& aig_;

    // Rewriting helpers
    struct Cut {
        std::vector<uint32_t> leaves;
        uint32_t root;
        uint32_t size;
    };

    std::vector<Cut> enumerate_cuts(uint32_t var, int max_cut_size = 4);
    AigLit rebuild_from_truth_table(uint16_t tt, const std::vector<uint32_t>& leaves);
};

} // namespace sf
