#pragma once
// SiliconForge — PODEM ATPG (Automatic Test Pattern Generation)
// Reference: P. Goel, "An Implicit Enumeration Algorithm to Generate Tests
//            for Combinational Logic Circuits", IEEE TC, 1981
//
// Generates test vectors that detect stuck-at faults in combinational circuits.

#include "core/netlist.hpp"
#include <string>
#include <vector>
#include <unordered_set>

namespace sf {

struct Fault {
    NetId net;
    Logic4 stuck_at; // ZERO = stuck-at-0, ONE = stuck-at-1
    std::string str() const;
};

struct AtpgResult {
    bool detected = false;
    std::vector<std::pair<NetId, Logic4>> test_vector; // PI assignments
    std::string message;
};

struct FaultCoverage {
    size_t total_faults = 0;
    size_t detected = 0;
    size_t undetectable = 0;
    double coverage_pct() const {
        return total_faults > 0 ? 100.0 * detected / total_faults : 0.0;
    }
    std::vector<std::pair<Fault, std::vector<std::pair<NetId, Logic4>>>> tests;
};

// Parallel-pattern fault simulation
struct ParallelFaultSimResult {
    int faults_simulated;
    int faults_detected;
    double coverage_pct;
    double time_ms;
    int patterns_used;
};

// Fault collapsing
struct CollapseResult {
    int original_faults;
    int collapsed_faults;
    int equivalence_removed;
    int dominance_removed;
};

// Test compaction
struct CompactResult {
    int original_patterns;
    int compacted_patterns;
    double reduction_pct;
};

// Transition fault ATPG
struct TransitionResult {
    int transition_faults;
    int detected;
    double coverage_pct;
    int patterns;
};

class PodemAtpg {
public:
    explicit PodemAtpg(Netlist& nl);

    // Generate test for a single fault
    AtpgResult generate_test(const Fault& fault);

    // Generate tests for all faults (stuck-at fault model)
    FaultCoverage run_full_atpg();

    // Enumerate all single stuck-at faults
    std::vector<Fault> enumerate_faults() const;

    // Enhanced ATPG methods
    ParallelFaultSimResult parallel_fault_sim(int word_width = 64);
    CollapseResult collapse_faults();
    CompactResult compact_patterns();
    TransitionResult generate_transition_patterns();
    FaultCoverage run_enhanced();

    // ── Tier 2: FAN algorithm enhancements ──────────────────────────────
    // FAN (Fanout-Oriented Test Generation) improvements over PODEM:
    // - Multiple backtrace: explores several PI assignments simultaneously
    // - Fanout-oriented backtrace: for fanout stems, determine required values
    //   at ALL branches before backtracing to PIs (reduces backtracks)
    // - Dynamic learning: record implications to avoid redundant work
    struct FanConfig {
        bool enable_multiple_backtrace = true;
        bool enable_learning = true;
        int max_backtrace_alternatives = 3;
    };
    void set_fan_config(const FanConfig& cfg) { fan_cfg_ = cfg; }
    FaultCoverage run_fan_atpg();

private:
    Netlist& nl_;
    std::vector<GateId> topo_;

    // PODEM core: recursive backtracking
    enum class PodemStatus { SUCCESS, FAILURE, BACKTRACK };

    static constexpr int MAX_RECURSION_DEPTH = 200;
    static constexpr int MAX_BACKTRACKS = 5000;
    int backtrack_count_;

    // 5-value logic for D-algorithm: {0, 1, X, D, D'}
    enum class DLogic { ZERO, ONE, X, D, DBAR };

    std::vector<DLogic> net_values_; // Current 5-value assignment
    std::unordered_set<NetId> pi_set_; // Cached PI + pseudo-PI set (includes DFF outputs)
    std::unordered_set<NetId> po_set_; // Cached PO + pseudo-PO set (includes DFF inputs)
    const Fault* current_fault_ = nullptr;

    void init_values();
    bool podem_recursive(const Fault& fault, int depth = 0);
    bool objective(const Fault& fault, NetId& obj_net, DLogic& obj_val);
    bool backtrace(NetId obj_net, DLogic obj_val, NetId& pi, Logic4& pi_val);
    void forward_imply_with_fault(const Fault& fault);
    bool fault_propagated(const Fault& fault);
    DLogic eval_gate_d(GateType type, const std::vector<DLogic>& inputs);

    FanConfig fan_cfg_;

    // FAN: learned implications (net → required value)
    std::unordered_map<NetId, DLogic> learned_implications_;

    // FAN: multiple backtrace — find multiple PI assignments
    struct BacktraceAlt {
        NetId pi;
        Logic4 value;
        double score; // lower = better
    };
    std::vector<BacktraceAlt> multiple_backtrace(NetId obj_net, DLogic obj_val, int max_alts);

    // FAN: fanout stem analysis — check if a net is a fanout stem
    bool is_fanout_stem(NetId net) const;
};

} // namespace sf
