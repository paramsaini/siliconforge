#pragma once
// SiliconForge — PODEM ATPG (Automatic Test Pattern Generation)
// Reference: P. Goel, "An Implicit Enumeration Algorithm to Generate Tests
//            for Combinational Logic Circuits", IEEE TC, 1981
//
// Generates test vectors that detect stuck-at faults in combinational circuits.

#include "core/netlist.hpp"
#include <string>
#include <vector>

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

class PodemAtpg {
public:
    explicit PodemAtpg(Netlist& nl);

    // Generate test for a single fault
    AtpgResult generate_test(const Fault& fault);

    // Generate tests for all faults (stuck-at fault model)
    FaultCoverage run_full_atpg();

    // Enumerate all single stuck-at faults
    std::vector<Fault> enumerate_faults() const;

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

    void init_values();
    bool podem_recursive(const Fault& fault, int depth = 0);
    bool objective(const Fault& fault, NetId& obj_net, DLogic& obj_val);
    bool backtrace(NetId obj_net, DLogic obj_val, NetId& pi, Logic4& pi_val);
    bool forward_imply();
    bool fault_propagated(const Fault& fault);
    DLogic eval_gate_d(GateType type, const std::vector<DLogic>& inputs);
};

} // namespace sf
