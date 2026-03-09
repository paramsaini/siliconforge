#pragma once
// SiliconForge — Parallel-Pattern Fault Simulator
// Uses bit-parallel simulation to evaluate 64 test patterns simultaneously.
// Reference: "Parallel Pattern Single Fault Propagation", Waicukauski et al., DAC 1985

#include "core/netlist.hpp"
#include "dft/podem.hpp"
#include <vector>
#include <cstdint>

namespace sf {

struct FaultSimResult {
    size_t total_faults = 0;
    size_t detected = 0;
    double coverage_pct() const {
        return total_faults > 0 ? 100.0 * detected / total_faults : 0.0;
    }
};

class FaultSimulator {
public:
    explicit FaultSimulator(Netlist& nl);

    // Run fault simulation with given test vectors
    // Each vector: values for primary inputs (Logic4)
    FaultSimResult simulate(const std::vector<std::vector<Logic4>>& test_vectors);

private:
    Netlist& nl_;
    std::vector<GateId> topo_;

    // Bit-parallel: simulate 64 patterns at once using uint64_t
    // good_val[net] = 64-bit pattern of good-circuit values
    // faulty_val[net] = 64-bit pattern with fault injected

    // Evaluate gate in bit-parallel mode
    uint64_t eval_gate_64(GateType type, const std::vector<uint64_t>& inputs);
};

} // namespace sf
