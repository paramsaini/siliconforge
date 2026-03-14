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

// ── Fault Dictionary & Diagnosis ─────────────────────────────────────────────
// Fault dictionary: maps each fault to the set of patterns that detect it.
// Used for fault diagnosis (identifying the actual defect from failing patterns).

struct FaultDictEntry {
    Fault fault;
    std::vector<int> detecting_patterns;  // indices into test vector set
};

struct FaultDictionary {
    std::vector<FaultDictEntry> entries;
    size_t total_faults = 0;
    double coverage_pct = 0;
};

struct DiagnosisResult {
    std::vector<Fault> candidate_faults;  // sorted by match confidence
    int patterns_used = 0;
    double confidence = 0;                // 0-100%, how well top candidate matches
};

// ── Fault Simulator ──────────────────────────────────────────────────────────

class FaultSimulator {
public:
    explicit FaultSimulator(Netlist& nl);

    // Run fault simulation with given test vectors
    // Each vector: values for primary inputs (Logic4)
    FaultSimResult simulate(const std::vector<std::vector<Logic4>>& test_vectors);

    // Build fault dictionary: for each stuck-at fault, record detecting patterns
    FaultDictionary build_fault_dictionary(const std::vector<std::vector<Logic4>>& vectors);

    // Fault diagnosis: compare observed vs expected, identify likely faults
    DiagnosisResult diagnose(const std::vector<std::vector<Logic4>>& vectors,
                             const std::vector<std::vector<Logic4>>& observed_responses);

    // Enhanced flow: simulate + dictionary + supplementary patterns
    FaultSimResult run_enhanced(const std::vector<std::vector<Logic4>>& vectors);

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
