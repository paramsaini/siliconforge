#pragma once
// SiliconForge — Scan Compression Engine (LBIST, EDT, DROST)
// Implements embedded deterministic test compression with LFSR-based
// decompression and MISR-based response compaction.
// Reference: Rajski et al., "Embedded Deterministic Test", IEEE TCOMP, 2004
//
// Architecture:
//   LFSR (seed) → Phase Shifter → Decompressor → Internal Scan Chains
//   Internal Scan Chains → Compactor (XOR tree) → MISR → Signature
//
// LBIST mode: LFSR generates pseudo-random patterns autonomously.
// EDT mode: LFSR seeds are deterministically computed for target faults.
// DROST mode: Design-rule-oriented constraints applied during compression.

#include "core/netlist.hpp"
#include <vector>
#include <string>
#include <cstdint>

namespace sf {

// ── Configuration ────────────────────────────────────────────────────────────

struct ScanCompressConfig {
    int compression_ratio = 10;     // 10:1 default
    int num_scan_chains = 32;
    bool enable_lbist = true;
    bool enable_misr = true;        // Multiple-Input Signature Register
    int lbist_patterns = 1000;
    int lfsr_width = 32;            // LFSR seed width
    double target_coverage = 95.0;
};

// ── Results ──────────────────────────────────────────────────────────────────

struct LbistResult {
    int patterns_applied = 0;
    double coverage_pct = 0;
    uint32_t misr_signature = 0;
    bool signature_match = false;
    int faults_detected = 0;
    int total_faults = 0;
};

struct CompressResult {
    int original_patterns = 0;
    int compressed_patterns = 0;
    double compression_ratio = 0;
    int decompressor_gates = 0;     // area overhead
    int compactor_gates = 0;
    double coverage_pct = 0;
    std::string report;
};

// ── Scan Compression Engine ──────────────────────────────────────────────────

class ScanCompressor {
public:
    explicit ScanCompressor(Netlist& nl);

    // EDT-style compression: LFSR seeds → decompressor → scan chains
    CompressResult compress(const ScanCompressConfig& cfg = {});

    // Logic Built-In Self-Test: autonomous pattern generation + MISR capture
    LbistResult run_lbist(const ScanCompressConfig& cfg = {});

    // Galois LFSR with maximal-length polynomial feedback
    // Taps: bits 31,21,1,0 for 32-bit (width-specific taps for smaller widths)
    uint32_t lfsr_next(uint32_t state, int width);

    // Multiple-Input Signature Register: compress response data into signature
    // MISR = (misr << 1) ^ data ^ feedback_from_MSB
    uint32_t misr_compress(uint32_t misr, uint32_t data);

    // Full enhanced flow: compress + LBIST + combined report
    CompressResult run_enhanced();

private:
    Netlist& nl_;
    std::vector<GateId> topo_;

    // Single-pattern Logic4 simulation through the netlist
    std::vector<Logic4> simulate_pattern(const std::vector<Logic4>& pi_values);

    // Simulate with a stuck-at fault injected
    std::vector<Logic4> simulate_faulty(const std::vector<Logic4>& pi_values,
                                        NetId fault_net, Logic4 stuck_val);
};

} // namespace sf
