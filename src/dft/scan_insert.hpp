#pragma once
// SiliconForge — Scan Chain Insertion for DFT
// Converts regular DFFs to muxed-scan flip-flops and stitches scan chains.

#include "core/netlist.hpp"
#include <string>
#include <vector>

namespace sf {

struct ScanConfig {
    int max_chain_length = 100;  // Max FFs per scan chain
    std::string scan_in_prefix = "scan_in";
    std::string scan_out_prefix = "scan_out";
    std::string scan_enable_name = "scan_enable";
};

struct ScanResult {
    int num_chains = 0;
    int total_ffs = 0;
    std::vector<int> chain_lengths;
    std::string message;
};

// Multi-chain partitioning
struct MultiChainConfig {
    int num_chains;
    enum BalanceMode { BY_LENGTH, BY_ROUTING, BY_DOMAIN } mode;
};
struct MultiChainResult {
    int num_chains;
    std::vector<int> chain_lengths;
    double length_imbalance_pct;
    std::vector<std::vector<int>> chains;  // FF ids per chain
};

// Scan compression (EDT-like)
struct CompressionConfig {
    int decompressor_inputs;
    int compressor_outputs;
    double compression_ratio;
};
struct CompressionResult {
    double actual_ratio;
    int internal_chains;
    int external_channels;
    int patterns_saved;
};

// Test point insertion
struct TestPointResult {
    int observe_points;
    int control_points;
    double coverage_improvement_pct;
};

// Scan chain reorder for routing
struct ReorderResult {
    double wirelength_before;
    double wirelength_after;
    double improvement_pct;
};

class ScanInserter {
public:
    explicit ScanInserter(Netlist& nl) : nl_(nl) {}

    // Insert scan chains — modifies the netlist in place
    ScanResult insert(const ScanConfig& config = {});

    MultiChainResult partition_chains(const MultiChainConfig& cfg);
    CompressionResult insert_compression(const CompressionConfig& cfg);
    TestPointResult insert_test_points(int max_points = 50);
    ReorderResult reorder_chains();
    ScanResult run_enhanced(const ScanConfig& scan_cfg = {},
                           const MultiChainConfig& mc_cfg = {4, MultiChainConfig::BY_LENGTH},
                           const CompressionConfig& comp_cfg = {4, 2, 10.0});

    // ── Tier 2: Partial scan selection ─────────────────────────────────
    // Instead of making all FFs scannable, select a subset based on testability
    // analysis (SCOAP controllability/observability) to achieve target coverage
    // with minimal area overhead.
    struct PartialScanConfig {
        double target_coverage = 0.95;  // target fault coverage (0.0-1.0)
        double scan_ratio = 0.7;        // max fraction of FFs to make scannable
        enum SelectionMode { SCOAP, RANDOM, CRITICAL_PATH } mode = SCOAP;
    };
    struct PartialScanResult {
        int total_ffs = 0;
        int selected_ffs = 0;
        double scan_ratio = 0;
        double estimated_coverage = 0;
        std::vector<int> selected_ff_ids;
        std::string message;
    };
    PartialScanResult select_partial_scan(const PartialScanConfig& cfg);

private:
    Netlist& nl_;

    // SCOAP testability scores per gate
    struct TestabilityScore {
        double controllability_0 = 0; // difficulty to set to 0
        double controllability_1 = 0; // difficulty to set to 1
        double observability = 0;     // difficulty to observe
    };
    std::unordered_map<GateId, TestabilityScore> compute_scoap();
};

} // namespace sf
