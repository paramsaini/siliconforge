#pragma once
// SiliconForge — Datapath Synthesis Engine
// Carry-chain inference, Wallace tree multiplier construction, and Booth
// encoding detection for arithmetic datapath optimization.
// Reference: Weste & Harris, "CMOS VLSI Design", Ch. 11 — Datapath Subsystems
// Reference: Wallace, "A Suggestion for a Fast Multiplier", IEEE TC, 1964

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include <string>
#include <vector>
#include <cstdint>

namespace sf {

// ============================================================================
// Configuration
// ============================================================================

struct DatapathConfig {
    bool   enable_carry_chain   = true;
    bool   enable_wallace_tree  = true;
    bool   enable_booth_encoding = true;
    int    max_adder_width      = 64;
    double target_freq_mhz     = 500.0;
};

// ============================================================================
// Result / report structures
// ============================================================================

struct DatapathResult {
    int    adders_optimized       = 0;
    int    multipliers_optimized  = 0;
    double area_reduction_pct    = 0.0;
    double delay_improvement_pct = 0.0;
    int    carry_chains_inferred  = 0;
    int    wallace_trees_built    = 0;
    int    booth_encoders_built   = 0;
    std::string report;
};

// ============================================================================
// Carry-chain analysis
// ============================================================================

struct CarryChainInfo {
    int    width             = 0;
    GateId start_gate        = -1;
    GateId end_gate          = -1;
    double propagate_delay_ps = 0.0;   // per-bit propagate delay
    double generate_delay_ps  = 0.0;   // per-bit generate delay
};

// ============================================================================
// Wallace tree multiplier
// ============================================================================

struct WallaceTreeInfo {
    int input_bits       = 0;
    int partial_products = 0;
    int levels           = 0;
    int area_gates       = 0;
};

// ============================================================================
// Booth encoder
// ============================================================================

struct BoothEncoderInfo {
    int multiplier_width = 0;
    enum EncodingType { RADIX2, RADIX4, RADIX8 };
    EncodingType encoding_type = RADIX2;
    double partial_product_reduction_pct = 0.0;
};

// ============================================================================
// Datapath Optimizer
// ============================================================================

class DatapathOptimizer {
public:
    explicit DatapathOptimizer(Netlist& nl, const LibertyLibrary* lib = nullptr);

    // Run all enabled optimizations
    DatapathResult optimize(const DatapathConfig& cfg = {});

    // Individual optimization passes
    std::vector<CarryChainInfo>   infer_carry_chains();
    std::vector<WallaceTreeInfo>  build_wallace_trees(int max_inputs = 8);
    std::vector<BoothEncoderInfo> apply_booth_encoding();

    // Full flow with detailed report
    DatapathResult run_enhanced();

private:
    Netlist& nl_;
    const LibertyLibrary* lib_;
    DatapathConfig cfg_;

    // Carry-chain helpers
    bool is_propagate_gate(GateId gid) const;
    bool is_generate_gate(GateId gid) const;
    bool share_inputs(GateId a, GateId b) const;
    double gate_delay_ps(GateType type) const;

    // Wallace tree helpers
    int wallace_levels(int n) const;
    int csa_gate_count(int partial_products) const;

    // Booth helpers
    BoothEncoderInfo::EncodingType select_booth_radix(int width) const;
    double booth_reduction_pct(BoothEncoderInfo::EncodingType enc) const;

    // Pattern recognition
    struct AndArray {
        std::vector<GateId> gates;
        int rows = 0;
        int cols = 0;
    };
    std::vector<AndArray> find_and_arrays() const;

    // Report generation
    std::string build_report(const DatapathResult& res) const;
};

} // namespace sf
