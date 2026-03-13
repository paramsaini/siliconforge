#pragma once
// SiliconForge — Memory Compiler
// Generates SRAM and Register File macros with configurable parameters.
// Produces netlists, timing models, and physical outlines.
// Reference: Weste & Harris, "CMOS VLSI Design", Ch. 12 — SRAM Design

#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace sf {

struct MemoryConfig {
    enum Type { SRAM_1P, SRAM_2P, REGISTER_FILE, ROM } type = SRAM_1P;
    int words = 256;
    int bits = 32;
    int read_ports = 1;
    int write_ports = 1;
    std::string name = "mem";
    // Technology parameters
    double bit_cell_width = 0.5;    // um
    double bit_cell_height = 1.0;   // um
    double decoder_overhead = 1.2;
    double sense_amp_height = 5.0;  // um
    double periphery_ratio = 0.3;   // 30% overhead for periphery
    // Column multiplexing (1, 2, 4, 8)
    int column_mux = 1;
    // Banking
    int num_banks = 1;
    bool bank_interleave = false;
    // ECC
    bool enable_ecc = false;        // SECDED (Single Error Correct, Double Error Detect)
    // MBIST wrapper
    bool enable_mbist = false;
    // Redundancy
    int redundant_rows = 0;
    int redundant_cols = 0;
    // Power management
    bool enable_power_gating = false;
    bool enable_retention = false;
    double vdd = 1.0;              // Supply voltage (V)
    double temperature = 25.0;     // Operating temperature (°C)
    // Process corner
    enum ProcessCorner { TT, FF, SS, SF, FS } corner = TT;
};

struct MemoryTiming {
    double tAA_ns = 0;      // Address-to-output access time
    double tCK_ns = 0;      // Minimum clock cycle
    double tSetup_ns = 0;   // Data setup time
    double tHold_ns = 0;    // Data hold time
    double tWC_ns = 0;      // Write cycle time
    double tCKH_ns = 0;    // Clock high time
    double tCKL_ns = 0;    // Clock low time
    double tAS_ns = 0;     // Address setup
    double tAH_ns = 0;     // Address hold
    double tDS_ns = 0;     // Data setup (alias for tSetup)
    double tDH_ns = 0;     // Data hold (alias for tHold)
    double tWEH_ns = 0;    // Write enable hold
    double tWES_ns = 0;    // Write enable setup
    double tCD_ns = 0;     // Clock to data output
    double tOE_ns = 0;     // Output enable delay
};

struct MemoryResult {
    bool success = false;
    std::string name;
    int words = 0, bits = 0;
    double width_um = 0, height_um = 0;
    double area_um2 = 0;
    double bit_density = 0;          // bits per um²
    MemoryTiming timing;
    double leakage_uw = 0;
    double read_energy_pj = 0;
    double write_energy_pj = 0;

    // Banking/mux info
    int actual_banks = 1;
    int actual_column_mux = 1;
    int total_rows = 0;
    int total_cols = 0;
    // ECC
    int ecc_bits = 0;          // Extra parity bits for SECDED
    int total_bits_per_word = 0;  // bits + ecc_bits
    // Redundancy
    int total_redundant_rows = 0;
    int total_redundant_cols = 0;
    // Power details
    double dynamic_power_mw = 0;
    double standby_leakage_uw = 0;
    double retention_leakage_uw = 0;
    // Corner-adjusted timing
    double timing_derating = 1.0;    // Applied to all timing values
    // MBIST
    std::string mbist_report;
    // Reports
    std::string detailed_report;     // Full characterization report

    // Generated outputs
    Netlist netlist;
    std::string liberty_model;       // timing model in Liberty format
    std::string verilog_model;       // behavioral Verilog
    std::string message;
};

// Multi-port SRAM
struct MultiPortConfig {
    int read_ports;
    int write_ports;
    int rw_ports;
    int word_width;
    int depth;
    bool has_byte_enable;
};
struct MultiPortResult {
    double area;
    double read_delay_ns;
    double write_delay_ns;
    double leakage_uw;
    int total_transistors;
};

// Timing model generation (.lib format)
struct TimingModelResult {
    std::string cell_name;
    double setup_time;
    double hold_time;
    double clk_to_q;
    double read_access_time;
    double write_access_time;
    std::string lib_format;
};

// Column/row redundancy
struct RedundancyResult {
    int spare_cols;
    int spare_rows;
    double yield_improvement_pct;
    double area_overhead_pct;
};

// BIST wrapper
struct BistWrapperResult {
    std::string wrapper_name;
    int march_algorithms;
    double test_time_cycles;
    bool self_repair;
};

class MemoryCompiler {
public:
    MemoryResult compile(const MemoryConfig& cfg);
    MultiPortResult generate_multi_port(const MultiPortConfig& cfg);
    TimingModelResult generate_timing_model(const MultiPortConfig& cfg);
    RedundancyResult add_redundancy(int spare_cols = 2, int spare_rows = 2);
    BistWrapperResult generate_bist_wrapper(bool with_repair = false);
    MemoryResult run_enhanced(const MemoryConfig& cfg = {});

    // ── Tier 3: Sense Amplifier Design ──────────────────────────────────
    struct SenseAmpConfig {
        double vth_mismatch_sigma = 0.030;  // V, threshold mismatch 1σ
        double precharge_time_ps = 200.0;   // precharge settling time
        double sense_time_ps = 150.0;       // SA evaluation time
        double min_differential_mv = 50.0;  // minimum BL differential for sensing
        double sa_power_uw = 5.0;           // per-SA dynamic power
        int column_mux_ratio = 1;
        bool enable_offset_cancellation = false;
    };

    struct SenseAmpResult {
        int total_sense_amps = 0;
        double sensing_margin_mv = 0;       // actual BL differential at sense time
        double offset_3sigma_mv = 0;        // 3σ SA offset voltage
        double min_sensing_window_ps = 0;   // minimum reliable sense window
        double sa_total_power_uw = 0;
        double read_bitline_delay_ps = 0;
        bool margin_adequate = false;       // sensing_margin > offset_3sigma
        std::string message;
    };

    SenseAmpResult design_sense_amps(const MemoryConfig& cfg,
                                      const SenseAmpConfig& sa_cfg);

private:
    void generate_netlist(const MemoryConfig& cfg, MemoryResult& r);
    void compute_timing(const MemoryConfig& cfg, MemoryResult& r);
    void generate_liberty(const MemoryConfig& cfg, MemoryResult& r);
    void generate_verilog(const MemoryConfig& cfg, MemoryResult& r);
    void compute_dimensions(const MemoryConfig& cfg, MemoryResult& r);
    void compute_power(const MemoryConfig& cfg, MemoryResult& r);
    void compute_ecc(const MemoryConfig& cfg, MemoryResult& r);
    void compute_banking(const MemoryConfig& cfg, MemoryResult& r);
    void compute_redundancy(const MemoryConfig& cfg, MemoryResult& r);
    void compute_corner_derating(const MemoryConfig& cfg, MemoryResult& r);
    void generate_mbist_wrapper(const MemoryConfig& cfg, MemoryResult& r);
    void generate_detailed_report(const MemoryConfig& cfg, MemoryResult& r);
    MemoryConfig last_cfg_;
};

} // namespace sf
