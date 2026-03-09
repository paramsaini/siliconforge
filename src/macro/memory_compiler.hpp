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
};

struct MemoryTiming {
    double tAA_ns = 0;      // Address-to-output access time
    double tCK_ns = 0;      // Minimum clock cycle
    double tSetup_ns = 0;   // Data setup time
    double tHold_ns = 0;    // Data hold time
    double tWC_ns = 0;      // Write cycle time
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

    // Generated outputs
    Netlist netlist;
    std::string liberty_model;       // timing model in Liberty format
    std::string verilog_model;       // behavioral Verilog
    std::string message;
};

class MemoryCompiler {
public:
    MemoryResult compile(const MemoryConfig& cfg);

private:
    void generate_netlist(const MemoryConfig& cfg, MemoryResult& r);
    void compute_timing(const MemoryConfig& cfg, MemoryResult& r);
    void generate_liberty(const MemoryConfig& cfg, MemoryResult& r);
    void generate_verilog(const MemoryConfig& cfg, MemoryResult& r);
    void compute_dimensions(const MemoryConfig& cfg, MemoryResult& r);
    void compute_power(const MemoryConfig& cfg, MemoryResult& r);
};

} // namespace sf
