// SiliconForge — Memory Compiler Implementation
#include "macro/memory_compiler.hpp"
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>

namespace sf {

// ---------------------------------------------------------------------------
// Banking: partition memory into independent banks
// ---------------------------------------------------------------------------
void MemoryCompiler::compute_banking(const MemoryConfig& cfg, MemoryResult& r) {
    r.actual_banks = std::max(1, cfg.num_banks);
    r.actual_column_mux = std::max(1, cfg.column_mux);
}

// ---------------------------------------------------------------------------
// ECC: Hamming SECDED parity bits
// ---------------------------------------------------------------------------
void MemoryCompiler::compute_ecc(const MemoryConfig& cfg, MemoryResult& r) {
    if (!cfg.enable_ecc) {
        r.ecc_bits = 0;
        r.total_bits_per_word = cfg.bits;
        return;
    }
    // SECDED: ecc_bits = ceil(log2(data_bits + ceil(log2(data_bits)) + 1)) + 1
    int m = (int)std::ceil(std::log2(cfg.bits + std::ceil(std::log2(cfg.bits)) + 1)) + 1;
    r.ecc_bits = m;
    r.total_bits_per_word = cfg.bits + r.ecc_bits;
}

// ---------------------------------------------------------------------------
// Redundancy: spare rows and columns
// ---------------------------------------------------------------------------
void MemoryCompiler::compute_redundancy(const MemoryConfig& cfg, MemoryResult& r) {
    r.total_redundant_rows = cfg.redundant_rows;
    r.total_redundant_cols = cfg.redundant_cols;
}

// ---------------------------------------------------------------------------
// Process corner and environment derating
// ---------------------------------------------------------------------------
void MemoryCompiler::compute_corner_derating(const MemoryConfig& cfg, MemoryResult& r) {
    double timing_factor = 1.0;
    double leakage_factor = 1.0;

    switch (cfg.corner) {
        case MemoryConfig::TT: timing_factor = 1.0;  leakage_factor = 1.0; break;
        case MemoryConfig::FF: timing_factor = 0.8;  leakage_factor = 1.5; break;
        case MemoryConfig::SS: timing_factor = 1.3;  leakage_factor = 0.6; break;
        case MemoryConfig::SF: timing_factor = 1.15; leakage_factor = 0.9; break;
        case MemoryConfig::FS: timing_factor = 0.9;  leakage_factor = 1.2; break;
    }

    // Temperature scaling for timing
    timing_factor *= 1.0 + 0.003 * (cfg.temperature - 25.0);
    // Voltage scaling for timing (normalized to 1V)
    timing_factor *= (1.0 / cfg.vdd);

    r.timing_derating = timing_factor;
    // Store leakage factor in standby_leakage_uw temporarily; compute_power applies it
    r.standby_leakage_uw = leakage_factor;
}

// ---------------------------------------------------------------------------
// Physical dimensions (updated for banking, ECC, mux, redundancy)
// ---------------------------------------------------------------------------
void MemoryCompiler::compute_dimensions(const MemoryConfig& cfg, MemoryResult& r) {
    int effective_bits = r.total_bits_per_word; // includes ECC bits
    int words_per_bank = cfg.words / r.actual_banks;

    // Array dimensions per bank
    int rows = (int)std::sqrt(words_per_bank);
    if (rows < 1) rows = 1;
    int cols = words_per_bank / rows;
    if (rows * cols < words_per_bank) cols++;

    // Column mux reduces sense-amp count
    int sense_amps_per_bank = (cols * effective_bits) / r.actual_column_mux;

    r.total_rows = rows + r.total_redundant_rows;
    r.total_cols = cols * effective_bits + r.total_redundant_cols;

    // Bit cell array per bank
    double array_w = (double)r.total_cols * cfg.bit_cell_width;
    double array_h = (double)r.total_rows * cfg.bit_cell_height;

    // Sense amp height reduced by column mux
    double sa_height = cfg.sense_amp_height / r.actual_column_mux;

    // Single bank area
    double bank_w = array_w * cfg.decoder_overhead;
    double bank_h = array_h + sa_height + (array_h * cfg.periphery_ratio);
    double single_bank_area = bank_w * bank_h;

    // ECC logic area overhead (~15%)
    double ecc_area_factor = cfg.enable_ecc ? 1.15 : 1.0;

    // Total area: banks * per-bank * routing overhead * ECC
    double total_area = single_bank_area * r.actual_banks *
                        (1.0 + 0.05 * (r.actual_banks - 1)) * ecc_area_factor;

    // Arrange banks in a roughly square grid for width/height estimate
    int bank_cols = (int)std::ceil(std::sqrt((double)r.actual_banks));
    int bank_rows_layout = (r.actual_banks + bank_cols - 1) / bank_cols;

    r.width_um = bank_w * bank_cols;
    r.height_um = bank_h * bank_rows_layout;
    r.area_um2 = total_area;
    r.bit_density = (double)(cfg.words * cfg.bits) / r.area_um2;
}

// ---------------------------------------------------------------------------
// Timing model (updated for mux, corner derating, new arcs)
// ---------------------------------------------------------------------------
void MemoryCompiler::compute_timing(const MemoryConfig& cfg, MemoryResult& r) {
    double depth_factor = std::log2(cfg.words) / 8.0;
    double width_factor = cfg.bits / 32.0;

    // Base access time
    r.timing.tAA_ns = 0.5 + 0.3 * depth_factor;
    r.timing.tCK_ns = r.timing.tAA_ns * 1.5;
    r.timing.tSetup_ns = 0.05 + 0.02 * depth_factor;
    r.timing.tHold_ns = 0.02;
    r.timing.tWC_ns = r.timing.tCK_ns;

    if (cfg.type == MemoryConfig::REGISTER_FILE) {
        r.timing.tAA_ns *= 0.6;
        r.timing.tCK_ns *= 0.6;
    }

    // Column mux delay: +0.05ns per mux stage
    if (r.actual_column_mux > 1) {
        double mux_delay = 0.05 * std::log2((double)r.actual_column_mux);
        r.timing.tAA_ns += mux_delay;
        r.timing.tCK_ns += mux_delay;
    }

    // Bank select decode delay
    if (r.actual_banks > 1) {
        double bank_delay = 0.1 * std::log2((double)r.actual_banks);
        r.timing.tAA_ns += bank_delay;
        r.timing.tCK_ns += bank_delay;
    }

    // New timing arcs (derived from base)
    r.timing.tCKH_ns = r.timing.tCK_ns * 0.45;
    r.timing.tCKL_ns = r.timing.tCK_ns * 0.45;
    r.timing.tAS_ns = r.timing.tSetup_ns * 1.1;
    r.timing.tAH_ns = r.timing.tHold_ns * 1.1;
    r.timing.tDS_ns = r.timing.tSetup_ns;
    r.timing.tDH_ns = r.timing.tHold_ns;
    r.timing.tWES_ns = r.timing.tSetup_ns * 0.9;
    r.timing.tWEH_ns = r.timing.tHold_ns * 0.9;
    r.timing.tCD_ns = r.timing.tAA_ns * 0.85;
    r.timing.tOE_ns = 0.1 + 0.02 * width_factor;

    // Apply corner derating to all timing values
    double d = r.timing_derating;
    r.timing.tAA_ns    *= d;
    r.timing.tCK_ns    *= d;
    r.timing.tSetup_ns *= d;
    r.timing.tHold_ns  *= d;
    r.timing.tWC_ns    *= d;
    r.timing.tCKH_ns   *= d;
    r.timing.tCKL_ns   *= d;
    r.timing.tAS_ns    *= d;
    r.timing.tAH_ns    *= d;
    r.timing.tDS_ns    *= d;
    r.timing.tDH_ns    *= d;
    r.timing.tWES_ns   *= d;
    r.timing.tWEH_ns   *= d;
    r.timing.tCD_ns    *= d;
    r.timing.tOE_ns    *= d;
}

// ---------------------------------------------------------------------------
// Power model (updated for temp, voltage, banking, gating, ECC)
// ---------------------------------------------------------------------------
void MemoryCompiler::compute_power(const MemoryConfig& cfg, MemoryResult& r) {
    double total_bits = (double)cfg.words * r.total_bits_per_word;

    // Base leakage: 1nW per bit
    double base_leakage = total_bits * 0.001;

    // Temperature scaling for leakage
    double temp_factor = std::exp(0.035 * (cfg.temperature - 25.0));
    base_leakage *= temp_factor;

    // Corner leakage factor (stored in standby_leakage_uw by compute_corner_derating)
    double leakage_corner_factor = r.standby_leakage_uw;
    base_leakage *= leakage_corner_factor;

    r.leakage_uw = base_leakage;

    // Dynamic energy
    double base_read = total_bits * 0.0005 + cfg.bits * 0.1;
    double base_write = base_read * 1.2;

    // Voltage scaling: dynamic ∝ Vdd²
    double vdd_sq = cfg.vdd * cfg.vdd;
    base_read *= vdd_sq;
    base_write *= vdd_sq;

    // ECC overhead: +10% dynamic power
    if (cfg.enable_ecc) {
        base_read *= 1.1;
        base_write *= 1.1;
    }

    // Banking: only active bank consumes dynamic power
    if (r.actual_banks > 1) {
        base_read /= r.actual_banks;
        base_write /= r.actual_banks;
    }

    r.read_energy_pj = base_read;
    r.write_energy_pj = base_write;

    // Estimate dynamic power at nominal 500 MHz
    double freq_ghz = 0.5;
    r.dynamic_power_mw = (r.read_energy_pj * freq_ghz) * 0.5 +
                          (r.write_energy_pj * freq_ghz) * 0.5;

    // Standby / retention leakage
    if (cfg.enable_power_gating) {
        if (cfg.enable_retention)
            r.retention_leakage_uw = r.leakage_uw * 0.1;
        else
            r.retention_leakage_uw = 0.0;
        r.standby_leakage_uw = 0.0;
    } else {
        r.standby_leakage_uw = r.leakage_uw;
        r.retention_leakage_uw = r.leakage_uw;
    }
}

// ---------------------------------------------------------------------------
// Netlist generation (functional DFF model)
// ---------------------------------------------------------------------------
void MemoryCompiler::generate_netlist(const MemoryConfig& cfg, MemoryResult& r) {
    auto& nl = r.netlist;
    int addr_bits = (int)std::ceil(std::log2(cfg.words));

    NetId clk = nl.add_net("CLK"); nl.mark_input(clk);
    NetId we = nl.add_net("WE"); nl.mark_input(we);
    NetId ce = nl.add_net("CE"); nl.mark_input(ce);

    std::vector<NetId> addr;
    for (int i = 0; i < addr_bits; ++i) {
        NetId a = nl.add_net("A[" + std::to_string(i) + "]");
        nl.mark_input(a);
        addr.push_back(a);
    }

    std::vector<NetId> din, dout;
    for (int i = 0; i < cfg.bits; ++i) {
        NetId d = nl.add_net("D[" + std::to_string(i) + "]"); nl.mark_input(d); din.push_back(d);
        NetId q = nl.add_net("Q[" + std::to_string(i) + "]"); nl.mark_output(q); dout.push_back(q);
    }

    // Functional model: one DFF per output bit with buffer
    for (int i = 0; i < cfg.bits; ++i) {
        NetId w = nl.add_net("mem_" + std::to_string(i));
        nl.add_dff(din[i], clk, w, -1, "bit_" + std::to_string(i));
        nl.add_gate(GateType::BUF, {w}, dout[i], "out_buf_" + std::to_string(i));
    }
}

// ---------------------------------------------------------------------------
// Liberty timing model (updated with full arcs, power, operating conditions)
// ---------------------------------------------------------------------------
void MemoryCompiler::generate_liberty(const MemoryConfig& cfg, MemoryResult& r) {
    std::ostringstream lib;
    int addr_bits = (int)std::ceil(std::log2(cfg.words));
    auto fixed = [](double v) { std::ostringstream o; o << std::fixed << std::setprecision(4) << v; return o.str(); };

    const char* corner_name = "TT";
    switch (cfg.corner) {
        case MemoryConfig::FF: corner_name = "FF"; break;
        case MemoryConfig::SS: corner_name = "SS"; break;
        case MemoryConfig::SF: corner_name = "SF"; break;
        case MemoryConfig::FS: corner_name = "FS"; break;
        default: break;
    }

    lib << "/* SiliconForge Memory Compiler — Liberty Model */\n"
        << "cell(" << cfg.name << ") {\n"
        << "  area : " << fixed(r.area_um2) << ";\n"
        << "  operating_conditions(" << corner_name << ") {\n"
        << "    voltage : " << fixed(cfg.vdd) << ";\n"
        << "    temperature : " << fixed(cfg.temperature) << ";\n"
        << "    process : " << corner_name << ";\n"
        << "  }\n"
        << "  memory() {\n"
        << "    type : ram;\n"
        << "    address_width : " << addr_bits << ";\n"
        << "    word_width : " << cfg.bits << ";\n"
        << "  }\n";

    // Power groups
    lib << "  leakage_power() {\n"
        << "    value : " << fixed(r.leakage_uw * 1e-6) << "; /* W */\n"
        << "  }\n";

    lib << "  pin(CLK) {\n"
        << "    direction : input;\n"
        << "    clock : true;\n"
        << "    min_pulse_width_high : " << fixed(r.timing.tCKH_ns) << ";\n"
        << "    min_pulse_width_low : " << fixed(r.timing.tCKL_ns) << ";\n"
        << "  }\n"
        << "  pin(CE) { direction : input; }\n"
        << "  pin(WE) {\n"
        << "    direction : input;\n"
        << "    timing() {\n"
        << "      related_pin : CLK;\n"
        << "      timing_type : setup_rising;\n"
        << "      intrinsic_rise : " << fixed(r.timing.tWES_ns) << ";\n"
        << "    }\n"
        << "    timing() {\n"
        << "      related_pin : CLK;\n"
        << "      timing_type : hold_rising;\n"
        << "      intrinsic_rise : " << fixed(r.timing.tWEH_ns) << ";\n"
        << "    }\n"
        << "  }\n";

    // Address bus with setup/hold
    lib << "  bus(A) {\n"
        << "    bus_type : A; direction : input;\n"
        << "    timing() {\n"
        << "      related_pin : CLK;\n"
        << "      timing_type : setup_rising;\n"
        << "      intrinsic_rise : " << fixed(r.timing.tAS_ns) << ";\n"
        << "    }\n"
        << "    timing() {\n"
        << "      related_pin : CLK;\n"
        << "      timing_type : hold_rising;\n"
        << "      intrinsic_rise : " << fixed(r.timing.tAH_ns) << ";\n"
        << "    }\n"
        << "  }\n";

    // Data input bus
    lib << "  bus(D) {\n"
        << "    bus_type : D; direction : input;\n"
        << "    timing() {\n"
        << "      related_pin : CLK;\n"
        << "      timing_type : setup_rising;\n"
        << "      intrinsic_rise : " << fixed(r.timing.tDS_ns) << ";\n"
        << "    }\n"
        << "    timing() {\n"
        << "      related_pin : CLK;\n"
        << "      timing_type : hold_rising;\n"
        << "      intrinsic_rise : " << fixed(r.timing.tDH_ns) << ";\n"
        << "    }\n"
        << "  }\n";

    // Data output bus
    lib << "  bus(Q) {\n"
        << "    bus_type : Q; direction : output;\n"
        << "    timing() {\n"
        << "      related_pin : CLK;\n"
        << "      timing_type : rising_edge;\n"
        << "      cell_rise : " << fixed(r.timing.tAA_ns) << ";\n"
        << "      cell_fall : " << fixed(r.timing.tAA_ns) << ";\n"
        << "    }\n"
        << "    internal_power() {\n"
        << "      related_pin : CLK;\n"
        << "      values : " << fixed(r.read_energy_pj * 1e-12) << "; /* J per access */\n"
        << "    }\n"
        << "  }\n";

    if (cfg.enable_power_gating) {
        lib << "  pin(PWR_GATE) { direction : input; }\n";
    }

    lib << "}\n";
    r.liberty_model = lib.str();
}

// ---------------------------------------------------------------------------
// Verilog behavioral model (multi-port, ECC, banking, power gating)
// ---------------------------------------------------------------------------
void MemoryCompiler::generate_verilog(const MemoryConfig& cfg, MemoryResult& r) {
    int addr_bits = (int)std::ceil(std::log2(cfg.words));
    std::ostringstream v;

    v << "// Generated by SiliconForge Memory Compiler\n";

    // Port list
    v << "module " << cfg.name << " (\n"
      << "  input CLK, CE, WE,\n";
    if (cfg.enable_power_gating)
        v << "  input PWR_GATE,\n";
    if (cfg.type == MemoryConfig::SRAM_2P) {
        v << "  input [" << (addr_bits-1) << ":0] RA,  // Read address\n"
          << "  input [" << (addr_bits-1) << ":0] WA,  // Write address\n";
    } else {
        v << "  input [" << (addr_bits-1) << ":0] A,\n";
    }
    v << "  input [" << (cfg.bits-1) << ":0] D,\n"
      << "  output reg [" << (cfg.bits-1) << ":0] Q\n"
      << ");\n\n";

    // Storage declaration
    int internal_bits = r.total_bits_per_word;
    v << "  reg [" << (internal_bits-1) << ":0] mem [0:" << (cfg.words-1) << "];\n";

    // Column mux decode (behavioral)
    if (r.actual_column_mux > 1) {
        int mux_bits = (int)std::log2((double)r.actual_column_mux);
        v << "\n  // Column mux decode — " << r.actual_column_mux << ":1\n"
          << "  wire [" << (mux_bits-1) << ":0] col_sel = "
          << (cfg.type == MemoryConfig::SRAM_2P ? "RA" : "A") << "[" << (mux_bits-1) << ":0];\n";
    }

    // Bank select (behavioral)
    if (r.actual_banks > 1) {
        int bank_bits = (int)std::ceil(std::log2((double)r.actual_banks));
        v << "\n  // Bank select — " << r.actual_banks << " banks\n"
          << "  wire [" << (bank_bits-1) << ":0] bank_sel = "
          << (cfg.type == MemoryConfig::SRAM_2P ? "RA" : "A") << "[" << (addr_bits-1) << ":" << (addr_bits-bank_bits) << "];\n";
    }

    // ECC encode/decode
    if (cfg.enable_ecc) {
        v << "\n  // ECC SECDED — " << r.ecc_bits << " parity bits\n"
          << "  function [" << (internal_bits-1) << ":0] ecc_encode;\n"
          << "    input [" << (cfg.bits-1) << ":0] data;\n"
          << "    reg [" << (r.ecc_bits-1) << ":0] parity;\n"
          << "    integer i;\n"
          << "    begin\n"
          << "      parity = 0;\n"
          << "      for (i = 0; i < " << cfg.bits << "; i = i + 1)\n"
          << "        if (data[i]) parity = parity ^ (i + 1);\n"
          << "      parity[" << (r.ecc_bits-1) << "] = ^{data, parity[" << (r.ecc_bits-2) << ":0]};\n"
          << "      ecc_encode = {parity, data};\n"
          << "    end\n"
          << "  endfunction\n\n"
          << "  function [" << (cfg.bits-1) << ":0] ecc_decode;\n"
          << "    input [" << (internal_bits-1) << ":0] codeword;\n"
          << "    reg [" << (cfg.bits-1) << ":0] data;\n"
          << "    reg [" << (r.ecc_bits-1) << ":0] syndrome;\n"
          << "    integer i;\n"
          << "    begin\n"
          << "      data = codeword[" << (cfg.bits-1) << ":0];\n"
          << "      syndrome = 0;\n"
          << "      for (i = 0; i < " << internal_bits << "; i = i + 1)\n"
          << "        if (codeword[i]) syndrome = syndrome ^ (i + 1);\n"
          << "      if (syndrome != 0 && syndrome <= " << cfg.bits << ")\n"
          << "        data[syndrome-1] = ~data[syndrome-1]; // single-bit correct\n"
          << "      ecc_decode = data;\n"
          << "    end\n"
          << "  endfunction\n";
    }

    // Main always block
    if (cfg.type == MemoryConfig::SRAM_2P) {
        // Dual-port: separate read and write
        v << "\n  always @(posedge CLK) begin\n";
        if (cfg.enable_power_gating) v << "    if (!PWR_GATE) begin\n";
        v << "    if (CE) begin\n"
          << "      if (WE)\n";
        if (cfg.enable_ecc)
            v << "        mem[WA] <= ecc_encode(D);\n";
        else
            v << "        mem[WA] <= D;\n";
        if (cfg.enable_ecc)
            v << "      Q <= ecc_decode(mem[RA]);\n";
        else
            v << "      Q <= mem[RA];\n";
        v << "    end\n";
        if (cfg.enable_power_gating) v << "    end\n";
        v << "  end\n";
    } else if (cfg.type == MemoryConfig::REGISTER_FILE && (cfg.read_ports > 1 || cfg.write_ports > 1)) {
        // Multi-port register file
        v << "\n  // Multi-port register file: " << cfg.read_ports << "R / " << cfg.write_ports << "W\n";
        for (int wp = 0; wp < cfg.write_ports; ++wp) {
            v << "  always @(posedge CLK) begin\n";
            if (cfg.enable_power_gating) v << "    if (!PWR_GATE)\n";
            v << "    if (CE && WE)\n"
              << "      mem[A] <= D;\n"
              << "  end\n";
        }
        v << "\n  always @(posedge CLK) begin\n";
        if (cfg.enable_power_gating) v << "    if (!PWR_GATE)\n";
        v << "    if (CE)\n";
        if (cfg.enable_ecc)
            v << "      Q <= ecc_decode(mem[A]);\n";
        else
            v << "      Q <= mem[A];\n";
        v << "  end\n";
    } else {
        // Single-port SRAM / ROM / simple RF
        v << "\n  always @(posedge CLK) begin\n";
        if (cfg.enable_power_gating) v << "    if (!PWR_GATE) begin\n";
        v << "    if (CE) begin\n";
        if (cfg.type != MemoryConfig::ROM) {
            v << "      if (WE)\n";
            if (cfg.enable_ecc)
                v << "        mem[A] <= ecc_encode(D);\n";
            else
                v << "        mem[A] <= D;\n";
        }
        if (cfg.enable_ecc)
            v << "      Q <= ecc_decode(mem[A]);\n";
        else
            v << "      Q <= mem[A];\n";
        v << "    end\n";
        if (cfg.enable_power_gating) v << "    end\n";
        v << "  end\n";
    }

    v << "\nendmodule\n";
    r.verilog_model = v.str();
}

// ---------------------------------------------------------------------------
// MBIST wrapper generation
// ---------------------------------------------------------------------------
void MemoryCompiler::generate_mbist_wrapper(const MemoryConfig& cfg, MemoryResult& r) {
    if (!cfg.enable_mbist) return;

    int addr_bits = (int)std::ceil(std::log2(cfg.words));
    std::ostringstream m;

    m << "// SiliconForge MBIST Wrapper — March C- Algorithm\n"
      << "module " << cfg.name << "_mbist (\n"
      << "  input  CLK, RST_N,\n"
      << "  input  TEST_MODE,\n"
      << "  output reg [" << (addr_bits-1) << ":0] TEST_ADDR,\n"
      << "  output reg [" << (cfg.bits-1) << ":0] TEST_DATA,\n"
      << "  output reg TEST_WE,\n"
      << "  output reg TEST_CE,\n"
      << "  output reg TEST_DONE,\n"
      << "  output reg TEST_FAIL\n"
      << ");\n\n"
      << "  // March C- FSM states\n"
      << "  localparam S_IDLE  = 3'd0,\n"
      << "             S_W0_UP = 3'd1,  // Write 0 ascending\n"
      << "             S_R0W1_UP = 3'd2,  // Read 0, Write 1 ascending\n"
      << "             S_R1W0_UP = 3'd3,  // Read 1, Write 0 ascending\n"
      << "             S_R0W1_DN = 3'd4,  // Read 0, Write 1 descending\n"
      << "             S_R1W0_DN = 3'd5,  // Read 1, Write 0 descending\n"
      << "             S_R0_UP   = 3'd6,  // Read 0 ascending (verify)\n"
      << "             S_DONE    = 3'd7;\n\n"
      << "  reg [2:0] state;\n"
      << "  reg [" << (addr_bits-1) << ":0] addr_cnt;\n"
      << "  reg direction; // 0=ascending, 1=descending\n\n"
      << "  always @(posedge CLK or negedge RST_N) begin\n"
      << "    if (!RST_N) begin\n"
      << "      state <= S_IDLE; TEST_DONE <= 0; TEST_FAIL <= 0;\n"
      << "    end else if (TEST_MODE) begin\n"
      << "      // March C- state machine (interface only — full RTL in synthesis)\n"
      << "      case (state)\n"
      << "        S_IDLE:  begin addr_cnt <= 0; state <= S_W0_UP; end\n"
      << "        S_DONE:  begin TEST_DONE <= 1; end\n"
      << "        default: begin\n"
      << "          TEST_ADDR <= addr_cnt;\n"
      << "          TEST_CE <= 1;\n"
      << "          if (addr_cnt == " << (cfg.words-1) << ")\n"
      << "            state <= state + 1;\n"
      << "          else\n"
      << "            addr_cnt <= addr_cnt + 1;\n"
      << "        end\n"
      << "      endcase\n"
      << "    end\n"
      << "  end\n\n"
      << "endmodule\n";

    r.verilog_model += "\n" + m.str();

    // MBIST report
    std::ostringstream rpt;
    long long total_ops = (long long)cfg.words * 10; // March C- ≈ 10n operations
    double test_cycles = (double)total_ops;
    double test_time_us = test_cycles * r.timing.tCK_ns * 0.001;

    rpt << "MBIST Report — " << cfg.name << "\n"
        << "  Algorithm: March C-\n"
        << "  Pattern: {⇑(w0); ⇑(r0,w1); ⇑(r1,w0); ⇓(r0,w1); ⇓(r1,w0); ⇑(r0)}\n"
        << "  Total operations: " << total_ops << "\n"
        << "  Estimated test time: " << std::fixed << std::setprecision(2) << test_time_us << " us\n"
        << "  Coverage: stuck-at-0, stuck-at-1, transition, coupling\n";
    r.mbist_report = rpt.str();
}

// ---------------------------------------------------------------------------
// Detailed characterization report
// ---------------------------------------------------------------------------
void MemoryCompiler::generate_detailed_report(const MemoryConfig& cfg, MemoryResult& r) {
    std::ostringstream rpt;
    auto fixed2 = [](double v) { std::ostringstream o; o << std::fixed << std::setprecision(4) << v; return o.str(); };

    const char* type_str = "SRAM_1P";
    switch (cfg.type) {
        case MemoryConfig::SRAM_2P:       type_str = "SRAM_2P"; break;
        case MemoryConfig::REGISTER_FILE: type_str = "REGISTER_FILE"; break;
        case MemoryConfig::ROM:           type_str = "ROM"; break;
        default: break;
    }
    const char* corner_str = "TT";
    switch (cfg.corner) {
        case MemoryConfig::FF: corner_str = "FF"; break;
        case MemoryConfig::SS: corner_str = "SS"; break;
        case MemoryConfig::SF: corner_str = "SF"; break;
        case MemoryConfig::FS: corner_str = "FS"; break;
        default: break;
    }

    rpt << "================================================================\n"
        << "  SiliconForge Memory Compiler — Characterization Report\n"
        << "================================================================\n"
        << "\n--- Configuration ---\n"
        << "  Name              : " << cfg.name << "\n"
        << "  Type              : " << type_str << "\n"
        << "  Words             : " << cfg.words << "\n"
        << "  Bits/word         : " << cfg.bits << "\n"
        << "  Column mux        : " << r.actual_column_mux << "\n"
        << "  Banks             : " << r.actual_banks << "\n"
        << "  ECC (SECDED)      : " << (cfg.enable_ecc ? "enabled" : "disabled") << "\n"
        << "  ECC bits/word     : " << r.ecc_bits << "\n"
        << "  Total bits/word   : " << r.total_bits_per_word << "\n"
        << "  Redundant rows    : " << r.total_redundant_rows << "\n"
        << "  Redundant cols    : " << r.total_redundant_cols << "\n"
        << "  MBIST             : " << (cfg.enable_mbist ? "enabled" : "disabled") << "\n"
        << "  Power gating      : " << (cfg.enable_power_gating ? "enabled" : "disabled") << "\n"
        << "  Retention         : " << (cfg.enable_retention ? "enabled" : "disabled") << "\n"
        << "  Corner            : " << corner_str << "\n"
        << "  Vdd               : " << fixed2(cfg.vdd) << " V\n"
        << "  Temperature       : " << fixed2(cfg.temperature) << " °C\n";

    rpt << "\n--- Area ---\n"
        << "  Width             : " << fixed2(r.width_um) << " um\n"
        << "  Height            : " << fixed2(r.height_um) << " um\n"
        << "  Area              : " << fixed2(r.area_um2) << " um²\n"
        << "  Bit density       : " << fixed2(r.bit_density) << " bits/um²\n"
        << "  Array rows        : " << r.total_rows << "\n"
        << "  Array cols        : " << r.total_cols << "\n";

    rpt << "\n--- Timing (corner=" << corner_str
        << ", derating=" << fixed2(r.timing_derating) << ") ---\n"
        << "  tAA               : " << fixed2(r.timing.tAA_ns) << " ns\n"
        << "  tCK               : " << fixed2(r.timing.tCK_ns) << " ns\n"
        << "  tCKH              : " << fixed2(r.timing.tCKH_ns) << " ns\n"
        << "  tCKL              : " << fixed2(r.timing.tCKL_ns) << " ns\n"
        << "  tSetup (tDS)      : " << fixed2(r.timing.tSetup_ns) << " ns\n"
        << "  tHold  (tDH)      : " << fixed2(r.timing.tHold_ns) << " ns\n"
        << "  tAS               : " << fixed2(r.timing.tAS_ns) << " ns\n"
        << "  tAH               : " << fixed2(r.timing.tAH_ns) << " ns\n"
        << "  tWES              : " << fixed2(r.timing.tWES_ns) << " ns\n"
        << "  tWEH              : " << fixed2(r.timing.tWEH_ns) << " ns\n"
        << "  tCD               : " << fixed2(r.timing.tCD_ns) << " ns\n"
        << "  tOE               : " << fixed2(r.timing.tOE_ns) << " ns\n"
        << "  tWC               : " << fixed2(r.timing.tWC_ns) << " ns\n";

    rpt << "\n--- Power ---\n"
        << "  Leakage           : " << fixed2(r.leakage_uw) << " uW\n"
        << "  Standby leakage   : " << fixed2(r.standby_leakage_uw) << " uW\n"
        << "  Retention leakage : " << fixed2(r.retention_leakage_uw) << " uW\n"
        << "  Dynamic power     : " << fixed2(r.dynamic_power_mw) << " mW (@500MHz est.)\n"
        << "  Read energy       : " << fixed2(r.read_energy_pj) << " pJ\n"
        << "  Write energy      : " << fixed2(r.write_energy_pj) << " pJ\n";

    if (!r.mbist_report.empty()) {
        rpt << "\n--- MBIST ---\n" << r.mbist_report;
    }

    rpt << "================================================================\n";
    r.detailed_report = rpt.str();
}

// ---------------------------------------------------------------------------
// Top-level compile flow
// ---------------------------------------------------------------------------
MemoryResult MemoryCompiler::compile(const MemoryConfig& cfg) {
    last_cfg_ = cfg;
    MemoryResult r;
    r.name = cfg.name;
    r.words = cfg.words;
    r.bits = cfg.bits;

    // 1. Banking / mux setup
    compute_banking(cfg, r);
    // 2. ECC bits
    compute_ecc(cfg, r);
    // 3. Redundancy
    compute_redundancy(cfg, r);
    // 4. Corner derating
    compute_corner_derating(cfg, r);
    // 5. Physical dimensions
    compute_dimensions(cfg, r);
    // 6. Timing
    compute_timing(cfg, r);
    // 7. Power
    compute_power(cfg, r);
    // 8. Functional netlist
    generate_netlist(cfg, r);
    // 9. Liberty model
    generate_liberty(cfg, r);
    // 10. Verilog model
    generate_verilog(cfg, r);
    // 11. MBIST wrapper
    generate_mbist_wrapper(cfg, r);
    // 12. Detailed report
    generate_detailed_report(cfg, r);

    r.success = true;
    r.message = cfg.name + ": " + std::to_string(cfg.words) + "×" +
                std::to_string(cfg.bits) + " " +
                (cfg.type == MemoryConfig::SRAM_1P ? "SRAM" :
                 cfg.type == MemoryConfig::SRAM_2P ? "SRAM_2P" :
                 cfg.type == MemoryConfig::REGISTER_FILE ? "RF" : "ROM") +
                " — " + std::to_string((int)r.width_um) + "×" +
                std::to_string((int)r.height_um) + "um, tAA=" +
                std::to_string(r.timing.tAA_ns) + "ns";
    return r;
}

// ---------------------------------------------------------------------------
// Multi-port SRAM generation
// ---------------------------------------------------------------------------
MultiPortResult MemoryCompiler::generate_multi_port(const MultiPortConfig& cfg) {
    MultiPortResult res{};
    int total_ports = cfg.read_ports + cfg.write_ports + cfg.rw_ports;
    double port_factor = 1.0 + (total_ports - 1) * 0.6;
    double cell_area = 0.5 * 1.0; // bit_cell_width * bit_cell_height in um²
    res.area = cfg.depth * cfg.word_width * cell_area * port_factor;

    double base_delay = 0.2 + std::log2(std::max(1, cfg.depth)) * 0.05;
    double width_factor = 1.0 + cfg.word_width / 256.0;
    res.read_delay_ns = base_delay * width_factor * (1.0 + (cfg.read_ports - 1) * 0.15);
    res.write_delay_ns = base_delay * width_factor * 1.1 * (1.0 + (cfg.write_ports - 1) * 0.15);

    int total_bits = cfg.depth * cfg.word_width;
    res.leakage_uw = total_bits * 0.001 * port_factor;

    int transistors_per_bit = 6;
    res.total_transistors = total_bits * transistors_per_bit * static_cast<int>(std::ceil(port_factor));
    return res;
}

// ---------------------------------------------------------------------------
// Liberty-format timing model generation
// ---------------------------------------------------------------------------
TimingModelResult MemoryCompiler::generate_timing_model(const MultiPortConfig& cfg) {
    TimingModelResult res{};
    int total_ports = cfg.read_ports + cfg.write_ports + cfg.rw_ports;
    res.cell_name = "SRAM_" + std::to_string(cfg.depth) + "x" +
                    std::to_string(cfg.word_width) + "_" +
                    std::to_string(total_ports) + "P";

    double depth_factor = std::log2(std::max(1, cfg.depth)) * 0.02;
    double width_factor = cfg.word_width / 512.0;
    res.setup_time = 0.05 + depth_factor;
    res.hold_time = 0.03 + depth_factor * 0.5;
    res.clk_to_q = 0.08 + depth_factor + width_factor * 0.01;
    res.read_access_time = 0.2 + depth_factor * 2.5 + width_factor * 0.05;
    res.write_access_time = res.read_access_time * 1.1;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "library (" << res.cell_name << "_lib) {\n";
    oss << "  cell (" << res.cell_name << ") {\n";
    oss << "    area : " << cfg.depth * cfg.word_width * 0.5 << " ;\n";
    oss << "    pin (CLK) {\n";
    oss << "      direction : input ;\n";
    oss << "      clock : true ;\n";
    oss << "    }\n";
    for (int i = 0; i < cfg.read_ports + cfg.rw_ports; ++i) {
        std::string port_name = "Q" + std::to_string(i);
        oss << "    pin (" << port_name << ") {\n";
        oss << "      direction : output ;\n";
        oss << "      timing () {\n";
        oss << "        related_pin : CLK ;\n";
        oss << "        timing_type : rising_edge ;\n";
        oss << "        cell_rise (scalar) { values(\"" << res.clk_to_q << "\"); }\n";
        oss << "        cell_fall (scalar) { values(\"" << res.clk_to_q << "\"); }\n";
        oss << "      }\n";
        oss << "    }\n";
    }
    for (int i = 0; i < cfg.write_ports + cfg.rw_ports; ++i) {
        std::string port_name = "D" + std::to_string(i);
        oss << "    pin (" << port_name << ") {\n";
        oss << "      direction : input ;\n";
        oss << "      timing () {\n";
        oss << "        related_pin : CLK ;\n";
        oss << "        timing_type : setup_rising ;\n";
        oss << "        rise_constraint (scalar) { values(\"" << res.setup_time << "\"); }\n";
        oss << "        fall_constraint (scalar) { values(\"" << res.setup_time << "\"); }\n";
        oss << "      }\n";
        oss << "      timing () {\n";
        oss << "        related_pin : CLK ;\n";
        oss << "        timing_type : hold_rising ;\n";
        oss << "        rise_constraint (scalar) { values(\"" << res.hold_time << "\"); }\n";
        oss << "        fall_constraint (scalar) { values(\"" << res.hold_time << "\"); }\n";
        oss << "      }\n";
        oss << "    }\n";
    }
    oss << "  }\n";
    oss << "}\n";
    res.lib_format = oss.str();
    return res;
}

// ---------------------------------------------------------------------------
// Column/row redundancy for yield improvement
// ---------------------------------------------------------------------------
RedundancyResult MemoryCompiler::add_redundancy(int spare_cols, int spare_rows) {
    RedundancyResult res{};
    res.spare_cols = spare_cols;
    res.spare_rows = spare_rows;
    double yield_pct = spare_cols * 3.5 + spare_rows * 4.0;
    res.yield_improvement_pct = std::min(yield_pct, 25.0);
    double col_overhead = (last_cfg_.bits > 0)
        ? (static_cast<double>(spare_cols) / last_cfg_.bits) * 100.0
        : spare_cols * 0.5;
    double row_overhead = (last_cfg_.words > 0)
        ? (static_cast<double>(spare_rows) / last_cfg_.words) * 100.0
        : spare_rows * 0.5;
    res.area_overhead_pct = col_overhead + row_overhead;
    return res;
}

// ---------------------------------------------------------------------------
// BIST wrapper generation (March algorithms)
// ---------------------------------------------------------------------------
BistWrapperResult MemoryCompiler::generate_bist_wrapper(bool with_repair) {
    BistWrapperResult res{};
    res.wrapper_name = "MBIST_" + last_cfg_.name + "_wrapper";
    // March C-, March LR, MATS+, Walking 1/0
    res.march_algorithms = 4;
    // March C- complexity: 14N where N = number of words (depth)
    int depth = std::max(1, last_cfg_.words);
    res.test_time_cycles = 14.0 * depth;
    res.self_repair = with_repair;
    return res;
}

// ---------------------------------------------------------------------------
// Enhanced compilation: compile + multi-port + redundancy + BIST
// ---------------------------------------------------------------------------
MemoryResult MemoryCompiler::run_enhanced(const MemoryConfig& cfg) {
    MemoryResult r = compile(cfg);

    MultiPortConfig mpc{};
    mpc.read_ports = cfg.read_ports;
    mpc.write_ports = cfg.write_ports;
    mpc.rw_ports = 0;
    mpc.word_width = cfg.bits;
    mpc.depth = cfg.words;
    mpc.has_byte_enable = false;
    generate_multi_port(mpc);

    add_redundancy(std::max(1, cfg.redundant_cols), std::max(1, cfg.redundant_rows));

    generate_bist_wrapper(cfg.enable_mbist);

    return r;
}

} // namespace sf
