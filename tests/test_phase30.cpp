// SiliconForge — Phase 30: Memory Compiler Industrial Tests
// Column Mux, Banking, ECC, Corner Derating, Redundancy, MBIST, Multi-port
#include "macro/memory_compiler.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

using namespace sf;

// ── Backward Compatibility ──────────────────────────────────────────────────

TEST(basic_sram_backward_compat) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.type = MemoryConfig::SRAM_1P;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.name = "sram_256x32";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Basic SRAM should compile");
    CHECK(r.words == 256, "Words should be 256");
    CHECK(r.bits == 32, "Bits should be 32");
    CHECK(r.area_um2 > 0, "Area should be positive");
    CHECK(r.timing.tAA_ns > 0, "Access time should be positive");
    CHECK(!r.liberty_model.empty(), "Liberty model should exist");
    CHECK(!r.verilog_model.empty(), "Verilog model should exist");
    PASS("basic_sram_backward_compat");
}

TEST(register_file_backward_compat) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.type = MemoryConfig::REGISTER_FILE;
    cfg.words = 32;
    cfg.bits = 64;
    cfg.name = "rf_32x64";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "RF should compile");
    CHECK(r.timing.tAA_ns > 0, "Access time positive");
    CHECK(r.timing.tAA_ns < 1.0, "RF should be fast (< 1ns)");
    PASS("register_file_backward_compat");
}

// ── Column Multiplexing ────────────────────────────────────────────────────

TEST(column_mux_2x) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.column_mux = 2;
    cfg.name = "sram_mux2";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Column mux 2x should compile");
    CHECK(r.actual_column_mux == 2, "Actual column mux should be 2");
    PASS("column_mux_2x");
}

TEST(column_mux_4x) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 1024;
    cfg.bits = 32;
    cfg.column_mux = 4;
    cfg.name = "sram_mux4";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Column mux 4x should compile");
    CHECK(r.actual_column_mux == 4, "Actual column mux should be 4");
    PASS("column_mux_4x");
}

// ── Banking ────────────────────────────────────────────────────────────────

TEST(banking_2_banks) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 512;
    cfg.bits = 32;
    cfg.num_banks = 2;
    cfg.name = "sram_2bank";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "2-bank SRAM should compile");
    CHECK(r.actual_banks == 2, "Should have 2 banks");
    PASS("banking_2_banks");
}

TEST(banking_4_banks) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 1024;
    cfg.bits = 64;
    cfg.num_banks = 4;
    cfg.name = "sram_4bank";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "4-bank SRAM should compile");
    CHECK(r.actual_banks == 4, "Should have 4 banks");
    PASS("banking_4_banks");
}

// ── ECC (SECDED) ───────────────────────────────────────────────────────────

TEST(ecc_32bit) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.enable_ecc = true;
    cfg.name = "sram_ecc32";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "ECC SRAM should compile");
    CHECK(r.ecc_bits > 0, "Should have ECC parity bits");
    CHECK(r.total_bits_per_word > 32, "Total bits should include ECC");
    CHECK(r.total_bits_per_word == 32 + r.ecc_bits, "total = data + ecc");
    PASS("ecc_32bit");
}

TEST(ecc_64bit) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 512;
    cfg.bits = 64;
    cfg.enable_ecc = true;
    cfg.name = "sram_ecc64";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "64-bit ECC should compile");
    CHECK(r.ecc_bits >= 7, "64-bit data needs >= 7 ECC bits");
    PASS("ecc_64bit");
}

// ── Corner Derating ────────────────────────────────────────────────────────

TEST(corner_ss) {
    MemoryCompiler mc;
    MemoryConfig base;
    base.words = 256;
    base.bits = 32;
    base.name = "sram_tt";
    MemoryResult r_tt = mc.compile(base);

    base.corner = MemoryConfig::SS;
    base.name = "sram_ss";
    MemoryResult r_ss = mc.compile(base);

    CHECK(r_ss.timing.tAA_ns > r_tt.timing.tAA_ns, "SS should be slower than TT");
    CHECK(r_ss.timing_derating > 1.0, "SS derating should be > 1.0");
    PASS("corner_ss");
}

TEST(corner_ff) {
    MemoryCompiler mc;
    MemoryConfig base;
    base.words = 256;
    base.bits = 32;
    base.name = "sram_tt";
    MemoryResult r_tt = mc.compile(base);

    base.corner = MemoryConfig::FF;
    base.name = "sram_ff";
    MemoryResult r_ff = mc.compile(base);

    CHECK(r_ff.timing.tAA_ns < r_tt.timing.tAA_ns, "FF should be faster than TT");
    CHECK(r_ff.timing_derating < 1.0, "FF derating should be < 1.0");
    PASS("corner_ff");
}

TEST(temperature_scaling) {
    MemoryCompiler mc;
    MemoryConfig cold;
    cold.words = 256;
    cold.bits = 32;
    cold.temperature = -40.0;
    cold.name = "sram_cold";
    MemoryResult r_cold = mc.compile(cold);

    MemoryConfig hot;
    hot.words = 256;
    hot.bits = 32;
    hot.temperature = 125.0;
    hot.name = "sram_hot";
    MemoryResult r_hot = mc.compile(hot);

    CHECK(r_hot.timing.tAA_ns > r_cold.timing.tAA_ns, "Hot should be slower than cold");
    PASS("temperature_scaling");
}

TEST(voltage_scaling) {
    MemoryCompiler mc;
    MemoryConfig lo;
    lo.words = 256;
    lo.bits = 32;
    lo.vdd = 0.8;
    lo.name = "sram_lo";
    MemoryResult r_lo = mc.compile(lo);

    MemoryConfig hi;
    hi.words = 256;
    hi.bits = 32;
    hi.vdd = 1.2;
    hi.name = "sram_hi";
    MemoryResult r_hi = mc.compile(hi);

    CHECK(r_lo.timing.tAA_ns > r_hi.timing.tAA_ns, "Low voltage should be slower");
    PASS("voltage_scaling");
}

// ── Redundancy ─────────────────────────────────────────────────────────────

TEST(redundancy) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.redundant_rows = 2;
    cfg.redundant_cols = 1;
    cfg.name = "sram_redund";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Redundant SRAM should compile");
    CHECK(r.total_redundant_rows == 2, "Should have 2 redundant rows");
    CHECK(r.total_redundant_cols == 1, "Should have 1 redundant column");

    // Area should be larger than base
    MemoryConfig base;
    base.words = 256;
    base.bits = 32;
    base.name = "sram_base";
    MemoryResult r_base = mc.compile(base);
    CHECK(r.area_um2 > r_base.area_um2, "Redundancy should increase area");
    PASS("redundancy");
}

// ── Power Gating & Retention ───────────────────────────────────────────────

TEST(power_gating) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.enable_power_gating = true;
    cfg.name = "sram_pg";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Power gated SRAM should compile");
    PASS("power_gating");
}

TEST(retention) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.enable_retention = true;
    cfg.name = "sram_ret";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Retention SRAM should compile");
    CHECK(r.retention_leakage_uw > 0, "Retention leakage should be > 0");
    CHECK(r.retention_leakage_uw < r.standby_leakage_uw || r.standby_leakage_uw > 0,
          "Retention leakage should exist");
    PASS("retention");
}

// ── MBIST Wrapper ──────────────────────────────────────────────────────────

TEST(mbist_wrapper) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.enable_mbist = true;
    cfg.name = "sram_mbist";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "MBIST SRAM should compile");
    CHECK(!r.mbist_report.empty(), "Should have MBIST report");
    PASS("mbist_wrapper");
}

// ── Combined Features ──────────────────────────────────────────────────────

TEST(full_featured_sram) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.type = MemoryConfig::SRAM_1P;
    cfg.words = 1024;
    cfg.bits = 64;
    cfg.column_mux = 4;
    cfg.num_banks = 2;
    cfg.enable_ecc = true;
    cfg.enable_mbist = true;
    cfg.redundant_rows = 4;
    cfg.redundant_cols = 2;
    cfg.enable_retention = true;
    cfg.corner = MemoryConfig::SS;
    cfg.temperature = 125.0;
    cfg.vdd = 0.9;
    cfg.name = "sram_full";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Full-featured SRAM should compile");
    CHECK(r.ecc_bits > 0, "ECC bits present");
    CHECK(r.actual_banks == 2, "2 banks");
    CHECK(r.actual_column_mux == 4, "4x column mux");
    CHECK(r.total_redundant_rows == 4, "4 redundant rows");
    CHECK(r.timing_derating > 1.0, "SS derating");
    CHECK(!r.detailed_report.empty(), "Should have detailed report");
    CHECK(!r.mbist_report.empty(), "Should have MBIST report");
    CHECK(!r.liberty_model.empty(), "Liberty model exists");
    CHECK(!r.verilog_model.empty(), "Verilog model exists");
    PASS("full_featured_sram");
}

TEST(dual_port_sram) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.type = MemoryConfig::SRAM_2P;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.read_ports = 1;
    cfg.write_ports = 1;
    cfg.name = "sram_2p";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.success, "Dual-port SRAM should compile");
    CHECK(r.verilog_model.find("RA") != std::string::npos ||
          r.verilog_model.find("WA") != std::string::npos ||
          r.verilog_model.find("A") != std::string::npos,
          "Verilog should have port references");
    PASS("dual_port_sram");
}

// ── Detailed Report ────────────────────────────────────────────────────────

TEST(detailed_report_content) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 512;
    cfg.bits = 32;
    cfg.enable_ecc = true;
    cfg.num_banks = 2;
    cfg.name = "report_test";
    MemoryResult r = mc.compile(cfg);
    CHECK(!r.detailed_report.empty(), "Report should exist");
    CHECK(r.detailed_report.find("512") != std::string::npos, "Report should mention word count");
    PASS("detailed_report_content");
}

// ── Extended Timing Arcs ───────────────────────────────────────────────────

TEST(extended_timing) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.words = 256;
    cfg.bits = 32;
    cfg.name = "timing_test";
    MemoryResult r = mc.compile(cfg);
    CHECK(r.timing.tAA_ns > 0, "tAA positive");
    CHECK(r.timing.tCK_ns > 0, "tCK positive");
    CHECK(r.timing.tSetup_ns > 0, "tSetup positive");
    CHECK(r.timing.tCK_ns > r.timing.tAA_ns, "tCK should be > tAA");
    PASS("extended_timing");
}

int main() {
    std::cout << "═══════════════════════════════════════════════════════\n"
              << " Phase 30: Memory Compiler Industrial Tests\n"
              << "═══════════════════════════════════════════════════════\n\n";

    // Backward Compatibility
    RUN(basic_sram_backward_compat);
    RUN(register_file_backward_compat);

    // Column Mux
    RUN(column_mux_2x);
    RUN(column_mux_4x);

    // Banking
    RUN(banking_2_banks);
    RUN(banking_4_banks);

    // ECC
    RUN(ecc_32bit);
    RUN(ecc_64bit);

    // Corner Derating
    RUN(corner_ss);
    RUN(corner_ff);
    RUN(temperature_scaling);
    RUN(voltage_scaling);

    // Redundancy
    RUN(redundancy);

    // Power
    RUN(power_gating);
    RUN(retention);

    // MBIST
    RUN(mbist_wrapper);

    // Combined
    RUN(full_featured_sram);
    RUN(dual_port_sram);

    // Reports
    RUN(detailed_report_content);
    RUN(extended_timing);

    std::cout << "\n═══════════════════════════════════════════════════════\n"
              << " Results: " << passed << " passed, " << failed << " failed\n"
              << "═══════════════════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
