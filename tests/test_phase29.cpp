// SiliconForge — Phase 29: JTAG/BIST Industrial Tests
// IEEE 1149.1 TAP, Boundary Scan, LFSR, MISR, LBIST, MBIST, BSDL, DFT Integration
#include "dft/jtag_bist.hpp"
#include <iostream>
#include <cassert>
#include <cstring>
#include <set>

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

using namespace sf;

// ── TAP State Machine Tests ─────────────────────────────────────────────────

TEST(tap_initial_state) {
    TapController tap;
    CHECK(tap.state() == TapState::TEST_LOGIC_RESET, "Initial state should be TLR");
    PASS("tap_initial_state");
}

TEST(tap_reset) {
    TapController tap;
    // Move away from TLR
    tap.clock_tms(false); // → Run-Test/Idle
    CHECK(tap.state() == TapState::RUN_TEST_IDLE, "Should be RTI after TMS=0");
    tap.reset();
    CHECK(tap.state() == TapState::TEST_LOGIC_RESET, "Reset should return to TLR");
    PASS("tap_reset");
}

TEST(tap_dr_scan_path) {
    TapController tap;
    tap.clock_tms(false); // TLR → RTI
    tap.clock_tms(true);  // RTI → Select-DR-Scan
    CHECK(tap.state() == TapState::SELECT_DR_SCAN, "Select-DR-Scan");
    tap.clock_tms(false); // → Capture-DR
    CHECK(tap.state() == TapState::CAPTURE_DR, "Capture-DR");
    tap.clock_tms(false); // → Shift-DR
    CHECK(tap.state() == TapState::SHIFT_DR, "Shift-DR");
    tap.clock_tms(true);  // → Exit1-DR
    CHECK(tap.state() == TapState::EXIT1_DR, "Exit1-DR");
    tap.clock_tms(false); // → Pause-DR
    CHECK(tap.state() == TapState::PAUSE_DR, "Pause-DR");
    tap.clock_tms(true);  // → Exit2-DR
    CHECK(tap.state() == TapState::EXIT2_DR, "Exit2-DR");
    tap.clock_tms(true);  // → Update-DR
    CHECK(tap.state() == TapState::UPDATE_DR, "Update-DR");
    PASS("tap_dr_scan_path");
}

TEST(tap_ir_scan_path) {
    TapController tap;
    tap.clock_tms(false); // → RTI
    tap.clock_tms(true);  // → Select-DR-Scan
    tap.clock_tms(true);  // → Select-IR-Scan
    CHECK(tap.state() == TapState::SELECT_IR_SCAN, "Select-IR-Scan");
    tap.clock_tms(false); // → Capture-IR
    CHECK(tap.state() == TapState::CAPTURE_IR, "Capture-IR");
    tap.clock_tms(false); // → Shift-IR
    CHECK(tap.state() == TapState::SHIFT_IR, "Shift-IR");
    tap.clock_tms(true);  // → Exit1-IR
    CHECK(tap.state() == TapState::EXIT1_IR, "Exit1-IR");
    tap.clock_tms(false); // → Pause-IR
    CHECK(tap.state() == TapState::PAUSE_IR, "Pause-IR");
    tap.clock_tms(true);  // → Exit2-IR
    CHECK(tap.state() == TapState::EXIT2_IR, "Exit2-IR");
    tap.clock_tms(true);  // → Update-IR
    CHECK(tap.state() == TapState::UPDATE_IR, "Update-IR");
    PASS("tap_ir_scan_path");
}

TEST(tap_tlr_hold) {
    TapController tap;
    // TMS=1 should keep us in TLR
    for (int i = 0; i < 10; i++) tap.clock_tms(true);
    CHECK(tap.state() == TapState::TEST_LOGIC_RESET, "TLR should hold on TMS=1");
    PASS("tap_tlr_hold");
}

TEST(tap_rti_hold) {
    TapController tap;
    tap.clock_tms(false); // → RTI
    for (int i = 0; i < 5; i++) tap.clock_tms(false);
    CHECK(tap.state() == TapState::RUN_TEST_IDLE, "RTI should hold on TMS=0");
    PASS("tap_rti_hold");
}

// ── JTAG Controller Tests ───────────────────────────────────────────────────

TEST(jtag_idcode) {
    JtagConfig cfg;
    cfg.idcode = 0xDEADBEEF;
    JtagController jtag(cfg);
    uint32_t id = jtag.read_idcode();
    CHECK(id == 0xDEADBEEF, "IDCODE should match config");
    PASS("jtag_idcode");
}

TEST(jtag_bypass) {
    JtagController jtag;
    bool ok = jtag.bypass_test();
    CHECK(ok, "Bypass test should pass (1-bit delay)");
    PASS("jtag_bypass");
}

TEST(jtag_ir_shift) {
    JtagController jtag;
    // Shift EXTEST instruction (opcode 0)
    std::vector<bool> ir = {false, false, false, false}; // EXTEST = 0000
    jtag.shift_ir(ir);
    CHECK(jtag.current_instruction() == JtagInstruction::EXTEST, "Should decode EXTEST");
    PASS("jtag_ir_shift");
}

TEST(jtag_dr_shift) {
    JtagController jtag;
    // Shift some data through DR
    std::vector<bool> data = {true, false, true, true, false, false, true, false};
    auto out = jtag.shift_dr(data);
    CHECK(out.size() == data.size(), "DR output should be same length as input");
    PASS("jtag_dr_shift");
}

// ── Boundary Scan Register Tests ────────────────────────────────────────────

TEST(bsr_add_cells) {
    BoundaryScanRegister bsr;
    bsr.add_cell({BoundaryCellType::BC_1, "PIN_A", "input"});
    bsr.add_cell({BoundaryCellType::BC_2, "PIN_B", "output"});
    bsr.add_cell({BoundaryCellType::BC_7, "PIN_C", "bidir"});
    CHECK(bsr.size() == 3, "BSR should have 3 cells");
    PASS("bsr_add_cells");
}

TEST(bsr_capture_shift_update) {
    BoundaryScanRegister bsr;
    bsr.add_cell({BoundaryCellType::BC_1, "IN0", "input"});
    bsr.add_cell({BoundaryCellType::BC_1, "IN1", "input"});
    bsr.add_cell({BoundaryCellType::BC_2, "OUT0", "output"});

    // Set parallel inputs
    bsr.set_parallel_inputs({true, false, true});
    bsr.capture();

    // Shift out 3 bits
    bool tdo0 = bsr.shift(false);
    bool tdo1 = bsr.shift(true);
    bool tdo2 = bsr.shift(false);
    (void)tdo0; (void)tdo1; (void)tdo2;

    bsr.update();
    auto outputs = bsr.get_parallel_outputs();
    CHECK(outputs.size() == 3, "Should have 3 parallel outputs after update");
    PASS("bsr_capture_shift_update");
}

// ── LFSR Tests ──────────────────────────────────────────────────────────────

TEST(lfsr_standard_polynomials) {
    CHECK(Lfsr::standard_polynomial(8) != 0, "8-bit polynomial should be non-zero");
    CHECK(Lfsr::standard_polynomial(16) != 0, "16-bit polynomial should be non-zero");
    CHECK(Lfsr::standard_polynomial(32) != 0, "32-bit polynomial should be non-zero");
    PASS("lfsr_standard_polynomials");
}

TEST(lfsr_sequence) {
    LfsrConfig cfg;
    cfg.width = 8;
    cfg.seed = 1;
    Lfsr lfsr(cfg);

    uint32_t first = lfsr.value();
    CHECK(first == 1, "Initial value should be seed");

    // Clock it and verify it changes
    lfsr.clock();
    uint32_t second = lfsr.value();
    CHECK(second != first, "LFSR should change after clock");

    // Verify it cycles (8-bit maximal length = 255 states)
    lfsr.reset();
    std::set<uint32_t> seen;
    for (int i = 0; i < 255; i++) {
        seen.insert(lfsr.value());
        lfsr.clock();
    }
    CHECK(seen.size() >= 200, "8-bit LFSR should visit many states");
    PASS("lfsr_sequence");
}

TEST(lfsr_get_pattern) {
    LfsrConfig cfg;
    cfg.width = 16;
    Lfsr lfsr(cfg);
    auto pat = lfsr.get_pattern(16);
    CHECK((int)pat.size() == 16, "Pattern should have 16 bits");
    PASS("lfsr_get_pattern");
}

TEST(lfsr_reset) {
    LfsrConfig cfg;
    cfg.width = 16;
    cfg.seed = 0xABCD;
    Lfsr lfsr(cfg);
    lfsr.clock();
    lfsr.clock();
    lfsr.reset();
    CHECK(lfsr.value() == 0xABCD, "Reset should restore seed");
    PASS("lfsr_reset");
}

// ── MISR Tests ──────────────────────────────────────────────────────────────

TEST(misr_compress) {
    Misr misr(16);
    std::vector<bool> response1 = {true, false, true, true, false, true, false, false};
    misr.compress(response1);
    uint32_t sig1 = misr.signature();
    CHECK(sig1 != 0, "Signature should be non-zero after compression");

    misr.compress(response1);
    uint32_t sig2 = misr.signature();
    CHECK(sig2 != sig1, "Signature should change with more data");
    PASS("misr_compress");
}

TEST(misr_deterministic) {
    Misr misr1(16), misr2(16);
    std::vector<bool> response = {true, false, true, true};
    misr1.compress(response);
    misr2.compress(response);
    CHECK(misr1.signature() == misr2.signature(), "Same input should give same signature");
    PASS("misr_deterministic");
}

TEST(misr_reset) {
    Misr misr(16);
    misr.compress({true, true, false});
    CHECK(misr.signature() != 0, "Signature non-zero before reset");
    misr.reset();
    CHECK(misr.signature() == 0, "Signature should be 0 after reset");
    PASS("misr_reset");
}

// ── LBIST Tests ─────────────────────────────────────────────────────────────

TEST(lbist_run) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId c = nl.add_net("c"); nl.mark_output(c);
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId w1 = nl.add_net("w1");
    nl.add_gate(GateType::AND, {a, b}, w1, "g1");
    NetId w2 = nl.add_net("w2");
    nl.add_dff(w1, clk, w2, -1, "ff1");
    nl.add_gate(GateType::BUF, {w2}, c, "g2");

    LbistConfig cfg;
    cfg.num_patterns = 100;
    cfg.scan_chain_length = 10;
    LbistController lbist;
    LbistResult result = lbist.run(nl, cfg);
    CHECK(result.patterns_applied > 0, "Should apply patterns");
    CHECK(result.signature != 0 || result.patterns_applied > 0, "Should produce a result");
    CHECK(!result.report.empty(), "Should have a report");
    PASS("lbist_run");
}

// ── MBIST Tests ─────────────────────────────────────────────────────────────

TEST(mbist_march_c_minus) {
    MbistConfig cfg;
    cfg.words = 64;
    cfg.bits = 8;
    cfg.algo = MarchAlgorithm::MARCH_C_MINUS;
    cfg.mem_name = "sram0";

    MbistController mbist;
    MbistResult result = mbist.run(cfg);
    // MBIST injects faults to demonstrate detection capability
    CHECK(result.faults_detected > 0, "March C- should detect injected faults");
    CHECK(!result.pass, "Memory with injected faults should fail");
    CHECK(!result.report.empty(), "Should have a report");
    CHECK(!result.fault_locations.empty(), "Should report fault locations");
    PASS("mbist_march_c_minus");
}

TEST(mbist_march_x) {
    MbistConfig cfg;
    cfg.words = 32;
    cfg.bits = 4;
    cfg.algo = MarchAlgorithm::MARCH_X;

    MbistController mbist;
    MbistResult result = mbist.run(cfg);
    CHECK(result.faults_detected > 0, "March X should detect injected faults");
    CHECK(!result.report.empty(), "Should have a report");
    PASS("mbist_march_x");
}

TEST(mbist_checkerboard) {
    MbistConfig cfg;
    cfg.words = 16;
    cfg.bits = 8;
    cfg.algo = MarchAlgorithm::CHECKERBOARD;

    MbistController mbist;
    MbistResult result = mbist.run(cfg);
    CHECK(result.faults_detected > 0, "Checkerboard should detect injected faults");
    CHECK(!result.report.empty(), "Should have a report");
    PASS("mbist_checkerboard");
}

TEST(mbist_different_sizes) {
    MbistController mbist;
    // Small memory — fault injection scales with size
    MbistResult r1 = mbist.run({16, 4, MarchAlgorithm::MARCH_C_MINUS, "tiny"});
    CHECK(r1.faults_detected > 0, "Small memory should detect faults");
    // Larger memory
    MbistResult r2 = mbist.run({256, 16, MarchAlgorithm::MARCH_C_MINUS, "medium"});
    CHECK(r2.faults_detected > 0, "Medium memory should detect faults");
    CHECK(r2.faults_detected >= r1.faults_detected, "Larger memory should detect more faults");
    PASS("mbist_different_sizes");
}

// ── BSDL Generator Tests ───────────────────────────────────────────────────

TEST(bsdl_generate) {
    JtagConfig cfg;
    cfg.device_name = "test_chip";
    cfg.ir_length = 4;
    cfg.idcode = 0x12345678;

    std::vector<BoundaryCell> cells = {
        {BoundaryCellType::BC_1, "CLK", "input"},
        {BoundaryCellType::BC_1, "RST", "input"},
        {BoundaryCellType::BC_2, "DOUT0", "output"},
        {BoundaryCellType::BC_7, "GPIO0", "bidir"}
    };

    BsdlGenerator gen;
    std::string bsdl = gen.generate(cfg, cells);
    CHECK(!bsdl.empty(), "BSDL should not be empty");
    CHECK(bsdl.find("TEST_CHIP") != std::string::npos || 
          bsdl.find("test_chip") != std::string::npos, "Should contain device name");
    CHECK(bsdl.find("INSTRUCTION") != std::string::npos, "Should contain instruction info");
    CHECK(bsdl.find("BOUNDARY") != std::string::npos || 
          bsdl.find("boundary") != std::string::npos, "Should contain boundary info");
    PASS("bsdl_generate");
}

// ── DFT Integration Tests ──────────────────────────────────────────────────

TEST(dft_full_integration) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId q = nl.add_net("q"); nl.mark_output(q);

    nl.add_gate(GateType::AND, {a, b}, w1, "g1");
    nl.add_dff(w1, clk, w2, -1, "ff1");
    nl.add_gate(GateType::BUF, {w2}, q, "out_buf");

    DftConfig cfg;
    cfg.scan.max_chain_length = 50;
    cfg.jtag.device_name = "integration_test";
    cfg.lbist.num_patterns = 50;
    cfg.mbist.push_back({32, 8, MarchAlgorithm::MARCH_C_MINUS, "test_mem"});

    DftIntegrator integrator;
    DftResult result = integrator.run_full_dft(nl, cfg);
    CHECK(result.scan.num_chains >= 0, "Scan should produce results");
    CHECK(!result.bsdl.empty(), "BSDL should be generated");
    CHECK(!result.report.empty(), "Report should be generated");
    CHECK(result.coverage_estimate >= 0.0, "Coverage should be non-negative");
    PASS("dft_full_integration");
}

int main() {
    std::cout << "═══════════════════════════════════════════════════════\n"
              << " Phase 29: JTAG/BIST Industrial Tests\n"
              << "═══════════════════════════════════════════════════════\n\n";

    // TAP State Machine
    RUN(tap_initial_state);
    RUN(tap_reset);
    RUN(tap_dr_scan_path);
    RUN(tap_ir_scan_path);
    RUN(tap_tlr_hold);
    RUN(tap_rti_hold);

    // JTAG Controller
    RUN(jtag_idcode);
    RUN(jtag_bypass);
    RUN(jtag_ir_shift);
    RUN(jtag_dr_shift);

    // Boundary Scan Register
    RUN(bsr_add_cells);
    RUN(bsr_capture_shift_update);

    // LFSR
    RUN(lfsr_standard_polynomials);
    RUN(lfsr_sequence);
    RUN(lfsr_get_pattern);
    RUN(lfsr_reset);

    // MISR
    RUN(misr_compress);
    RUN(misr_deterministic);
    RUN(misr_reset);

    // LBIST
    RUN(lbist_run);

    // MBIST
    RUN(mbist_march_c_minus);
    RUN(mbist_march_x);
    RUN(mbist_checkerboard);
    RUN(mbist_different_sizes);

    // BSDL
    RUN(bsdl_generate);

    // Full DFT Integration
    RUN(dft_full_integration);

    std::cout << "\n═══════════════════════════════════════════════════════\n"
              << " Results: " << passed << " passed, " << failed << " failed\n"
              << "═══════════════════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
