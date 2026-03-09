// SiliconForge — Phase 8 Test Suite
// Tests: Memory Compiler, MCMM, Detailed Router, Noise, Chip Assembler

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/detailed_router.hpp"
#include "pnr/chip_assembler.hpp"
#include "macro/memory_compiler.hpp"
#include "timing/mcmm.hpp"
#include "timing/noise.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static PhysicalDesign build_placed(int n = 15) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < n; ++i)
        pd.add_cell("c" + std::to_string(i), "AND2", 3.0, 10.0);
    for (int i = 0; i < n - 1; ++i)
        pd.add_net("n" + std::to_string(i), {i, i+1});
    AnalyticalPlacer placer(pd);
    placer.place();
    return pd;
}

static Netlist build_circuit() {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId c = nl.add_net("c"); nl.mark_input(c);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, w1, "G1");
    nl.add_gate(GateType::OR, {w1, c}, w2, "G2");
    nl.add_gate(GateType::NOT, {w2}, y, "G3");
    return nl;
}

// ============================================================================
// Memory Compiler Tests
// ============================================================================
TEST(mem_sram) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.name = "sram_256x32";
    cfg.type = MemoryConfig::SRAM_1P;
    cfg.words = 256;
    cfg.bits = 32;

    auto r = mc.compile(cfg);
    CHECK(r.success, "compile success");
    CHECK(r.words == 256 && r.bits == 32, "dimensions");
    CHECK(r.area_um2 > 0, "area computed");
    CHECK(r.timing.tAA_ns > 0, "access time");
    CHECK(r.timing.tCK_ns > r.timing.tAA_ns, "cycle > access");
    CHECK(r.leakage_uw > 0, "leakage power");
    CHECK(r.read_energy_pj > 0, "read energy");
    PASS("mem_sram");
}

TEST(mem_regfile) {
    MemoryCompiler mc;
    MemoryConfig cfg;
    cfg.name = "rf_32x64";
    cfg.type = MemoryConfig::REGISTER_FILE;
    cfg.words = 32;
    cfg.bits = 64;

    auto r = mc.compile(cfg);
    CHECK(r.success, "RF compile");
    CHECK(r.timing.tAA_ns < 0.5, "RF faster than SRAM");
    CHECK(r.verilog_model.find("module rf_32x64") != std::string::npos, "Verilog generated");
    CHECK(r.liberty_model.find("cell(rf_32x64)") != std::string::npos, "Liberty generated");
    PASS("mem_regfile");
}

// ============================================================================
// MCMM Tests
// ============================================================================
TEST(mcmm_default) {
    auto nl = build_circuit();
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners(); // 3 corners
    mcmm.load_default_modes();   // 3 modes

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 9, "3 corners × 3 modes = 9 scenarios");
    CHECK(r.worst_corner.size() > 0, "worst corner identified");
    CHECK(r.max_power_mw > 0, "max power found");
    PASS("mcmm_default");
}

TEST(mcmm_custom) {
    auto nl = build_circuit();
    McmmAnalyzer mcmm(nl);
    mcmm.add_corner({"slow", 0.81, 125, PvtCorner::SLOW, 1.5, 0.8});
    mcmm.add_corner({"fast", 0.99, -40, PvtCorner::FAST, 0.7, 1.3});
    mcmm.add_mode({"func", 1000, 0.2, "Functional"});

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 2, "2 corners × 1 mode");
    CHECK(r.details.size() == 2, "2 scenario details");
    PASS("mcmm_custom");
}

// ============================================================================
// Detailed Router Tests
// ============================================================================
TEST(detailed_route_small) {
    auto pd = build_placed(10);
    DetailedRouter dr(pd, 4, 2.0);
    auto r = dr.route();

    CHECK(r.routed_nets > 0, "nets routed");
    CHECK(r.total_wirelength > 0, "wirelength > 0");
    CHECK(r.total_vias > 0, "vias inserted");
    PASS("detailed_route_small");
}

TEST(detailed_route_medium) {
    auto pd = build_placed(30);
    DetailedRouter dr(pd, 4, 1.5);
    auto r = dr.route();

    CHECK(r.routed_nets > 0, "nets routed");
    PASS("detailed_route_medium");
}

// ============================================================================
// Noise Tests
// ============================================================================
TEST(noise_basic) {
    auto pd = build_placed(20);
    NoiseAnalyzer na(pd, 1.8, 10.0);
    auto r = na.analyze();

    CHECK(r.peak_noise_mv >= 0, "peak noise computed");
    CHECK(r.rms_noise_mv >= 0, "RMS noise computed");
    CHECK(r.psij_ps >= 0, "jitter computed");
    PASS("noise_basic");
}

// ============================================================================
// Chip Assembler Tests
// ============================================================================
TEST(chip_assemble) {
    auto core = build_placed(10);
    ChipAssembler ca(core);
    ca.add_pad("clk_pad", "CLK", IoPad::SIGNAL, IoPad::NORTH);
    ca.add_pad("data_in", "DIN", IoPad::SIGNAL, IoPad::WEST);
    ca.add_pad("data_out", "DOUT", IoPad::SIGNAL, IoPad::EAST);
    ca.auto_power_pads(4, 4);

    auto r = ca.assemble();
    CHECK(r.chip_width > core.die_area.width(), "chip wider than core");
    CHECK(r.chip_height > core.die_area.height(), "chip taller than core");
    CHECK(r.total_pads == 11, "3 signal + 4 VDD + 4 VSS = 11");
    CHECK(r.signal_pads == 3, "3 signal");
    CHECK(r.power_pads == 8, "8 power");
    PASS("chip_assemble");
}

TEST(chip_export) {
    auto core = build_placed(5);
    ChipAssembler ca(core);
    ca.add_pad("p1", "SIG1", IoPad::SIGNAL, IoPad::SOUTH);
    ca.auto_power_pads(2, 2);
    ca.assemble();

    auto pd = ca.to_physical_design();
    CHECK(pd.cells.size() > core.cells.size(), "chip has more cells than core (pads + seal)");
    CHECK(pd.die_area.area() > 0, "die area set");
    PASS("chip_export");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 8 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Memory Compiler ──\n";
    RUN(mem_sram);
    RUN(mem_regfile);

    std::cout << "\n── MCMM Analyzer ──\n";
    RUN(mcmm_default);
    RUN(mcmm_custom);

    std::cout << "\n── Detailed Router ──\n";
    RUN(detailed_route_small);
    RUN(detailed_route_medium);

    std::cout << "\n── Noise / PSIJ ──\n";
    RUN(noise_basic);

    std::cout << "\n── Chip Assembler ──\n";
    RUN(chip_assemble);
    RUN(chip_export);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
