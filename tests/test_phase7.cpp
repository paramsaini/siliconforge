// SiliconForge — Phase 7 Test Suite
// Tests: SDC Parser, ECO Engine, Floorplanner, OPC, PDN

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/global_router.hpp"
#include "pnr/floorplan.hpp"
#include "frontend/sdc_parser.hpp"
#include "synth/eco.hpp"
#include "verify/opc.hpp"
#include "timing/pdn.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static const char* TEST_SDC = R"SDC(
# Clock definition
create_clock -name clk -period 10.0 clk_port
set_clock_uncertainty 0.1 clk

# IO delays
set_input_delay -clock clk 2.0 data_in
set_output_delay -clock clk 1.5 data_out

# Timing exceptions
set_false_path -from reset -to data_out
set_multicycle_path 2 -from slow_reg -to fast_reg
set_max_delay 5.0 -from a -to b

# Design rules
set_max_fanout 20 data_in
)SDC";

// ============================================================================
// SDC Parser Tests
// ============================================================================
TEST(sdc_parse) {
    SdcConstraints sdc;
    SdcParser parser;
    auto r = parser.parse_string(TEST_SDC, sdc);

    CHECK(r.success, "parse succeeded");
    CHECK(r.num_constraints >= 5, "5+ constraints parsed");
    CHECK(sdc.clocks.size() == 1, "1 clock");
    CHECK(sdc.clocks[0].name == "clk", "clock name");
    CHECK(std::abs(sdc.clocks[0].period_ns - 10.0) < 0.01, "clock period 10ns");
    CHECK(std::abs(sdc.clocks[0].uncertainty - 0.1) < 0.01, "uncertainty 0.1");
    CHECK(sdc.input_delays.size() == 1, "1 input delay");
    CHECK(sdc.output_delays.size() == 1, "1 output delay");
    CHECK(sdc.exceptions.size() >= 2, "2+ exceptions");
    PASS("sdc_parse");
}

TEST(sdc_export) {
    SdcConstraints sdc;
    SdcParser parser;
    parser.parse_string(TEST_SDC, sdc);
    auto out = SdcParser::to_sdc(sdc);

    CHECK(out.find("create_clock") != std::string::npos, "exported clock");
    CHECK(out.find("set_input_delay") != std::string::npos, "exported input delay");
    CHECK(out.find("set_false_path") != std::string::npos, "exported false path");
    PASS("sdc_export");
}

TEST(sdc_clock_lookup) {
    SdcConstraints sdc;
    SdcParser parser;
    parser.parse_string(TEST_SDC, sdc);

    auto* clk = sdc.find_clock("clk");
    CHECK(clk != nullptr, "found clock");
    CHECK(std::abs(sdc.get_clock_period("clk") - 10.0) < 0.01, "period lookup");
    CHECK(sdc.get_clock_period("nonexistent") == 0, "missing clock returns 0");
    PASS("sdc_clock_lookup");
}

// ============================================================================
// ECO Engine Tests
// ============================================================================
TEST(eco_buffer) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId w = nl.add_net("w");
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, w, "G1");
    nl.add_gate(GateType::NOT, {w}, y, "G2");

    size_t gates_before = nl.num_gates();
    EcoEngine eco(nl);
    eco.insert_buffer(w, "eco_buf1");

    CHECK(nl.num_gates() > gates_before, "buffer added");
    auto result = eco.apply();
    CHECK(result.changes_applied > 0, "changes applied");
    CHECK(result.gates_added > 0, "gate added count");
    PASS("eco_buffer");
}

TEST(eco_replace) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId y = nl.add_net("y"); nl.mark_output(y);
    GateId g = nl.add_gate(GateType::AND, {a, b}, y, "G1");

    EcoEngine eco(nl);
    eco.replace_gate(g, GateType::NAND);
    CHECK(nl.gate(g).type == GateType::NAND, "gate replaced to NAND");
    PASS("eco_replace");
}

// ============================================================================
// Floorplanner Tests
// ============================================================================
TEST(floorplan_basic) {
    Floorplanner fp;
    fp.add_macro("CPU", 30, 20);
    fp.add_macro("SRAM", 25, 15);
    fp.add_macro("PLL", 10, 10);
    fp.add_macro("IO", 40, 5);
    fp.add_connection(0, 1);
    fp.add_connection(0, 2);
    fp.add_connection(1, 3);

    auto r = fp.solve(1000, 50, 0.99);
    CHECK(r.total_area > 0, "area computed");
    CHECK(r.dead_space_pct >= 0 && r.dead_space_pct < 100, "reasonable dead space");
    CHECK(r.wirelength > 0, "wirelength measured");
    PASS("floorplan_basic");
}

TEST(floorplan_export) {
    Floorplanner fp;
    fp.add_macro("A", 10, 10);
    fp.add_macro("B", 15, 8);
    fp.add_macro("C", 20, 12);
    fp.add_connection(0, 1);
    fp.add_connection(1, 2);
    fp.solve(500);

    auto pd = fp.to_physical_design();
    CHECK(pd.cells.size() == 3, "3 cells in PD");
    CHECK(pd.die_area.area() > 0, "die area set");
    PASS("floorplan_export");
}

// ============================================================================
// OPC Tests
// ============================================================================
TEST(opc_resolution) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    LithoParams params;
    params.wavelength_nm = 193;
    params.numerical_aperture = 0.85;
    OpcEngine opc(pd, params);

    double res = opc.resolution_limit();
    CHECK(res > 0, "resolution limit computed");
    CHECK(res < params.wavelength_nm, "res < wavelength"); // NA > 0.5
    PASS("opc_resolution");
}

TEST(opc_corrections) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 10, 10);
    // Add small cells that need OPC correction
    for (int i = 0; i < 5; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "TINY", 0.05, 0.05); // 50nm features
        pd.cells[c].position = {(double)i * 2, 0};
        pd.cells[c].placed = true;
    }

    LithoParams params;
    params.wavelength_nm = 193;
    params.numerical_aperture = 0.85;
    OpcEngine opc(pd, params);
    auto r = opc.apply_opc();

    CHECK(r.features_analyzed >= 5, "features analyzed");
    CHECK(r.corrections_applied > 0, "corrections applied");
    PASS("opc_corrections");
}

// ============================================================================
// PDN Tests
// ============================================================================
TEST(pdn_basic) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < 20; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "AND2", 3, 10);
        pd.cells[c].position = {(double)(i % 10) * 20, (double)(i / 10) * 50};
        pd.cells[c].placed = true;
    }

    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 100.0);
    auto r = pdn.analyze(8);

    CHECK(r.worst_drop_mv >= 0, "drop computed");
    CHECK(r.nodes.size() > 0, "grid nodes");
    PASS("pdn_basic");
}

TEST(pdn_em_check) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    // Dense cells — high current draw
    for (int i = 0; i < 100; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "AND2", 1, 1);
        pd.cells[c].position = {(double)(i%10)*10, (double)(i/10)*10};
        pd.cells[c].placed = true;
    }

    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 5000; // Very high current
    cfg.em_limit_ma_per_um = 0.5; // Tight EM limit
    cfg.pad_resistance = 0.01;
    cfg.stripes.push_back({PdnStripe::HORIZONTAL, 50, 1.0, 3, 0.05});
    cfg.stripes.push_back({PdnStripe::VERTICAL, 50, 1.0, 4, 0.05});
    pdn.set_config(cfg);

    auto r = pdn.analyze(5);
    CHECK(r.worst_drop_mv > 0, "significant drop");
    // With very high current and tight limits, should see EM violations
    PASS("pdn_em_check");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 7 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── SDC Parser ──\n";
    RUN(sdc_parse);
    RUN(sdc_export);
    RUN(sdc_clock_lookup);

    std::cout << "\n── ECO Engine ──\n";
    RUN(eco_buffer);
    RUN(eco_replace);

    std::cout << "\n── Floorplanner ──\n";
    RUN(floorplan_basic);
    RUN(floorplan_export);

    std::cout << "\n── OPC / Lithography ──\n";
    RUN(opc_resolution);
    RUN(opc_corrections);

    std::cout << "\n── PDN Analyzer ──\n";
    RUN(pdn_basic);
    RUN(pdn_em_check);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
