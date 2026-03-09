// SiliconForge — Phase 6 Test Suite
// Tests: Verilog parser, DRC, LVS, Signal Integrity, IR Drop

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/global_router.hpp"
#include "frontend/verilog_parser.hpp"
#include "verify/drc.hpp"
#include "verify/lvs.hpp"
#include "timing/signal_integrity.hpp"
#include "timing/ir_drop.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static const char* SIMPLE_VERILOG = R"V(
module adder (a, b, cin, sum, cout);
  input a, b, cin;
  output sum, cout;
  wire w1, w2, w3;

  xor g1 (w1, a, b);
  xor g2 (sum, w1, cin);
  and g3 (w2, a, b);
  and g4 (w3, w1, cin);
  or  g5 (cout, w2, w3);
endmodule
)V";

static PhysicalDesign build_placed_routed(int n = 15) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < n; ++i)
        pd.add_cell("c" + std::to_string(i), "AND2", 3.0, 10.0);
    for (int i = 0; i < n - 1; ++i)
        pd.add_net("n" + std::to_string(i), {i, i+1});
    AnalyticalPlacer placer(pd);
    placer.place();
    GlobalRouter router(pd, 10, 10);
    router.route();
    return pd;
}

// ============================================================================
// Verilog Parser Tests
// ============================================================================
TEST(verilog_parse) {
    Netlist nl;
    VerilogParser parser;
    auto r = parser.parse_string(SIMPLE_VERILOG, nl);

    CHECK(r.success, "parse succeeded");
    CHECK(r.module_name == "adder", "module name");
    CHECK(r.num_inputs == 3, "3 inputs (a,b,cin)");
    CHECK(r.num_outputs == 2, "2 outputs (sum,cout)");
    CHECK(r.num_gates == 5, "5 gates");
    PASS("verilog_parse");
}

TEST(verilog_export) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, y, "G1");

    auto vstr = VerilogParser::to_verilog(nl, "test_and");
    CHECK(vstr.find("module test_and") != std::string::npos, "module declaration");
    CHECK(vstr.find("input a") != std::string::npos, "input a");
    CHECK(vstr.find("output y") != std::string::npos, "output y");
    CHECK(vstr.find("and G1") != std::string::npos, "AND gate");
    PASS("verilog_export");
}

TEST(verilog_assign) {
    const char* src = R"V(
    module inv_test (a, y);
      input a; output y;
      assign y = ~a;
    endmodule
    )V";
    Netlist nl;
    VerilogParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "parse assign");
    CHECK(r.num_gates == 1, "1 NOT gate from assign");
    PASS("verilog_assign");
}

// ============================================================================
// DRC Tests
// ============================================================================
TEST(drc_clean) {
    auto pd = build_placed_routed(10);
    DrcEngine drc(pd);
    drc.load_default_rules(0.01); // Very relaxed rules
    auto r = drc.check();

    CHECK(r.total_rules > 0, "rules checked");
    // With tiny min_feature, should be mostly clean
    PASS("drc_clean");
}

TEST(drc_violations) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    pd.row_height = 10.0;
    // Cell outside die boundary
    int c = pd.add_cell("bad_cell", "AND2", 3, 10);
    pd.cells[c].position = {100, 100}; // way outside
    pd.cells[c].placed = true;

    DrcEngine drc(pd);
    drc.load_default_rules(0.13);
    auto r = drc.check();
    CHECK(r.errors > 0, "boundary violation detected");
    PASS("drc_violations");
}

// ============================================================================
// LVS Tests
// ============================================================================
TEST(lvs_match) {
    Netlist schem;
    NetId a = schem.add_net("a"); schem.mark_input(a);
    NetId b = schem.add_net("b"); schem.mark_input(b);
    NetId y = schem.add_net("y"); schem.mark_output(y);
    schem.add_gate(GateType::AND, {a, b}, y, "G1");

    // Build matching layout
    PhysicalDesign layout;
    layout.die_area = Rect(0,0,100,100);
    layout.add_cell("G1", "AND", 3, 10);
    layout.cells[0].placed = true;
    layout.add_net("n1", {0});

    LvsChecker lvs(schem, layout);
    auto r = lvs.check();
    CHECK(r.matched_cells >= 1, "cells matched");
    PASS("lvs_match");
}

TEST(lvs_mismatch) {
    Netlist schem;
    NetId a = schem.add_net("a"); schem.mark_input(a);
    NetId y = schem.add_net("y"); schem.mark_output(y);
    schem.add_gate(GateType::AND, {a}, y, "G1");
    schem.add_gate(GateType::OR, {a}, y, "G2");

    // Layout has only 1 cell
    PhysicalDesign layout;
    layout.die_area = Rect(0,0,100,100);
    layout.add_cell("G1", "AND", 3, 10);
    layout.cells[0].placed = true;

    LvsChecker lvs(schem, layout);
    auto r = lvs.check();
    CHECK(!r.match, "mismatch detected");
    CHECK(r.mismatches.size() > 0, "mismatches listed");
    PASS("lvs_mismatch");
}

// ============================================================================
// Signal Integrity Tests
// ============================================================================
TEST(si_basic) {
    auto pd = build_placed_routed(15);
    SignalIntegrityAnalyzer si(pd, 1.8);
    auto r = si.analyze();

    CHECK(r.nets_analyzed > 0, "nets analyzed");
    CHECK(r.time_ms >= 0, "time measured");
    PASS("si_basic");
}

// ============================================================================
// IR Drop Tests
// ============================================================================
TEST(ir_drop_basic) {
    auto pd = build_placed_routed(20);
    IrDropAnalyzer ir(pd, 1.8, 50.0); // 50mA total
    auto r = ir.analyze(8);

    CHECK(r.grid_x == 8 && r.grid_y == 8, "grid resolution");
    CHECK(r.worst_drop_mv >= 0, "drop computed");
    CHECK(r.avg_drop_mv >= 0, "avg drop computed");
    CHECK(r.drop_map.size() == 8, "drop map rows");
    CHECK(r.drop_map[0].size() == 8, "drop map cols");
    PASS("ir_drop_basic");
}

TEST(ir_drop_hotspot) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    pd.row_height = 10.0;
    // Concentrate all cells in center → hotspot
    for (int i = 0; i < 50; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "AND2", 2, 10);
        pd.cells[c].position = {45 + (i%5)*2.0, 40 + (i/5)*10.0};
        pd.cells[c].placed = true;
    }
    for (int i = 0; i < 49; ++i)
        pd.add_net("n" + std::to_string(i), {i, i+1});

    IrDropAnalyzer ir(pd, 1.8, 200.0); // high current
    auto r = ir.analyze(10);
    CHECK(r.worst_drop_mv > 0, "drop detected in dense region");
    PASS("ir_drop_hotspot");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 6 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Verilog Parser ──\n";
    RUN(verilog_parse);
    RUN(verilog_export);
    RUN(verilog_assign);

    std::cout << "\n── DRC Engine ──\n";
    RUN(drc_clean);
    RUN(drc_violations);

    std::cout << "\n── LVS Checker ──\n";
    RUN(lvs_match);
    RUN(lvs_mismatch);

    std::cout << "\n── Signal Integrity ──\n";
    RUN(si_basic);

    std::cout << "\n── IR Drop ──\n";
    RUN(ir_drop_basic);
    RUN(ir_drop_hotspot);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
