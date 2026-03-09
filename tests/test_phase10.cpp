// SiliconForge — Phase 10 Test Suite
// Tests: Clock Gating, Multi-Vt, Hierarchy, Report Generator, Design Database

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "core/hierarchy.hpp"
#include "core/report_gen.hpp"
#include "core/design_db.hpp"
#include "pnr/physical.hpp"
#include "timing/sta.hpp"
#include "timing/power.hpp"
#include "synth/clock_gating.hpp"
#include "synth/multi_vt.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static Netlist make_ff_circuit() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d1 = nl.add_net("d1"); nl.mark_input(d1);
    NetId d2 = nl.add_net("d2"); nl.mark_input(d2);
    NetId d3 = nl.add_net("d3"); nl.mark_input(d3);
    NetId q1 = nl.add_net("q1"); nl.mark_output(q1);
    NetId q2 = nl.add_net("q2"); nl.mark_output(q2);
    NetId q3 = nl.add_net("q3"); nl.mark_output(q3);
    nl.add_dff(d1, clk, q1, -1, "FF1");
    nl.add_dff(d2, clk, q2, -1, "FF2");
    nl.add_dff(d3, clk, q3, -1, "FF3");
    return nl;
}

static Netlist make_logic_circuit() {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId c = nl.add_net("c"); nl.mark_input(c);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId w3 = nl.add_net("w3");
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, w1, "G1");
    nl.add_gate(GateType::OR, {w1, c}, w2, "G2");
    nl.add_gate(GateType::NOT, {w2}, w3, "G3");
    nl.add_gate(GateType::AND, {w3, a}, y, "G4");
    return nl;
}

// ============================================================================
// Clock Gating Tests
// ============================================================================
TEST(clock_gating) {
    auto nl = make_ff_circuit();
    size_t gates_before = nl.num_gates();

    ClockGatingEngine cg(nl);
    cg.set_min_group(1);
    cg.set_activity(0.1);
    auto r = cg.insert();

    CHECK(r.total_ffs >= 1, "FFs detected");
    CHECK(r.icg_cells_inserted >= 1, "ICG cell inserted");
    CHECK(r.gated_ffs >= 1, "FFs gated");
    CHECK(r.power_reduction_pct > 0, "power reduction > 0");
    CHECK(nl.num_gates() > gates_before, "gate count increased (ICG added)");
    PASS("clock_gating");
}

// ============================================================================
// Multi-Vt Tests
// ============================================================================
TEST(multi_vt_basic) {
    auto nl = make_logic_circuit();
    MultiVtOptimizer opt(nl);
    auto r = opt.optimize();

    CHECK(r.total_cells > 0, "cells analyzed");
    CHECK(r.hvt_cells + r.svt_cells + r.lvt_cells == r.total_cells, "all cells assigned");
    CHECK(r.leakage_reduction_pct != 0, "leakage impact computed");
    PASS("multi_vt_basic");
}

TEST(multi_vt_deep) {
    // Deeper circuit → more differentiation between LVT/HVT
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId prev = a;
    for (int i = 0; i < 10; ++i) {
        NetId next = nl.add_net("w" + std::to_string(i));
        nl.add_gate(GateType::NOT, {prev}, next, "G" + std::to_string(i));
        prev = next;
    }
    nl.mark_output(prev);

    MultiVtOptimizer opt(nl);
    auto r = opt.optimize();
    CHECK(r.total_cells == 10, "10 gates");
    CHECK(r.lvt_cells > 0, "some LVT on critical");
    CHECK(r.hvt_cells > 0, "some HVT on non-critical");
    PASS("multi_vt_deep");
}

// ============================================================================
// Hierarchy Tests
// ============================================================================
TEST(hierarchy_basic) {
    HierarchyManager hm;
    auto nl_alu = make_logic_circuit();
    auto nl_reg = make_ff_circuit();

    Netlist nl_top;
    nl_top.add_net("clk"); nl_top.mark_input(0);
    nl_top.add_net("data"); nl_top.mark_input(1);

    hm.add_module("alu", nl_alu);
    hm.add_module("regfile", nl_reg);
    hm.add_module("top", nl_top);
    hm.instantiate("top", "alu");
    hm.instantiate("top", "regfile");

    auto r = hm.analyze();
    CHECK(r.total_modules == 3, "3 modules");
    CHECK(r.leaf_modules == 2, "2 leaf modules");
    CHECK(r.hierarchy_depth >= 1, "depth >= 1");
    PASS("hierarchy_basic");
}

TEST(hierarchy_flatten) {
    HierarchyManager hm;
    auto nl1 = make_logic_circuit();
    Netlist nl2;
    nl2.add_net("x"); nl2.mark_input(0);
    nl2.add_net("y"); nl2.mark_output(1);
    nl2.add_gate(GateType::NOT, {0}, 1, "INV");

    hm.add_module("sub", nl2);
    hm.add_module("top", nl1);
    hm.instantiate("top", "sub");

    auto flat = hm.flatten("top");
    CHECK(flat.num_gates() > 0, "flattened has gates");
    PASS("hierarchy_flatten");
}

// ============================================================================
// Report Generator Tests
// ============================================================================
TEST(report_gen) {
    ReportGenerator rg("test_chip");

    StaResult sta;
    sta.wns = -0.5;
    sta.tns = -2.3;
    rg.add_timing(sta);

    PowerResult pwr;
    pwr.dynamic_power_mw = 12.5;
    pwr.static_power_mw = 1.2;
    pwr.total_power_mw = 13.7;
    rg.add_power(pwr);

    auto nl = make_logic_circuit();
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    rg.add_area(nl, pd);
    rg.add_drc_summary(0, 3);

    auto report = rg.generate();
    CHECK(report.find("SiliconForge") != std::string::npos, "header present");
    CHECK(report.find("TIMING") != std::string::npos, "timing section");
    CHECK(report.find("POWER") != std::string::npos, "power section");
    CHECK(report.find("AREA") != std::string::npos, "area section");
    CHECK(report.find("DRC") != std::string::npos, "DRC section");
    CHECK(rg.section_count() == 4, "4 sections");
    PASS("report_gen");
}

// ============================================================================
// Design Database Tests
// ============================================================================
TEST(design_db) {
    DesignDatabase db;
    db.create("chip_v1", "7nm");
    CHECK(db.exists("chip_v1"), "design exists");

    auto& ds = db.get("chip_v1");
    CHECK(ds.technology == "7nm", "tech correct");
    CHECK(ds.stage == DesignState::INIT, "initial stage");

    db.set_stage("chip_v1", DesignState::SYNTHESIZED);
    CHECK(ds.stage == DesignState::SYNTHESIZED, "stage advanced");

    db.log("chip_v1", "Synthesis complete");
    int v = db.snapshot("chip_v1");
    CHECK(v == 1, "version 1");

    auto summary = db.summary("chip_v1");
    CHECK(summary.find("SYNTHESIZED") != std::string::npos, "summary shows stage");
    PASS("design_db");
}

TEST(design_db_multi) {
    DesignDatabase db;
    db.create("d1", "5nm");
    db.create("d2", "3nm");

    auto names = db.list();
    CHECK(names.size() == 2, "2 designs");

    auto stats = db.stats();
    CHECK(stats.designs == 2, "stats: 2 designs");

    db.remove("d1");
    CHECK(!db.exists("d1"), "d1 removed");
    CHECK(db.exists("d2"), "d2 still exists");
    PASS("design_db_multi");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 10 — Test Suite              ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Clock Gating ──\n";
    RUN(clock_gating);

    std::cout << "\n── Multi-Vt ──\n";
    RUN(multi_vt_basic);
    RUN(multi_vt_deep);

    std::cout << "\n── Hierarchy Manager ──\n";
    RUN(hierarchy_basic);
    RUN(hierarchy_flatten);

    std::cout << "\n── Report Generator ──\n";
    RUN(report_gen);

    std::cout << "\n── Design Database ──\n";
    RUN(design_db);
    RUN(design_db_multi);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
