// SiliconForge — Phase 5 Test Suite
// Tests: STA, Power Analysis, Parasitic Extraction, Global Router, DEF Parser

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "core/def_parser.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/global_router.hpp"
#include "timing/sta.hpp"
#include "timing/power.hpp"
#include "timing/parasitics.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// Helper: build a combinational circuit for timing analysis
static Netlist build_timing_circuit() {
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

// Helper: build a sequential circuit with DFFs
static Netlist build_seq_circuit() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d = nl.add_net("d"); nl.mark_input(d);
    NetId q = nl.add_net("q"); nl.mark_output(q);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");

    nl.add_gate(GateType::AND, {d, q}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1}, w2, "G2");
    nl.add_dff(w2, clk, q, -1, "FF1");
    return nl;
}

// Helper: build a placed design
static PhysicalDesign build_placed_design(int n = 20) {
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

// ============================================================================
// STA Tests
// ============================================================================
TEST(sta_combinational) {
    auto nl = build_timing_circuit();
    StaEngine sta(nl);
    auto result = sta.analyze(1.0); // 1ns clock

    CHECK(result.clock_period == 1.0, "clock period set");
    CHECK(result.num_endpoints > 0, "endpoints found");
    CHECK(result.critical_paths.size() > 0, "critical paths extracted");
    CHECK(result.critical_paths[0].delay > 0, "path delay > 0");
    PASS("sta_combinational");
}

TEST(sta_sequential) {
    auto nl = build_seq_circuit();
    StaEngine sta(nl);
    auto result = sta.analyze(2.0); // 2ns=500MHz

    CHECK(result.num_endpoints > 0, "endpoints");
    CHECK(result.critical_paths.size() > 0, "paths found");
    PASS("sta_sequential");
}

TEST(sta_tight_timing) {
    auto nl = build_timing_circuit();
    StaEngine sta(nl);
    auto result = sta.analyze(0.01); // very tight = violations expected

    CHECK(result.num_violations > 0, "tight timing causes violations");
    CHECK(result.wns < 0, "WNS is negative");
    CHECK(result.tns < 0, "TNS is negative");
    PASS("sta_tight_timing");
}

TEST(sta_relaxed_timing) {
    auto nl = build_timing_circuit();
    StaEngine sta(nl);
    auto result = sta.analyze(100.0); // very relaxed = should meet

    CHECK(result.num_violations == 0, "relaxed timing meets");
    CHECK(result.wns >= 0, "WNS >= 0");
    PASS("sta_relaxed_timing");
}

// ============================================================================
// Power Tests
// ============================================================================
TEST(power_basic) {
    auto nl = build_timing_circuit();
    PowerAnalyzer pa(nl);
    auto result = pa.analyze(500.0, 1.8); // 500MHz, 1.8V

    CHECK(result.total_power_mw > 0, "total power > 0");
    CHECK(result.dynamic_power_mw > 0, "dynamic > 0");
    CHECK(result.static_power_mw > 0, "static > 0");
    CHECK(result.dynamic_power_mw > result.static_power_mw, "dynamic > static at high freq");
    CHECK(result.num_cells == 3, "3 gates analyzed");
    PASS("power_basic");
}

TEST(power_voltage_scaling) {
    auto nl = build_timing_circuit();
    PowerAnalyzer pa1(nl);
    auto r1 = pa1.analyze(500.0, 1.8);  // 1.8V
    PowerAnalyzer pa2(nl);
    auto r2 = pa2.analyze(500.0, 0.9);  // 0.9V

    CHECK(r1.dynamic_power_mw > r2.dynamic_power_mw,
          "lower voltage = lower dynamic power (V² scaling)");
    PASS("power_voltage_scaling");
}

// ============================================================================
// Parasitic Extraction Tests
// ============================================================================
TEST(parasitics_basic) {
    auto pd = build_placed_design(10);
    ParasiticExtractor pe(pd);
    auto result = pe.extract();

    CHECK(result.nets.size() == pd.nets.size(), "all nets extracted");
    bool has_rc = false;
    for (auto& pn : result.nets) {
        if (pn.total_cap_ff > 0 && pn.total_res_ohm > 0) has_rc = true;
    }
    CHECK(has_rc, "some nets have RC parasitics");
    PASS("parasitics_basic");
}

TEST(parasitics_spef) {
    auto pd = build_placed_design(5);
    ParasiticExtractor pe(pd);
    auto result = pe.extract();
    auto spef = result.to_spef();

    CHECK(spef.find("*SPEF") != std::string::npos, "SPEF header");
    CHECK(spef.find("*D_NET") != std::string::npos, "SPEF has nets");
    CHECK(spef.find("*CAP") != std::string::npos, "SPEF has capacitance");
    CHECK(spef.find("*RES") != std::string::npos, "SPEF has resistance");
    PASS("parasitics_spef");
}

// ============================================================================
// Global Router Tests
// ============================================================================
TEST(global_router_small) {
    auto pd = build_placed_design(10);
    GlobalRouter router(pd, 10, 10);
    auto result = router.route();

    CHECK(result.routed_nets > 0, "nets routed");
    CHECK(result.failed_nets == 0, "no failures on small design");
    CHECK(result.total_wirelength > 0, "wirelength > 0");
    PASS("global_router_small");
}

TEST(global_router_medium) {
    auto pd = build_placed_design(40);
    GlobalRouter router(pd, 20, 10);
    auto result = router.route();

    CHECK(result.routed_nets > 0, "nets routed");
    CHECK(result.max_congestion >= 0, "congestion computed");
    PASS("global_router_medium");
}

// ============================================================================
// DEF Parser Tests
// ============================================================================
TEST(def_roundtrip) {
    auto pd = build_placed_design(5);
    std::string def_str = DefParser::export_def(pd);

    CHECK(def_str.find("DIEAREA") != std::string::npos, "DEF has diearea");
    CHECK(def_str.find("COMPONENTS") != std::string::npos, "DEF has components");
    CHECK(def_str.find("NETS") != std::string::npos, "DEF has nets");

    // Parse it back
    PhysicalDesign pd2;
    pd2.row_height = 10.0;
    DefParser parser;
    bool ok = parser.parse_string(def_str, pd2);
    CHECK(ok, "DEF re-parsed");
    CHECK(pd2.cells.size() == pd.cells.size(), "same cell count after roundtrip");
    PASS("def_roundtrip");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 5 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Static Timing Analysis ──\n";
    RUN(sta_combinational);
    RUN(sta_sequential);
    RUN(sta_tight_timing);
    RUN(sta_relaxed_timing);

    std::cout << "\n── Power Analysis ──\n";
    RUN(power_basic);
    RUN(power_voltage_scaling);

    std::cout << "\n── Parasitic Extraction ──\n";
    RUN(parasitics_basic);
    RUN(parasitics_spef);

    std::cout << "\n── Global Router ──\n";
    RUN(global_router_small);
    RUN(global_router_medium);

    std::cout << "\n── DEF Parser ──\n";
    RUN(def_roundtrip);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
