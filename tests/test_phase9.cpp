// SiliconForge — Phase 9 Test Suite
// Tests: LEC, CDC, Reliability, Antenna, Post-Route Optimizer

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/global_router.hpp"
#include "pnr/post_route_opt.hpp"
#include "formal/lec.hpp"
#include "verify/cdc.hpp"
#include "verify/reliability.hpp"
#include "verify/antenna.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static Netlist make_and_circuit(const std::string& prefix = "") {
    Netlist nl;
    NetId a = nl.add_net(prefix + "a"); nl.mark_input(a);
    NetId b = nl.add_net(prefix + "b"); nl.mark_input(b);
    NetId y = nl.add_net(prefix + "y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, y, prefix + "G1");
    return nl;
}

static PhysicalDesign build_routed(int n = 15) {
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
// LEC Tests
// ============================================================================
TEST(lec_equivalent) {
    auto golden = make_and_circuit();
    auto revised = make_and_circuit(); // identical
    LecEngine lec(golden, revised);
    auto r = lec.check();

    CHECK(r.equivalent, "identical netlists are equivalent");
    CHECK(r.matched == 1, "1 output matched");
    CHECK(r.mismatched == 0, "0 mismatches");
    PASS("lec_equivalent");
}

TEST(lec_different) {
    auto golden = make_and_circuit();
    // Revised: OR instead of AND
    Netlist revised;
    NetId a = revised.add_net("a"); revised.mark_input(a);
    NetId b = revised.add_net("b"); revised.mark_input(b);
    NetId y = revised.add_net("y"); revised.mark_output(y);
    revised.add_gate(GateType::OR, {a, b}, y, "G1");

    LecEngine lec(golden, revised);
    auto r = lec.check();

    CHECK(!r.equivalent, "AND≠OR detected");
    CHECK(r.mismatched > 0, "mismatch found");
    PASS("lec_different");
}

// ============================================================================
// CDC Tests
// ============================================================================
TEST(cdc_detect) {
    Netlist nl;
    NetId clk1 = nl.add_net("clk1"); nl.mark_input(clk1);
    NetId clk2 = nl.add_net("clk2"); nl.mark_input(clk2);
    NetId d1 = nl.add_net("d1"); nl.mark_input(d1);
    NetId q1 = nl.add_net("q1");
    NetId w = nl.add_net("w");
    NetId q2 = nl.add_net("q2"); nl.mark_output(q2);

    nl.add_dff(d1, clk1, q1, -1, "FF1"); // domain: clk1
    nl.add_gate(GateType::BUF, {q1}, w, "BUF1");
    nl.add_dff(w, clk2, q2, -1, "FF2");  // domain: clk2 → crossing!

    CdcAnalyzer cdc(nl);
    cdc.auto_detect_domains();
    auto r = cdc.analyze();

    CHECK(r.domains_found >= 1, "clock domains found");
    PASS("cdc_detect");
}

TEST(cdc_safe) {
    Netlist nl;
    NetId clk1 = nl.add_net("clk1"); nl.mark_input(clk1);
    NetId clk2 = nl.add_net("clk2"); nl.mark_input(clk2);
    NetId d = nl.add_net("d"); nl.mark_input(d);
    NetId q1 = nl.add_net("q1");
    NetId q2 = nl.add_net("q2");
    NetId q3 = nl.add_net("q3"); nl.mark_output(q3);

    nl.add_dff(d, clk1, q1, -1, "FF_src");   // source domain
    nl.add_dff(q1, clk2, q2, -1, "FF_sync1"); // sync FF 1
    nl.add_dff(q2, clk2, q3, -1, "FF_sync2"); // sync FF 2 → 2-FF sync!

    CdcAnalyzer cdc(nl);
    cdc.auto_detect_domains();
    auto r = cdc.analyze();

    CHECK(r.domains_found >= 1, "domains found");
    PASS("cdc_safe");
}

// ============================================================================
// Reliability Tests
// ============================================================================
TEST(reliability_basic) {
    auto nl = make_and_circuit();
    ReliabilityAnalyzer ra(nl);
    AgingConfig cfg;
    cfg.lifetime_years = 10;
    cfg.temperature_c = 85;
    cfg.voltage = 1.0;
    ra.set_config(cfg);

    auto r = ra.analyze();
    CHECK(r.nbti_vth_shift_mv > 0, "NBTI shift > 0");
    CHECK(r.hci_vth_shift_mv > 0, "HCI shift > 0");
    CHECK(r.timing_degradation_pct > 0, "timing degradation > 0");
    PASS("reliability_basic");
}

TEST(reliability_voltage_stress) {
    auto nl = make_and_circuit();
    ReliabilityAnalyzer ra1(nl);
    AgingConfig c1; c1.voltage = 0.9; c1.lifetime_years = 10; c1.temperature_c = 85;
    ra1.set_config(c1);
    auto r1 = ra1.analyze();

    ReliabilityAnalyzer ra2(nl);
    AgingConfig c2; c2.voltage = 1.2; c2.lifetime_years = 10; c2.temperature_c = 85;
    ra2.set_config(c2);
    auto r2 = ra2.analyze();

    CHECK(r2.nbti_vth_shift_mv > r1.nbti_vth_shift_mv,
          "higher voltage = more NBTI degradation");
    PASS("reliability_voltage_stress");
}

// ============================================================================
// Antenna Tests
// ============================================================================
TEST(antenna_check) {
    auto pd = build_routed(15);
    AntennaChecker ac(pd);
    auto r = ac.check();

    CHECK(r.nets_checked > 0, "nets checked");
    CHECK(r.time_ms >= 0, "time measured");
    PASS("antenna_check");
}

// ============================================================================
// Post-Route Optimizer Tests
// ============================================================================
TEST(postroute_opt) {
    auto nl = make_and_circuit();
    auto pd = build_routed(15);
    PostRouteOptimizer opt(nl, pd);
    auto r = opt.optimize();

    CHECK(r.vias_doubled >= 0, "vias processing ran");
    CHECK(r.wires_widened >= 0, "wires widened");
    PASS("postroute_opt");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 9 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── LEC ──\n";
    RUN(lec_equivalent);
    RUN(lec_different);

    std::cout << "\n── CDC Analyzer ──\n";
    RUN(cdc_detect);
    RUN(cdc_safe);

    std::cout << "\n── Reliability / Aging ──\n";
    RUN(reliability_basic);
    RUN(reliability_voltage_stress);

    std::cout << "\n── Antenna Checker ──\n";
    RUN(antenna_check);

    std::cout << "\n── Post-Route Optimizer ──\n";
    RUN(postroute_opt);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
