// SiliconForge Phase 19 — Industrial-Grade End-to-End Integration Tests
// Exercises ALL upgrades together in a single flow: Retiming (Gap 2),
// STA OCV/AOCV (Gap 3), MCMM (Gap 4), DRC SKY130 (Gap 5),
// Timing-Driven Placement (Gap 6), Post-Route Optimization (Gap 1).

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "pnr/placer.hpp"
#include "pnr/post_route_opt.hpp"
#include "synth/retiming.hpp"
#include "timing/sta.hpp"
#include "timing/mcmm.hpp"
#include "verify/drc.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <string>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ---------------------------------------------------------------------------
// Helper: sequential circuit with feedback (DFF → AND → NOT → DFF)
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Helper: PhysicalDesign with cells and nets
// ---------------------------------------------------------------------------
static PhysicalDesign build_test_pd(int n = 20) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < n; i++)
        pd.add_cell("cell_" + std::to_string(i), "AND2_X1", 3.0, 10.0);
    for (int i = 0; i < n - 1; i++)
        pd.add_net("n" + std::to_string(i), {i, i + 1});
    return pd;
}

// ---------------------------------------------------------------------------
// Helper: pipeline circuit (DFF → 4 gates → DFF) for retiming tests
// ---------------------------------------------------------------------------
static Netlist build_pipeline_circuit() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId in  = nl.add_net("in");  nl.mark_input(in);
    NetId q1  = nl.add_net("q1");
    nl.add_dff(in, clk, q1, -1, "FF_in");

    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId w3 = nl.add_net("w3");
    NetId w4 = nl.add_net("w4");
    nl.add_gate(GateType::AND, {q1, q1}, w1, "G1");
    nl.add_gate(GateType::OR,  {w1, q1}, w2, "G2");
    nl.add_gate(GateType::XOR, {w2, q1}, w3, "G3");
    nl.add_gate(GateType::NOT, {w3},     w4, "G4");

    NetId out = nl.add_net("out"); nl.mark_output(out);
    nl.add_dff(w4, clk, out, -1, "FF_out");
    return nl;
}

// ---------------------------------------------------------------------------
// Helper: deep pipeline (DFF → 10 NOT gates → DFF) for AOCV depth stress
// ---------------------------------------------------------------------------
static Netlist build_deep_pipeline() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId in  = nl.add_net("in");  nl.mark_input(in);
    NetId q1  = nl.add_net("q1");
    nl.add_dff(in, clk, q1, -1, "FF_in");

    NetId prev = q1;
    for (int i = 0; i < 10; i++) {
        NetId next = nl.add_net("w" + std::to_string(i));
        nl.add_gate(GateType::NOT, {prev}, next, "G" + std::to_string(i));
        prev = next;
    }

    NetId out = nl.add_net("out"); nl.mark_output(out);
    nl.add_dff(prev, clk, out, -1, "FF_out");
    return nl;
}

// ---------------------------------------------------------------------------
// Helper: add wires to a PhysicalDesign to simulate routing
// ---------------------------------------------------------------------------
static void add_routing_wires(PhysicalDesign& pd, int count = 10) {
    for (int i = 0; i < count; i++) {
        WireSegment w;
        w.layer  = 0;
        w.start  = Point(static_cast<double>(i * 10), 5.0);
        w.end    = Point(static_cast<double>((i + 1) * 10), 5.0);
        w.width  = 0.2;
        w.net_id = (i < static_cast<int>(pd.nets.size())) ? i : -1;
        pd.wires.push_back(w);
    }
}

// ===================================================================
//  1. e2e_sta_ocv_to_mcmm — Gap 3 + Gap 4 Integration
// ===================================================================
TEST(e2e_sta_ocv_to_mcmm) {
    auto nl = build_seq_circuit();

    // Step 1: STA with AOCV enabled
    StaEngine sta(nl);
    sta.set_ocv_mode(OcvMode::AOCV);
    sta.enable_aocv(0.10, 0.10);
    auto sta_res = sta.analyze(10.0, 5);
    CHECK(sta_res.ocv_mode == OcvMode::AOCV,
          "STA OCV mode should be AOCV");
    CHECK(sta_res.num_endpoints > 0,
          "STA should have endpoints, got " + std::to_string(sta_res.num_endpoints));

    // Step 2: MCMM with foundry corners (each with OCV)
    McmmAnalyzer mcmm(nl);
    mcmm.load_foundry_corners();

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto mcmm_res = mcmm.analyze();
    CHECK(mcmm_res.corners == 7,
          "expected 7 foundry corners, got " + std::to_string(mcmm_res.corners));
    CHECK(mcmm_res.scenarios == 7,
          "expected 7 scenarios, got " + std::to_string(mcmm_res.scenarios));
    CHECK(mcmm_res.details.size() == 7,
          "expected 7 detail entries, got " + std::to_string(mcmm_res.details.size()));

    // Verify MCMM worst_setup_wns matches the worst individual scenario
    double manual_worst = 0;
    for (auto& sc : mcmm_res.details) {
        if (sc.sta.wns < manual_worst)
            manual_worst = sc.sta.wns;
    }
    CHECK(std::fabs(mcmm_res.worst_setup_wns - manual_worst) < 1e-6,
          "worst_setup_wns mismatch: " + std::to_string(mcmm_res.worst_setup_wns) +
          " vs manual " + std::to_string(manual_worst));

    PASS("e2e_sta_ocv_to_mcmm");
}

// ===================================================================
//  2. e2e_place_then_sta — Gap 6 + Gap 3 Integration
// ===================================================================
TEST(e2e_place_then_sta) {
    // Build physical design and sequential netlist
    PhysicalDesign pd = build_test_pd(20);
    auto nl = build_seq_circuit();

    // Step 1: Timing-driven placement
    AnalyticalPlacer placer(pd);
    placer.enable_timing_driven(nl, 10.0);
    auto place_res = placer.place();
    CHECK(place_res.hpwl > 0,
          "HPWL should be > 0, got " + std::to_string(place_res.hpwl));
    CHECK(place_res.iterations > 0,
          "placement iterations should be > 0");
    CHECK(place_res.legal, "placement should be legal");

    // Step 2: STA with OCV on the same netlist
    StaEngine sta(nl, nullptr, &pd);
    sta.set_ocv_mode(OcvMode::OCV);
    sta.enable_ocv(1.15, 0.85);
    auto sta_res = sta.analyze(10.0, 5);
    CHECK(sta_res.ocv_mode == OcvMode::OCV,
          "STA should report OCV mode");
    CHECK(sta_res.num_endpoints > 0,
          "STA should have endpoints after placement");

    PASS("e2e_place_then_sta");
}

// ===================================================================
//  3. e2e_place_route_postopt — Gap 6 + Gap 1 Integration
// ===================================================================
TEST(e2e_place_route_postopt) {
    auto nl = build_seq_circuit();
    PhysicalDesign pd = build_test_pd(20);

    // Step 1: Timing-driven placement
    AnalyticalPlacer placer(pd);
    placer.enable_timing_driven(nl, 10.0);
    auto place_res = placer.place();
    CHECK(place_res.legal, "placement should be legal");

    // Step 2: Simulate routing by adding wires
    add_routing_wires(pd, 15);
    CHECK(!pd.wires.empty(), "wires should be added after routing");

    // Step 3: Post-route optimization
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(10.0);
    auto opt_res = opt.optimize();
    CHECK(opt_res.iterations > 0,
          "post-route optimizer should run at least 1 iteration");
    CHECK(!opt_res.message.empty(),
          "post-route result message should not be empty");

    PASS("e2e_place_route_postopt");
}

// ===================================================================
//  4. e2e_retiming_then_sta — Gap 2 + Gap 3 Integration
// ===================================================================
TEST(e2e_retiming_then_sta) {
    auto nl = build_pipeline_circuit();

    // Step 1: Retiming pass
    RetimingEngine retime;
    auto retime_res = retime.optimize_with_result(nl);
    CHECK(retime_res.critical_path_before > 0,
          "critical_path_before should be > 0");
    CHECK(!retime_res.message.empty(),
          "retiming message should not be empty");

    // Step 2: STA on the (possibly modified) netlist
    StaEngine sta(nl);
    sta.set_ocv_mode(OcvMode::AOCV);
    sta.enable_aocv(0.10, 0.10);
    auto sta_res = sta.analyze(10.0, 5);
    CHECK(sta_res.num_endpoints > 0,
          "STA should have endpoints after retiming, got " +
          std::to_string(sta_res.num_endpoints));

    PASS("e2e_retiming_then_sta");
}

// ===================================================================
//  5. e2e_drc_with_sky130_after_placement — Gap 5 + DRC Industrial
// ===================================================================
TEST(e2e_drc_with_sky130_after_placement) {
    PhysicalDesign pd = build_test_pd(20);

    // Step 1: Place cells
    AnalyticalPlacer placer(pd);
    auto place_res = placer.place();
    CHECK(place_res.legal, "placement should be legal");

    // Step 2: Add wires to simulate routing
    add_routing_wires(pd, 15);

    // Step 3: Load SKY130 DRC rules and run DRC
    DrcEngine drc(pd);
    drc.load_sky130_rules();
    CHECK(drc.rule_count() > 200,
          "SKY130 should have > 200 rules, got " + std::to_string(drc.rule_count()));

    auto drc_res = drc.check();
    CHECK(drc_res.total_rules > 0,
          "DRC total_rules should be > 0, got " + std::to_string(drc_res.total_rules));
    CHECK(drc_res.time_ms >= 0,
          "DRC time_ms should be >= 0");

    PASS("e2e_drc_with_sky130_after_placement");
}

// ===================================================================
//  6. e2e_full_signoff_flow — THE BIG ONE
// ===================================================================
TEST(e2e_full_signoff_flow) {
    // (a) Build sequential netlist + PhysicalDesign
    auto nl = build_pipeline_circuit();
    PhysicalDesign pd = build_test_pd(20);

    // (b) Retiming pass (Gap 2)
    RetimingEngine retime;
    auto retime_res = retime.optimize_with_result(nl);
    CHECK(retime_res.critical_path_before > 0,
          "retiming: critical_path_before should be > 0");
    std::cout << "    [signoff] Retiming: cp_before=" << retime_res.critical_path_before
              << " cp_after=" << retime_res.critical_path_after << "\n";

    // (c) Timing-driven placement (Gap 6)
    AnalyticalPlacer placer(pd);
    placer.enable_timing_driven(nl, 10.0);
    auto place_res = placer.place();
    CHECK(place_res.legal, "placement should be legal");
    CHECK(place_res.hpwl > 0, "HPWL should be > 0");
    std::cout << "    [signoff] Placement: HPWL=" << place_res.hpwl
              << " timing_iters=" << place_res.timing_iterations << "\n";

    // (d) Add wires to simulate routing
    add_routing_wires(pd, 15);
    CHECK(!pd.wires.empty(), "wires should exist after routing");

    // (e) Post-route optimization (Gap 1)
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(10.0);
    auto opt_res = opt.optimize();
    CHECK(opt_res.iterations > 0, "post-route should run iterations");
    std::cout << "    [signoff] PostRoute: WNS " << opt_res.wns_before
              << " -> " << opt_res.wns_after
              << " iters=" << opt_res.iterations << "\n";

    // (f) STA with AOCV (Gap 3)
    StaEngine sta(nl, nullptr, &pd);
    sta.set_ocv_mode(OcvMode::AOCV);
    sta.enable_aocv(0.10, 0.10);
    auto sta_res = sta.analyze(10.0, 5);
    CHECK(sta_res.num_endpoints > 0, "STA should have endpoints");
    CHECK(sta_res.ocv_mode == OcvMode::AOCV, "STA should use AOCV");
    std::cout << "    [signoff] STA AOCV: WNS=" << sta_res.wns
              << " endpoints=" << sta_res.num_endpoints << "\n";

    // (g) MCMM with foundry corners (Gap 4)
    McmmAnalyzer mcmm(nl, nullptr, &pd);
    mcmm.load_foundry_corners();
    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);
    auto mcmm_res = mcmm.analyze();
    CHECK(mcmm_res.scenarios == 7, "MCMM should have 7 scenarios");
    std::cout << "    [signoff] MCMM: scenarios=" << mcmm_res.scenarios
              << " worst_setup_wns=" << mcmm_res.worst_setup_wns << "\n";

    // (h) DRC with SKY130 rules (Gap 5 + DRC Industrial)
    DrcEngine drc(pd);
    drc.load_sky130_rules();
    CHECK(drc.rule_count() > 200,
          "DRC should have > 200 loaded rules, got " + std::to_string(drc.rule_count()));
    auto drc_res = drc.check();
    CHECK(drc_res.total_rules > 0, "DRC total_rules should be > 0");
    std::cout << "    [signoff] DRC SKY130: rules=" << drc.rule_count()
              << " violations=" << drc_res.violations << "\n";

    PASS("e2e_full_signoff_flow");
}

// ===================================================================
//  7. e2e_mcmm_full_sweep — Stress test for Gap 4
// ===================================================================
TEST(e2e_mcmm_full_sweep) {
    auto nl = build_seq_circuit();
    McmmAnalyzer mcmm(nl);

    // Load 7 foundry corners
    mcmm.load_foundry_corners();

    // Add 3 functional modes
    FunctionalMode m1; m1.name = "func_high"; m1.clock_freq_mhz = 1000;
    FunctionalMode m2; m2.name = "func_norm"; m2.clock_freq_mhz = 500;
    FunctionalMode m3; m3.name = "func_low";  m3.clock_freq_mhz = 200;
    mcmm.add_mode(m1);
    mcmm.add_mode(m2);
    mcmm.add_mode(m3);

    auto r = mcmm.analyze();
    CHECK(r.scenarios == 21,
          "expected 21 scenarios (7×3), got " + std::to_string(r.scenarios));
    CHECK(static_cast<int>(r.details.size()) == 21,
          "expected 21 detail entries, got " + std::to_string(r.details.size()));

    // Verify all details are populated
    for (int i = 0; i < static_cast<int>(r.details.size()); i++) {
        CHECK(!r.details[i].scenario_name.empty(),
              "scenario " + std::to_string(i) + " name should not be empty");
        CHECK(!r.details[i].corner.name.empty(),
              "scenario " + std::to_string(i) + " corner name should not be empty");
        CHECK(!r.details[i].mode.name.empty(),
              "scenario " + std::to_string(i) + " mode name should not be empty");
    }

    // Verify worst-case tracking is correct
    double manual_worst = 0;
    for (auto& sc : r.details) {
        if (sc.sta.wns < manual_worst)
            manual_worst = sc.sta.wns;
    }
    CHECK(std::fabs(r.worst_setup_wns - manual_worst) < 1e-6,
          "worst_setup_wns mismatch across 21 scenarios: " +
          std::to_string(r.worst_setup_wns) + " vs " + std::to_string(manual_worst));

    PASS("e2e_mcmm_full_sweep");
}

// ===================================================================
//  8. e2e_drc_export_reimport_check — DRC pipeline round-trip
// ===================================================================
TEST(e2e_drc_export_reimport_check) {
    PhysicalDesign pd = build_test_pd(10);
    add_routing_wires(pd, 8);

    // Load SKY130 rules and run DRC
    DrcEngine drc1(pd);
    drc1.load_sky130_rules();
    auto res1 = drc1.check();
    CHECK(drc1.rule_count() > 200,
          "original should have > 200 rules");

    // Export to JSON
    const std::string tmpfile = "/tmp/sf_phase19_drc_roundtrip.json";
    bool wrote = drc1.write_rules_to_file(tmpfile);
    CHECK(wrote, "write_rules_to_file should succeed");

    // Reimport into a new DrcEngine
    DrcEngine drc2(pd);
    int loaded = drc2.load_rules_from_file(tmpfile);
    CHECK(loaded > 200,
          "reimported rules should be > 200, got " + std::to_string(loaded));
    CHECK(drc2.rule_count() == drc1.rule_count(),
          "reimported rule count should match: " +
          std::to_string(drc2.rule_count()) + " vs " +
          std::to_string(drc1.rule_count()));

    // Run DRC with reimported rules on same design
    auto res2 = drc2.check();
    CHECK(res2.violations == res1.violations,
          "reimported DRC violations should match: " +
          std::to_string(res2.violations) + " vs " + std::to_string(res1.violations));

    // Cleanup
    std::remove(tmpfile.c_str());
    PASS("e2e_drc_export_reimport_check");
}

// ===================================================================
//  9. e2e_aocv_deep_pipeline — Stress-test AOCV depth at scale
// ===================================================================
TEST(e2e_aocv_deep_pipeline) {
    auto nl = build_deep_pipeline();

    // Run STA with AOCV
    StaEngine sta(nl);
    sta.set_ocv_mode(OcvMode::AOCV);
    sta.enable_aocv(0.10, 0.10);
    auto sta_res = sta.analyze(10.0, 5);
    CHECK(sta_res.num_endpoints > 0,
          "STA should have endpoints on deep pipeline");

    // Verify critical path depth >= 10
    bool has_deep = false;
    for (auto& path : sta_res.critical_paths) {
        if (path.depth >= 10) { has_deep = true; break; }
    }
    // Also accept if gates in the path >= 10
    if (!has_deep) {
        for (auto& path : sta_res.critical_paths) {
            if (static_cast<int>(path.gates.size()) >= 10) { has_deep = true; break; }
        }
    }
    CHECK(has_deep,
          "critical path should have depth >= 10 for 10-gate chain");

    // Run MCMM with 3 corners, each with AOCV
    McmmAnalyzer mcmm(nl);
    mcmm.load_default_corners(); // 3 corners

    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);

    auto mcmm_res = mcmm.analyze();
    CHECK(mcmm_res.scenarios == 3,
          "expected 3 scenarios, got " + std::to_string(mcmm_res.scenarios));
    CHECK(static_cast<int>(mcmm_res.details.size()) == 3,
          "expected 3 detail entries");

    PASS("e2e_aocv_deep_pipeline");
}

// ===================================================================
//  10. e2e_all_upgrades_summary — Final comprehensive summary
// ===================================================================
TEST(e2e_all_upgrades_summary) {
    // Build a reasonable circuit
    auto nl = build_pipeline_circuit();
    PhysicalDesign pd = build_test_pd(20);

    // 1. Retiming (Gap 2)
    RetimingEngine retime;
    auto retime_res = retime.optimize_with_result(nl);
    CHECK(retime_res.critical_path_before > 0,
          "retiming cp_before should be > 0");

    // 2. Timing-driven placement (Gap 6)
    AnalyticalPlacer placer(pd);
    placer.enable_timing_driven(nl, 10.0);
    auto place_res = placer.place();
    CHECK(place_res.legal, "placement should be legal");
    CHECK(place_res.hpwl > 0, "HPWL should be > 0");

    // 3. Add routing wires
    add_routing_wires(pd, 15);

    // 4. Post-route optimization (Gap 1)
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(10.0);
    auto opt_res = opt.optimize();
    CHECK(opt_res.iterations > 0, "post-route should run iterations");

    // 5. STA with AOCV (Gap 3)
    StaEngine sta(nl, nullptr, &pd);
    sta.set_ocv_mode(OcvMode::AOCV);
    sta.enable_aocv(0.10, 0.10);
    auto sta_res = sta.analyze(10.0, 5);
    CHECK(sta_res.num_endpoints > 0, "STA should have endpoints");

    // 6. MCMM with foundry corners (Gap 4)
    McmmAnalyzer mcmm(nl, nullptr, &pd);
    mcmm.load_foundry_corners();
    FunctionalMode m;
    m.name = "func"; m.clock_freq_mhz = 500;
    mcmm.add_mode(m);
    auto mcmm_res = mcmm.analyze();
    CHECK(mcmm_res.scenarios == 7, "MCMM should have 7 scenarios");

    // 7. DRC with SKY130 rules (Gap 5)
    DrcEngine drc(pd);
    drc.load_sky130_rules();
    CHECK(drc.rule_count() > 200,
          "DRC should have > 200 loaded rules, got " + std::to_string(drc.rule_count()));
    auto drc_res = drc.check();
    CHECK(drc_res.total_rules > 0, "DRC total_rules should be > 0");

    // Print comprehensive summary
    std::cout << "\n"
        "    ╔══════════════════════════════════════════════════════╗\n"
        "    ║  INDUSTRIAL UPGRADE E2E SUMMARY                      ║\n"
        "    ╠══════════════════════════════════════════════════════╣\n";
    std::cout << "    ║  Retiming:           ✓ (cp_before="
              << retime_res.critical_path_before
              << ", cp_after=" << retime_res.critical_path_after << ")\n";
    std::cout << "    ║  Timing-Driven Place: ✓ (HPWL="
              << place_res.hpwl
              << ", timing_iters=" << place_res.timing_iterations << ")\n";
    std::cout << "    ║  Post-Route Opt:     ✓ (WNS: "
              << opt_res.wns_before << "→" << opt_res.wns_after
              << ", iters=" << opt_res.iterations << ")\n";
    std::cout << "    ║  STA OCV/AOCV:       ✓ (WNS="
              << sta_res.wns << ", mode=AOCV)\n";
    std::cout << "    ║  MCMM:               ✓ (scenarios="
              << mcmm_res.scenarios
              << ", worst_wns=" << mcmm_res.worst_setup_wns << ")\n";
    std::cout << "    ║  DRC SKY130:         ✓ (rules="
              << drc.rule_count()
              << ", violations=" << drc_res.violations << ")\n";
    std::cout <<
        "    ╚══════════════════════════════════════════════════════╝\n\n";

    PASS("e2e_all_upgrades_summary");
}

// ===================================================================
// Main
// ===================================================================
int main() {
    std::cout << "\n"
        "╔══════════════════════════════════════════════════════╗\n"
        "║  SiliconForge Phase 19 — E2E Industrial Integration   ║\n"
        "╚══════════════════════════════════════════════════════╝\n\n";

    std::cout << "── STA + MCMM Integration ──\n";
    RUN(e2e_sta_ocv_to_mcmm);
    RUN(e2e_aocv_deep_pipeline);
    RUN(e2e_mcmm_full_sweep);

    std::cout << "\n── PnR Pipeline ──\n";
    RUN(e2e_place_then_sta);
    RUN(e2e_place_route_postopt);
    RUN(e2e_retiming_then_sta);

    std::cout << "\n── DRC Pipeline ──\n";
    RUN(e2e_drc_with_sky130_after_placement);
    RUN(e2e_drc_export_reimport_check);

    std::cout << "\n── Full Signoff Flow ──\n";
    RUN(e2e_full_signoff_flow);
    RUN(e2e_all_upgrades_summary);

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "Phase 19 Results: " << passed << " passed, "
              << failed << " failed\n";
    std::cout << "════════════════════════════════════════\n";
    return failed ? 1 : 0;
}
