// SiliconForge — Phase 2 Test Suite
// Tests: k-induction, netlist, simulator, lint, PODEM ATPG, fault simulation

#include "core/types.hpp"
#include "core/aig.hpp"
#include "core/netlist.hpp"
#include "sat/cdcl_solver.hpp"
#include "formal/k_induction.hpp"
#include "sim/simulator.hpp"
#include "lint/lint_engine.hpp"
#include "dft/podem.hpp"
#include "dft/fault_sim.hpp"
#include <iostream>
#include <cassert>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// Helper: build a simple AND gate netlist
static Netlist build_and_netlist() {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, y, "G1");
    return nl;
}

// Helper: build a 2-bit counter netlist
static Netlist build_counter_netlist() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId q0 = nl.add_net("q0"); nl.mark_output(q0);
    NetId q1 = nl.add_net("q1"); nl.mark_output(q1);
    NetId not_q0 = nl.add_net("not_q0");
    NetId xor_out = nl.add_net("xor_out");

    nl.add_gate(GateType::NOT, {q0}, not_q0);
    nl.add_gate(GateType::XOR, {q1, q0}, xor_out);
    nl.add_dff(not_q0, clk, q0, -1, "FF0");
    nl.add_dff(xor_out, clk, q1, -1, "FF1");
    return nl;
}

// ============================================================================
// Test: Netlist
// ============================================================================
TEST(netlist_basic) {
    auto nl = build_and_netlist();
    CHECK(nl.num_nets() == 3, "3 nets");
    CHECK(nl.num_gates() == 1, "1 gate");
    CHECK(nl.primary_inputs().size() == 2, "2 PIs");
    CHECK(nl.primary_outputs().size() == 1, "1 PO");

    auto order = nl.topo_order();
    CHECK(order.size() == 1, "1 gate in topo order");
    PASS("netlist_basic");
}

TEST(netlist_eval_gate) {
    CHECK(Netlist::eval_gate(GateType::AND, {Logic4::ONE, Logic4::ONE}) == Logic4::ONE, "1&1=1");
    CHECK(Netlist::eval_gate(GateType::AND, {Logic4::ONE, Logic4::ZERO}) == Logic4::ZERO, "1&0=0");
    CHECK(Netlist::eval_gate(GateType::OR, {Logic4::ZERO, Logic4::ONE}) == Logic4::ONE, "0|1=1");
    CHECK(Netlist::eval_gate(GateType::NOT, {Logic4::ONE}) == Logic4::ZERO, "!1=0");
    CHECK(Netlist::eval_gate(GateType::XOR, {Logic4::ONE, Logic4::ONE}) == Logic4::ZERO, "1^1=0");
    CHECK(Netlist::eval_gate(GateType::NAND, {Logic4::ONE, Logic4::ONE}) == Logic4::ZERO, "NAND");
    PASS("netlist_eval_gate");
}

TEST(netlist_counter) {
    auto nl = build_counter_netlist();
    CHECK(nl.flip_flops().size() == 2, "2 FFs");
    CHECK(nl.num_nets() == 5, "5 nets");
    PASS("netlist_counter");
}

// ============================================================================
// Test: Event-Driven Simulator
// ============================================================================
TEST(sim_and_gate) {
    auto nl = build_and_netlist();
    EventSimulator sim(nl);

    std::vector<TestVector> vectors = {
        {0, {{0, Logic4::ZERO}, {1, Logic4::ZERO}}},
        {10, {{0, Logic4::ONE}, {1, Logic4::ZERO}}},
        {20, {{0, Logic4::ONE}, {1, Logic4::ONE}}},
        {30, {{0, Logic4::ZERO}, {1, Logic4::ONE}}},
    };
    sim.run(vectors);

    auto& trace = sim.trace();
    // At time index 0 (init): X
    // At time index 1 (t=0): 0&0=0
    // At time index 2 (t=10): 1&0=0
    // At time index 3 (t=20): 1&1=1
    // At time index 4 (t=30): 0&1=0

    NetId y = nl.primary_outputs()[0];
    auto& yt = trace.traces.at(y);
    CHECK(yt.size() >= 5, "enough time points");
    CHECK(yt[1] == Logic4::ZERO, "0&0=0");
    CHECK(yt[2] == Logic4::ZERO, "1&0=0");
    CHECK(yt[3] == Logic4::ONE, "1&1=1");
    CHECK(yt[4] == Logic4::ZERO, "0&1=0");
    PASS("sim_and_gate");
}

TEST(sim_vcd_output) {
    auto nl = build_and_netlist();
    EventSimulator sim(nl);
    std::vector<TestVector> vectors = {
        {0, {{0, Logic4::ONE}, {1, Logic4::ONE}}},
    };
    sim.run(vectors);
    auto vcd = sim.generate_vcd("test");
    CHECK(vcd.find("$timescale") != std::string::npos, "VCD has timescale");
    CHECK(vcd.find("$var") != std::string::npos, "VCD has var defs");
    CHECK(vcd.find("#0") != std::string::npos, "VCD has time 0");
    PASS("sim_vcd_output");
}

// ============================================================================
// Test: Lint Engine
// ============================================================================
TEST(lint_clean_circuit) {
    auto nl = build_and_netlist();
    LintEngine lint(nl);
    auto violations = lint.run_all();
    // Clean circuit should have no errors
    int errors = 0;
    for (auto& v : violations)
        if (v.severity == LintViolation::ERROR) errors++;
    CHECK(errors == 0, "no lint errors on clean circuit");
    PASS("lint_clean_circuit");
}

TEST(lint_undriven_net) {
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); // NOT marked as input, NOT driven
    NetId y = nl.add_net("y"); nl.mark_output(y);
    nl.add_gate(GateType::AND, {a, b}, y);

    LintEngine lint(nl);
    auto v = lint.check_undriven_nets();
    CHECK(!v.empty(), "undriven net detected");
    CHECK(v[0].rule == "LINT-001", "correct rule");
    PASS("lint_undriven_net");
}

// ============================================================================
// Test: k-Induction
// ============================================================================
TEST(k_induction_safe) {
    // 1-bit toggle: never reaches X (trivially safe with strong enough k)
    AigGraph g;
    AigLit bit = g.create_input("bit");
    AigLit next = aig_not(bit);
    g.add_latch(next, AIG_FALSE, "bit");
    // Property: bit is always 0 OR always 1 — actually, let's check bit != X
    // Simpler: counter mod 2 always < 2 (trivially true)
    // Bad state = FALSE (never bad)
    g.add_output(AIG_FALSE, "bad"); // Property always holds

    auto r = KInduction::check(g, 5);
    CHECK(r.status == KIndResult::PROVEN, "trivially safe property proven");
    PASS("k_induction_safe");
}

TEST(k_induction_unsafe) {
    // 2-bit counter reaching 3 — same as BMC test
    AigGraph ctr;
    ctr.create_input("clk");
    AigLit b0 = ctr.create_input("b0");
    AigLit b1 = ctr.create_input("b1");
    ctr.add_latch(aig_not(b0), AIG_FALSE, "b0");
    ctr.add_latch(ctr.create_xor(b1, b0), AIG_FALSE, "b1");
    ctr.add_output(ctr.create_and(b0, b1), "bad");

    auto r = KInduction::check(ctr, 10);
    CHECK(r.status == KIndResult::FAILED, "unsafe counter detected");
    PASS("k_induction_unsafe");
}

// ============================================================================
// Test: PODEM ATPG
// ============================================================================
TEST(podem_and_gate) {
    auto nl = build_and_netlist();
    PodemAtpg atpg(nl);

    // SA1 on input a: to detect, need a=0 and b=1
    // Good: a=0,b=1 -> y=0. Faulty: a=1(stuck),b=1 -> y=1. Different!
    Fault f{0, Logic4::ONE}; // input a SA1
    auto r = atpg.generate_test(f);
    CHECK(r.detected, "SA1 on AND input detected");
    PASS("podem_and_gate");
}

TEST(podem_full_coverage) {
    auto nl = build_and_netlist();
    PodemAtpg atpg(nl);
    auto fc = atpg.run_full_atpg();
    CHECK(fc.total_faults == 6, "3 nets × 2 = 6 faults");
    CHECK(fc.detected > 0, "some faults detected");
    CHECK(fc.coverage_pct() > 0.0, "coverage > 0%");
    PASS("podem_full_coverage");
}

// ============================================================================
// Test: Fault Simulator
// ============================================================================
TEST(fault_sim_basic) {
    auto nl = build_and_netlist();
    FaultSimulator fsim(nl);

    // Provide all 4 input combinations
    std::vector<std::vector<Logic4>> vectors = {
        {Logic4::ZERO, Logic4::ZERO},
        {Logic4::ZERO, Logic4::ONE},
        {Logic4::ONE, Logic4::ZERO},
        {Logic4::ONE, Logic4::ONE},
    };
    auto r = fsim.simulate(vectors);
    CHECK(r.total_faults == 6, "6 faults total");
    CHECK(r.detected > 0, "faults detected");
    CHECK(r.coverage_pct() > 50.0, "reasonable coverage");
    PASS("fault_sim_basic");
}

// ============================================================================
int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 2 — Test Suite               ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Netlist ──\n";
    RUN(netlist_basic);
    RUN(netlist_eval_gate);
    RUN(netlist_counter);

    std::cout << "\n── Event-Driven Simulator ──\n";
    RUN(sim_and_gate);
    RUN(sim_vcd_output);

    std::cout << "\n── Lint Engine ──\n";
    RUN(lint_clean_circuit);
    RUN(lint_undriven_net);

    std::cout << "\n── k-Induction ──\n";
    RUN(k_induction_safe);
    RUN(k_induction_unsafe);

    std::cout << "\n── PODEM ATPG ──\n";
    RUN(podem_and_gate);
    RUN(podem_full_coverage);

    std::cout << "\n── Fault Simulator ──\n";
    RUN(fault_sim_basic);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
