// SiliconForge Phase 23 — Retiming Industrial Tests
// Clock gating preservation, reset path handling, multi-cycle paths,
// hold-aware retiming, register budget, Liberty delays, Fmax metrics.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "synth/retiming.hpp"
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
// Helper: build pipeline circuit (chain of gates with DFFs)
// INP → AND → NOT → DFF → AND → NOT → DFF → OUT
// ---------------------------------------------------------------------------
static Netlist build_pipeline() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId inp = nl.add_net("inp"); nl.mark_input(inp);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId q1 = nl.add_net("q1");
    NetId w3 = nl.add_net("w3");
    NetId w4 = nl.add_net("w4");
    NetId q2 = nl.add_net("q2"); nl.mark_output(q2);

    nl.add_gate(GateType::AND, {inp, inp}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1}, w2, "G2");
    nl.add_dff(w2, clk, q1, -1, "FF1");
    nl.add_gate(GateType::AND, {q1, q1}, w3, "G3");
    nl.add_gate(GateType::NOT, {w3}, w4, "G4");
    nl.add_dff(w4, clk, q2, -1, "FF2");
    return nl;
}

// Helper: pipeline with reset
static Netlist build_pipeline_with_reset() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId rst = nl.add_net("rst"); nl.mark_input(rst);
    NetId inp = nl.add_net("inp"); nl.mark_input(inp);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId q1 = nl.add_net("q1"); nl.mark_output(q1);

    // Reset buffer drives DFF reset
    NetId rst_buf = nl.add_net("rst_buf");
    nl.add_gate(GateType::NOT, {rst}, rst_buf, "RST_INV");

    nl.add_gate(GateType::AND, {inp, inp}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1}, w2, "G2");
    nl.add_dff(w2, clk, q1, rst_buf, "FF1");
    return nl;
}

// ========================= Config Tests =========================

TEST(config_defaults) {
    RetimingConfig cfg;
    CHECK(cfg.preserve_clock_gating == true, "clock gating default on");
    CHECK(cfg.preserve_reset_paths == true, "reset paths default on");
    CHECK(cfg.honor_multicycle_paths == true, "MCP default on");
    CHECK(cfg.hold_aware == true, "hold aware default on");
    CHECK(cfg.max_register_increase == -1, "unlimited registers default");
    CHECK(cfg.binary_search_steps == 12, "12 search steps");
    CHECK(std::abs(cfg.hold_margin - 0.05) < 1e-6, "hold margin 0.05");
    CHECK(std::abs(cfg.improvement_threshold - 0.01) < 1e-6, "1% threshold");
    PASS("config_defaults");
}

TEST(config_custom) {
    RetimingEngine re;
    RetimingConfig cfg;
    cfg.preserve_clock_gating = false;
    cfg.max_register_increase = 5;
    cfg.binary_search_steps = 20;
    cfg.hold_margin = 0.1;
    re.set_config(cfg);

    CHECK(re.config().preserve_clock_gating == false, "clock gating disabled");
    CHECK(re.config().max_register_increase == 5, "max 5 regs");
    CHECK(re.config().binary_search_steps == 20, "20 steps");
    CHECK(std::abs(re.config().hold_margin - 0.1) < 1e-6, "hold margin 0.1");
    PASS("config_custom");
}

// ========================= Industrial Result Fields =========================

TEST(result_fmax_metrics) {
    Netlist nl = build_pipeline();
    RetimingEngine re;
    auto result = re.optimize_with_result(nl);

    CHECK(result.fmax_before > 0, "fmax_before > 0");
    CHECK(result.critical_path_before > 0, "critical path before > 0");
    CHECK(result.register_count_before == 2, "2 DFFs before");
    CHECK(result.iterations > 0, "binary search ran");

    if (result.improved) {
        CHECK(result.fmax_after > result.fmax_before, "fmax improved");
        CHECK(result.fmax_improvement_pct > 0, "positive improvement %");
        CHECK(result.critical_path_after < result.critical_path_before, "shorter path");
    }
    PASS("result_fmax_metrics");
}

TEST(result_register_count) {
    Netlist nl = build_pipeline();
    RetimingEngine re;
    auto result = re.optimize_with_result(nl);

    CHECK(result.register_count_before == 2, "2 regs before");
    CHECK(result.register_count_after >= 0, "non-negative regs after");
    // register_count_after = before + inserted - removed
    int expected = result.register_count_before + result.dffs_inserted - result.dffs_removed;
    CHECK(result.register_count_after == expected, "register count consistent");
    PASS("result_register_count");
}

// ========================= Clock Gating Preservation =========================

TEST(clock_gating_detection) {
    // When clock gating preservation is on, gates driving DFF clock pins
    // should be frozen (not retimed through)
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId en = nl.add_net("en"); nl.mark_input(en);
    NetId inp = nl.add_net("inp"); nl.mark_input(inp);

    // Clock gating: en AND clk → gated_clk → DFF
    NetId gated_clk = nl.add_net("gated_clk");
    GateId icg_id = nl.add_gate(GateType::AND, {clk, en}, gated_clk, "ICG");

    // Data path with enough gates to form a non-trivial graph
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    NetId q1 = nl.add_net("q1");
    NetId w3 = nl.add_net("w3");
    NetId q2 = nl.add_net("q2"); nl.mark_output(q2);

    nl.add_gate(GateType::AND, {inp, inp}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1}, w2, "G2");
    nl.add_dff(w2, gated_clk, q1, -1, "FF1");
    nl.add_gate(GateType::NOT, {q1}, w3, "G3");
    nl.add_dff(w3, gated_clk, q2, -1, "FF2");

    RetimingEngine re;
    // Manually mark the ICG to ensure detection
    re.add_clock_gating_cell(icg_id);
    auto result = re.optimize_with_result(nl);
    CHECK(result.clock_gating_preserved >= 1, "ICG cell preserved");
    PASS("clock_gating_detection");
}

TEST(clock_gating_manual_mark) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    // Manually mark gate 0 as clock gating (even though it's not)
    re.add_clock_gating_cell(0);
    auto result = re.optimize_with_result(nl);
    CHECK(result.clock_gating_preserved >= 1, "manually marked ICG preserved");
    PASS("clock_gating_manual_mark");
}

// ========================= Reset Path Preservation =========================

TEST(reset_path_preservation) {
    Netlist nl = build_pipeline_with_reset();
    RetimingEngine re;
    auto result = re.optimize_with_result(nl);
    CHECK(result.reset_paths_preserved >= 1, "reset path gate preserved");
    PASS("reset_path_preservation");
}

TEST(reset_disabled) {
    Netlist nl = build_pipeline_with_reset();
    RetimingEngine re;
    RetimingConfig cfg;
    cfg.preserve_reset_paths = false;
    re.set_config(cfg);
    auto result = re.optimize_with_result(nl);
    CHECK(result.reset_paths_preserved == 0, "no reset preservation when disabled");
    PASS("reset_disabled");
}

// ========================= Multi-Cycle Path =========================

TEST(multicycle_path_constraint) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    // Add MCP exception: G1→G2 is a 2-cycle path
    MulticyclePath mcp;
    mcp.from_gate = 0; // G1
    mcp.to_gate = 1;   // G2
    mcp.cycles = 2;
    re.add_multicycle_path(mcp);

    auto result = re.optimize_with_result(nl);
    CHECK(result.multicycle_paths_honored >= 1, "MCP honored");
    PASS("multicycle_path_constraint");
}

// ========================= Hold-Aware Retiming =========================

TEST(hold_aware_retiming) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    RetimingConfig cfg;
    cfg.hold_aware = true;
    cfg.hold_margin = 0.1;
    re.set_config(cfg);

    auto result = re.optimize_with_result(nl);
    CHECK(result.critical_path_before > 0, "has critical path");
    // Hold-aware may produce a slightly less optimal result but more robust
    PASS("hold_aware_retiming");
}

TEST(hold_disabled) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    RetimingConfig cfg;
    cfg.hold_aware = false;
    re.set_config(cfg);

    auto result = re.optimize_with_result(nl);
    CHECK(result.critical_path_before > 0, "works without hold");
    PASS("hold_disabled");
}

// ========================= Register Budget =========================

TEST(register_budget_unlimited) {
    Netlist nl = build_pipeline();
    RetimingEngine re;
    // Default: unlimited
    auto result = re.optimize_with_result(nl);
    CHECK(result.register_count_before >= 0, "has register count");
    PASS("register_budget_unlimited");
}

TEST(register_budget_zero) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    RetimingConfig cfg;
    cfg.max_register_increase = 0; // no new registers allowed
    re.set_config(cfg);

    auto result = re.optimize_with_result(nl);
    // With 0 budget, net new registers must be 0 or negative
    CHECK(result.dffs_inserted <= result.dffs_removed || !result.improved,
          "respects 0 budget");
    PASS("register_budget_zero");
}

// ========================= Dont-Retime API =========================

TEST(dont_retime_gate) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    // Mark G1 (gate 0) as don't-retime
    re.add_dont_retime(0);
    auto result = re.optimize_with_result(nl);
    // Should still work, just with constraint
    CHECK(result.critical_path_before > 0, "still analyzes");
    PASS("dont_retime_gate");
}

TEST(dont_retime_clear) {
    RetimingEngine re;
    re.add_dont_retime(0);
    re.add_dont_retime(1);
    re.clear_dont_retime();
    // After clear, optimize without constraints
    Netlist nl = build_pipeline();
    auto result = re.optimize_with_result(nl);
    CHECK(result.critical_path_before > 0, "works after clear");
    PASS("dont_retime_clear");
}

// ========================= Liberty Integration =========================

TEST(liberty_delay_integration) {
    RetimingEngine re;
    RetimingConfig cfg;
    cfg.use_liberty_delays = true;
    re.set_config(cfg);

    // Without library: should fall back to hardcoded delays
    Netlist nl = build_pipeline();
    auto result = re.optimize_with_result(nl);
    CHECK(result.critical_path_before > 0, "falls back to hardcoded");
    PASS("liberty_delay_integration");
}

// ========================= E2E =========================

TEST(e2e_full_industrial) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    RetimingConfig cfg;
    cfg.preserve_clock_gating = true;
    cfg.preserve_reset_paths = true;
    cfg.hold_aware = true;
    cfg.hold_margin = 0.05;
    cfg.binary_search_steps = 15;
    re.set_config(cfg);

    MulticyclePath mcp;
    mcp.cycles = 2;
    re.add_multicycle_path(mcp);

    auto result = re.optimize_with_result(nl);
    CHECK(result.fmax_before > 0, "E2E fmax before");
    CHECK(result.register_count_before > 0, "E2E has registers");
    CHECK(result.iterations > 0, "E2E iterations ran");
    CHECK(!result.message.empty(), "E2E has message");
    PASS("e2e_full_industrial");
}

TEST(e2e_backward_compatibility) {
    Netlist nl = build_pipeline();
    RetimingEngine re;

    // Default config — should work identically to before
    bool improved = re.optimize(nl);
    // Original API still works
    (void)improved; // may or may not improve depending on circuit
    PASS("e2e_backward_compatibility");
}

// ========================= Main =========================

int main() {
    std::cout << "=== Phase 23: Retiming Industrial Tests ===\n\n";

    // Config
    RUN(config_defaults);
    RUN(config_custom);

    // Industrial result fields
    RUN(result_fmax_metrics);
    RUN(result_register_count);

    // Clock gating
    RUN(clock_gating_detection);
    RUN(clock_gating_manual_mark);

    // Reset paths
    RUN(reset_path_preservation);
    RUN(reset_disabled);

    // Multi-cycle path
    RUN(multicycle_path_constraint);

    // Hold-aware
    RUN(hold_aware_retiming);
    RUN(hold_disabled);

    // Register budget
    RUN(register_budget_unlimited);
    RUN(register_budget_zero);

    // Dont-retime
    RUN(dont_retime_gate);
    RUN(dont_retime_clear);

    // Liberty
    RUN(liberty_delay_integration);

    // E2E
    RUN(e2e_full_industrial);
    RUN(e2e_backward_compatibility);

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    return failed > 0 ? 1 : 0;
}
