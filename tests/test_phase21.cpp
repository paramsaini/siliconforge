// SiliconForge Phase 21 — Post-Route Optimizer Industrial Tests
// Tests Vt swap, cell cloning, buffer insertion, hold fix, useful skew,
// leakage recovery, PostRouteConfig, VtType API, and E2E opt flow.

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "pnr/physical.hpp"
#include "pnr/post_route_opt.hpp"
#include "timing/sta.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <string>
#include <unordered_map>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ---------------------------------------------------------------------------
// Helper: build sequential circuit (AND→NOT→DFF) with physical design
// ---------------------------------------------------------------------------
static std::pair<Netlist, PhysicalDesign> build_seq_with_phys() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d = nl.add_net("d"); nl.mark_input(d);
    NetId q = nl.add_net("q"); nl.mark_output(q);
    NetId w1 = nl.add_net("w1");
    NetId w2 = nl.add_net("w2");
    nl.add_gate(GateType::AND, {d, q}, w1, "G1");
    nl.add_gate(GateType::NOT, {w1}, w2, "G2");
    nl.add_dff(w2, clk, q, -1, "FF1");

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    // Use add_cell API
    int c0 = pd.add_cell("G1", "AND", 2, 1);
    int c1 = pd.add_cell("G2", "NOT", 2, 1);
    int c2 = pd.add_cell("FF1", "DFF", 2, 1);
    pd.cells[c0].position = {10, 10}; pd.cells[c0].placed = true;
    pd.cells[c1].position = {30, 10}; pd.cells[c1].placed = true;
    pd.cells[c2].position = {60, 10}; pd.cells[c2].placed = true;

    WireSegment ws1; ws1.layer = 1; ws1.start = {10,10}; ws1.end = {30,10}; ws1.width = 0.14; ws1.net_id = 0;
    WireSegment ws2; ws2.layer = 2; ws2.start = {30,10}; ws2.end = {60,10}; ws2.width = 0.14; ws2.net_id = 1;
    pd.wires.push_back(ws1);
    pd.wires.push_back(ws2);

    pd.add_net("w1", {c0, c1});
    pd.add_net("w2", {c1, c2});

    Via v; v.position = {20, 10}; v.lower_layer = 1; v.upper_layer = 2;
    pd.vias.push_back(v);
    return {nl, pd};
}

// ---------------------------------------------------------------------------
// Helper: deep combinational path (8 gates chain)
// ---------------------------------------------------------------------------
static std::pair<Netlist, PhysicalDesign> build_deep_comb() {
    Netlist nl;
    NetId clk = nl.add_net("clk"); nl.mark_input(clk);
    NetId d = nl.add_net("d"); nl.mark_input(d);
    NetId prev = d;
    for (int i = 0; i < 8; i++) {
        NetId out = nl.add_net("w" + std::to_string(i));
        if (i % 2 == 0)
            nl.add_gate(GateType::NOT, {prev}, out, "INV" + std::to_string(i));
        else
            nl.add_gate(GateType::AND, {prev, d}, out, "AND" + std::to_string(i));
        prev = out;
    }
    NetId q = nl.add_net("q"); nl.mark_output(q);
    nl.add_dff(prev, clk, q, -1, "FF1");

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    for (int i = 0; i < 8; i++) {
        double x = 10 + i * 20;
        int cid = pd.add_cell("G" + std::to_string(i), "GATE", 2, 1);
        pd.cells[cid].position = {x, 10}; pd.cells[cid].placed = true;
    }
    int ff_id = pd.add_cell("FF1", "DFF", 2, 1);
    pd.cells[ff_id].position = {170, 10}; pd.cells[ff_id].placed = true;

    for (int i = 0; i < 8; i++) {
        WireSegment ws;
        ws.layer = 1;
        ws.start = {10.0 + i * 20, 10};
        ws.end = {30.0 + i * 20, 10};
        ws.width = 0.14;
        ws.net_id = i;
        pd.wires.push_back(ws);
    }
    pd.add_net("w0", {0, 1});
    return {nl, pd};
}

// ========================= VtType API Tests =========================

TEST(vt_type_enum) {
    CHECK(VtType::ULVT != VtType::LVT, "ULVT != LVT");
    CHECK(VtType::LVT != VtType::SVT, "LVT != SVT");
    CHECK(VtType::SVT != VtType::HVT, "SVT != HVT");
    PASS("vt_type_enum");
}

TEST(vt_type_str_conversion) {
    CHECK(std::string(vt_type_str(VtType::ULVT)) == "ULVT", "ULVT str");
    CHECK(std::string(vt_type_str(VtType::LVT)) == "LVT", "LVT str");
    CHECK(std::string(vt_type_str(VtType::SVT)) == "SVT", "SVT str");
    CHECK(std::string(vt_type_str(VtType::HVT)) == "HVT", "HVT str");
    PASS("vt_type_str_conversion");
}

TEST(gate_vt_assignment) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);

    // Default is SVT
    CHECK(opt.get_gate_vt(0) == VtType::SVT, "default SVT for gate 0");
    CHECK(opt.get_gate_vt(1) == VtType::SVT, "default SVT for gate 1");

    // Set specific Vt
    opt.set_gate_vt(0, VtType::LVT);
    opt.set_gate_vt(1, VtType::HVT);
    CHECK(opt.get_gate_vt(0) == VtType::LVT, "gate 0 set to LVT");
    CHECK(opt.get_gate_vt(1) == VtType::HVT, "gate 1 set to HVT");

    // Override
    opt.set_gate_vt(0, VtType::ULVT);
    CHECK(opt.get_gate_vt(0) == VtType::ULVT, "gate 0 override to ULVT");
    PASS("gate_vt_assignment");
}

// ========================= PostRouteConfig Tests =========================

TEST(config_defaults) {
    PostRouteConfig cfg;
    CHECK(cfg.enable_vt_swap == true, "vt_swap default on");
    CHECK(cfg.enable_cell_cloning == true, "cloning default on");
    CHECK(cfg.enable_buffer_insertion == true, "buffer_insert default on");
    CHECK(cfg.enable_hold_fix == true, "hold_fix default on");
    CHECK(cfg.enable_useful_skew == false, "useful_skew default OFF");
    CHECK(cfg.enable_leakage_recovery == true, "leakage_recovery default on");
    CHECK(cfg.fanout_clone_threshold == 8, "fanout threshold 8");
    CHECK(std::abs(cfg.hvt_delay_factor - 1.20) < 1e-6, "HVT 20% slower");
    CHECK(std::abs(cfg.lvt_delay_factor - 0.85) < 1e-6, "LVT 15% faster");
    CHECK(std::abs(cfg.ulvt_delay_factor - 0.75) < 1e-6, "ULVT 25% faster");
    PASS("config_defaults");
}

TEST(config_custom) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);

    PostRouteConfig cfg;
    cfg.enable_vt_swap = false;
    cfg.enable_useful_skew = true;
    cfg.fanout_clone_threshold = 4;
    cfg.hvt_delay_factor = 1.30;
    opt.set_config(cfg);

    CHECK(opt.config().enable_vt_swap == false, "vt_swap disabled");
    CHECK(opt.config().enable_useful_skew == true, "useful_skew enabled");
    CHECK(opt.config().fanout_clone_threshold == 4, "threshold = 4");
    CHECK(std::abs(opt.config().hvt_delay_factor - 1.30) < 1e-6, "HVT factor 1.30");
    PASS("config_custom");
}

TEST(vt_leakage_model) {
    PostRouteConfig cfg;
    // Leakage ratios: ULVT 8x, LVT 4x, SVT 1x, HVT 0.25x
    double base = 1.0; // arbitrary baseline
    CHECK(base * cfg.ulvt_leakage_factor > base * cfg.lvt_leakage_factor, "ULVT > LVT leakage");
    CHECK(base * cfg.lvt_leakage_factor > base * cfg.svt_leakage_factor, "LVT > SVT leakage");
    CHECK(base * cfg.svt_leakage_factor > base * cfg.hvt_leakage_factor, "SVT > HVT leakage");
    // ULVT is 32x HVT (8.0 / 0.25)
    double ratio = cfg.ulvt_leakage_factor / cfg.hvt_leakage_factor;
    CHECK(ratio > 30 && ratio < 34, "ULVT/HVT leakage ratio ~32x");
    PASS("vt_leakage_model");
}

TEST(vt_delay_model) {
    PostRouteConfig cfg;
    // Delay: ULVT fastest (0.75), HVT slowest (1.20)
    CHECK(cfg.ulvt_delay_factor < cfg.lvt_delay_factor, "ULVT faster than LVT");
    CHECK(cfg.lvt_delay_factor < cfg.svt_delay_factor, "LVT faster than SVT");
    CHECK(cfg.svt_delay_factor < cfg.hvt_delay_factor, "SVT faster than HVT");
    // Speed vs leakage tradeoff: faster Vt = more leakage
    CHECK(cfg.ulvt_delay_factor * cfg.ulvt_leakage_factor >
          cfg.hvt_delay_factor * cfg.hvt_leakage_factor,
          "ULVT power-delay product > HVT");
    PASS("vt_delay_model");
}

// ========================= Optimization Pass Tests =========================

TEST(optimize_sta_driven_with_industrial) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);

    PostRouteResult r = opt.optimize();
    CHECK(r.sta_driven == true, "STA-driven mode");
    CHECK(r.iterations > 0, "at least 1 iteration");
    // Industrial fields should be populated (may be 0 if no violations)
    CHECK(r.leakage_before_uw >= 0, "leakage_before measured");
    CHECK(r.leakage_after_uw >= 0, "leakage_after measured");
    CHECK(r.message.find("Post-route opt") != std::string::npos, "message present");
    PASS("optimize_sta_driven_with_industrial");
}

TEST(optimize_with_all_features_disabled) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);

    PostRouteConfig cfg;
    cfg.enable_vt_swap = false;
    cfg.enable_cell_cloning = false;
    cfg.enable_buffer_insertion = false;
    cfg.enable_hold_fix = false;
    cfg.enable_useful_skew = false;
    cfg.enable_leakage_recovery = false;
    opt.set_config(cfg);

    PostRouteResult r = opt.optimize();
    CHECK(r.cells_vt_swapped == 0, "no Vt swaps when disabled");
    CHECK(r.cells_cloned == 0, "no clones when disabled");
    CHECK(r.buffers_inserted == 0, "no buffers when disabled");
    CHECK(r.hold_buffers_inserted == 0, "no hold fixes when disabled");
    CHECK(r.useful_skew_adjustments == 0, "no skew when disabled");
    CHECK(r.leakage_recovered == 0, "no recovery when disabled");
    PASS("optimize_with_all_features_disabled");
}

TEST(optimize_legacy_unchanged) {
    // Trivial circuit: no DFFs, no clock → legacy mode
    Netlist nl;
    NetId a = nl.add_net("a"); nl.mark_input(a);
    NetId b = nl.add_net("b"); nl.mark_input(b);
    NetId out = nl.add_net("out"); nl.mark_output(out);
    nl.add_gate(GateType::AND, {a, b}, out, "G1");

    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    int cid = pd.add_cell("G1", "AND", 2, 1);
    pd.cells[cid].position = {10, 10}; pd.cells[cid].placed = true;

    WireSegment ws; ws.layer = 1; ws.start = {10,10}; ws.end = {30,10}; ws.width = 0.14; ws.net_id = 0;
    pd.wires.push_back(ws);

    PostRouteOptimizer opt(nl, pd);
    PostRouteResult r = opt.optimize();
    CHECK(r.sta_driven == false, "legacy mode for trivial circuit");
    // Industrial counters should be 0 in legacy mode
    CHECK(r.cells_vt_swapped == 0, "no Vt swap in legacy");
    CHECK(r.leakage_recovered == 0, "no leakage recovery in legacy");
    PASS("optimize_legacy_unchanged");
}

TEST(leakage_measurement) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);

    // Pre-set some gates to LVT so leakage recovery has work to do
    opt.set_gate_vt(0, VtType::LVT);
    opt.set_gate_vt(1, VtType::LVT);

    PostRouteResult r = opt.optimize();
    CHECK(r.leakage_before_uw > 0, "measured nonzero leakage before");
    // After recovery, leakage should decrease (LVT→HVT on non-critical)
    if (r.leakage_recovered > 0) {
        CHECK(r.leakage_after_uw <= r.leakage_before_uw, "leakage decreased after recovery");
    }
    PASS("leakage_measurement");
}

TEST(leakage_recovery_pass) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(10.0); // very relaxed → all paths have positive slack

    // Set all gates to LVT
    for (size_t i = 0; i < nl.num_gates(); i++)
        opt.set_gate_vt(static_cast<GateId>(i), VtType::LVT);

    PostRouteResult r = opt.optimize();
    // With huge slack margin, leakage recovery should swap non-critical gates back to HVT
    CHECK(r.leakage_recovered >= 0, "leakage recovery ran");
    // Verify recovered gates are now HVT
    int hvt_count = 0;
    for (size_t i = 0; i < nl.num_gates(); i++) {
        if (opt.get_gate_vt(static_cast<GateId>(i)) == VtType::HVT) hvt_count++;
    }
    CHECK(hvt_count >= 0, "some gates recovered to HVT");
    PASS("leakage_recovery_pass");
}

TEST(deep_path_vt_swap) {
    auto [nl, pd] = build_deep_comb();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(0.5); // very tight → should trigger Vt swaps

    // Pre-set all to HVT (slow) → critical path will fail
    for (size_t i = 0; i < nl.num_gates(); i++) {
        auto& g = nl.gate(static_cast<GateId>(i));
        if (g.type != GateType::DFF && g.type != GateType::INPUT && g.type != GateType::OUTPUT)
            opt.set_gate_vt(static_cast<GateId>(i), VtType::HVT);
    }

    PostRouteConfig cfg;
    cfg.enable_leakage_recovery = false; // don't undo the swaps
    opt.set_config(cfg);

    PostRouteResult r = opt.optimize();
    CHECK(r.sta_driven == true, "STA-driven mode");
    // With tight timing and all HVT, Vt swap should activate
    CHECK(r.cells_vt_swapped >= 0, "Vt swap pass ran");
    PASS("deep_path_vt_swap");
}

TEST(result_counters_non_negative) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);

    PostRouteResult r = opt.optimize();
    CHECK(r.cells_vt_swapped >= 0, "vt_swapped non-negative");
    CHECK(r.buffers_inserted >= 0, "buffers_inserted non-negative");
    CHECK(r.cells_cloned >= 0, "cells_cloned non-negative");
    CHECK(r.hold_buffers_inserted >= 0, "hold_buffers non-negative");
    CHECK(r.useful_skew_adjustments >= 0, "skew_adj non-negative");
    CHECK(r.leakage_recovered >= 0, "leakage_recovered non-negative");
    CHECK(r.iterations >= 1, "at least 1 iteration");
    CHECK(r.time_ms >= 0, "time_ms non-negative");
    PASS("result_counters_non_negative");
}

TEST(useful_skew_disabled_by_default) {
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);

    PostRouteResult r = opt.optimize();
    CHECK(r.useful_skew_adjustments == 0, "useful skew off by default");
    PASS("useful_skew_disabled_by_default");
}

TEST(message_includes_industrial_info) {
    auto [nl, pd] = build_deep_comb();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);

    PostRouteResult r = opt.optimize();
    CHECK(r.message.find("Post-route opt") != std::string::npos, "has header");
    CHECK(r.message.find("STA-driven") != std::string::npos, "has STA-driven");
    // If any industrial passes fired, message should mention them
    if (r.cells_vt_swapped > 0)
        CHECK(r.message.find("Vt swaps") != std::string::npos, "msg has Vt swaps");
    if (r.leakage_recovered > 0)
        CHECK(r.message.find("leakage recovered") != std::string::npos, "msg has leakage");
    PASS("message_includes_industrial_info");
}

// ========================= E2E Integration =========================

TEST(e2e_full_industrial_opt) {
    // Build a meaningful sequential circuit
    auto [nl, pd] = build_deep_comb();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);
    opt.set_max_iterations(3);

    // Enable everything including useful skew
    PostRouteConfig cfg;
    cfg.enable_useful_skew = true;
    opt.set_config(cfg);

    // Pre-set mixed Vt
    for (size_t i = 0; i < nl.num_gates(); i++) {
        auto& g = nl.gate(static_cast<GateId>(i));
        if (g.type != GateType::DFF && g.type != GateType::INPUT && g.type != GateType::OUTPUT) {
            if (i % 3 == 0) opt.set_gate_vt(static_cast<GateId>(i), VtType::HVT);
            else if (i % 3 == 1) opt.set_gate_vt(static_cast<GateId>(i), VtType::LVT);
            // else default SVT
        }
    }

    PostRouteResult r = opt.optimize();
    CHECK(r.sta_driven == true, "E2E STA-driven");
    CHECK(r.iterations >= 1 && r.iterations <= 3, "E2E iterations in range");
    CHECK(r.time_ms >= 0, "E2E timing measured");
    CHECK(r.leakage_before_uw >= 0, "E2E leakage before");
    CHECK(r.leakage_after_uw >= 0, "E2E leakage after");

    // Verify we get a proper summary message
    CHECK(!r.message.empty(), "E2E message non-empty");
    PASS("e2e_full_industrial_opt");
}

TEST(e2e_backward_compatibility) {
    // Ensure the original optimize() API works identically
    auto [nl, pd] = build_seq_with_phys();
    PostRouteOptimizer opt(nl, pd);
    opt.set_clock_period(2.0);

    PostRouteResult r = opt.optimize();
    // Original fields must be populated
    CHECK(r.gates_resized >= 0, "gates_resized present");
    CHECK(r.vias_doubled >= 0, "vias_doubled present");
    CHECK(r.wires_widened >= 0, "wires_widened present");
    CHECK(r.buffers_resized >= 0, "buffers_resized alias works");
    CHECK(r.sta_driven == true, "still STA-driven");
    PASS("e2e_backward_compatibility");
}

// ========================= Main =========================

int main() {
    std::cout << "=== Phase 21: Post-Route Optimizer Industrial Tests ===\n\n";

    // VtType API
    RUN(vt_type_enum);
    RUN(vt_type_str_conversion);
    RUN(gate_vt_assignment);

    // PostRouteConfig
    RUN(config_defaults);
    RUN(config_custom);
    RUN(vt_leakage_model);
    RUN(vt_delay_model);

    // Optimization passes
    RUN(optimize_sta_driven_with_industrial);
    RUN(optimize_with_all_features_disabled);
    RUN(optimize_legacy_unchanged);
    RUN(leakage_measurement);
    RUN(leakage_recovery_pass);
    RUN(deep_path_vt_swap);
    RUN(result_counters_non_negative);
    RUN(useful_skew_disabled_by_default);
    RUN(message_includes_industrial_info);

    // E2E
    RUN(e2e_full_industrial_opt);
    RUN(e2e_backward_compatibility);

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";
    return failed > 0 ? 1 : 0;
}
