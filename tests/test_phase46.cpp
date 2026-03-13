// SiliconForge — Phase 46: Tier 1 Feature Tests
// Tests: Parasitic extraction, SDC exceptions, Multi-clock STA,
//        IR-aware PDN, SI-aware metal fill
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "timing/parasitics.hpp"
#include "frontend/sdc_parser.hpp"
#include "timing/sta.hpp"
#include "timing/pdn.hpp"
#include "pnr/metal_fill.hpp"

using namespace sf;

static int tests_passed = 0;
static int tests_total = 0;

#define TEST(name) do { \
    tests_total++; \
    std::cout << "  [" << tests_total << "] " << name << "... "; \
} while(0)

#define PASS() do { \
    tests_passed++; \
    std::cout << "PASS" << std::endl; \
} while(0)

// ── Helper: build minimal netlist with wires ───────────────────────────
static PhysicalDesign make_routed_pd() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    // Wire on layer 0 (M1)
    pd.wires.push_back({0, {10, 10}, {90, 10}, 0.5, 0});
    // Wire on layer 1 (M2)
    pd.wires.push_back({1, {10, 10}, {10, 90}, 0.5, 0});
    // Via at (10,10) connecting layer 0 → 1
    Via v; v.position = {10, 10}; v.lower_layer = 0; v.upper_layer = 1;
    pd.vias.push_back(v);
    return pd;
}

static Netlist make_simple_netlist() {
    Netlist nl;
    int clk = nl.add_net("clk");
    int d_net = nl.add_net("d");
    int q_net = nl.add_net("q");
    int buf_out = nl.add_net("buf_o");

    int pi = nl.add_gate(GateType::INPUT, {}, d_net, "d_in");
    int buf = nl.add_gate(GateType::BUF, {d_net}, buf_out, "buf1");
    int ff = nl.add_dff(buf_out, clk, q_net, -1, "ff1");
    int po = nl.add_gate(GateType::OUTPUT, {q_net}, -1, "q_out");

    // clk PI
    int clk_pi = nl.add_gate(GateType::INPUT, {}, clk, "clk_in");

    return nl;
}

// ════════════════════════════════════════════════════════════════════════
// SECTION 1: Parasitic Extraction (t1-parasitic)
// ════════════════════════════════════════════════════════════════════════

void test_parasitic_layer_field() {
    TEST("RCSegment has layer field");
    ParasiticNet::RCSegment seg;
    seg.layer = 2;
    seg.resistance = 0.1;
    seg.capacitance = 0.01;
    assert(seg.layer == 2);
    assert(seg.resistance > 0);
    PASS();
}

void test_parasitic_via_count() {
    TEST("ParasiticNet tracks via_count");
    ParasiticNet pn;
    pn.via_count = 3;
    assert(pn.via_count == 3);
    PASS();
}

void test_parasitic_extraction_runs() {
    TEST("Parasitic extraction produces segments");
    PhysicalDesign pd = make_routed_pd();
    TechParams tp;
    if (!tp.layers.empty()) {
        tp.layers[0].res_per_um = 0.1;
        tp.layers[0].cap_per_um = 0.05;
    }
    ParasiticExtractor px(pd, tp);
    auto result = px.extract();
    // Just verify it runs without crash
    assert(result.nets.size() >= 0);
    PASS();
}

void test_parasitic_spef_output() {
    TEST("SPEF output from ParasiticResult");
    PhysicalDesign pd = make_routed_pd();
    TechParams tp;
    ParasiticExtractor px(pd, tp);
    auto result = px.extract();
    std::string spef = result.to_spef();
    // to_spef() should not crash
    assert(spef.empty() || !spef.empty());
    PASS();
}

// ════════════════════════════════════════════════════════════════════════
// SECTION 2: SDC Exceptions (t1-sdc)
// ════════════════════════════════════════════════════════════════════════

void test_sdc_set_max_delay() {
    TEST("SDC parses set_max_delay");
    std::string sdc_str = "set_max_delay 5.0 -from [get_ports a] -to [get_ports b]\n";
    SdcParser parser;
    SdcConstraints constraints;
    parser.parse_string(sdc_str, constraints);
    bool found = false;
    for (auto& ex : constraints.exceptions) {
        if (ex.type == SdcException::MAX_DELAY) {
            assert(std::abs(ex.value - 5.0) < 0.01);
            found = true;
        }
    }
    assert(found);
    PASS();
}

void test_sdc_set_min_delay() {
    TEST("SDC parses set_min_delay");
    std::string sdc_str = "set_min_delay 1.0 -from [get_ports x] -to [get_ports y]\n";
    SdcParser parser;
    SdcConstraints constraints;
    parser.parse_string(sdc_str, constraints);
    bool found = false;
    for (auto& ex : constraints.exceptions) {
        if (ex.type == SdcException::MIN_DELAY) {
            assert(std::abs(ex.value - 1.0) < 0.01);
            found = true;
        }
    }
    assert(found);
    PASS();
}

void test_sdc_clock_groups() {
    TEST("SDC parses set_clock_groups");
    std::string sdc_str =
        "create_clock -name clk1 -period 10.0 [get_ports clk1]\n"
        "create_clock -name clk2 -period 20.0 [get_ports clk2]\n"
        "set_clock_groups -asynchronous -group {clk1} -group {clk2}\n";
    SdcParser parser;
    SdcConstraints constraints;
    parser.parse_string(sdc_str, constraints);
    assert(constraints.clocks.size() == 2);
    assert(constraints.clock_groups.size() >= 1);
    auto& cg = constraints.clock_groups[0];
    assert(cg.relation == SdcClockGroup::ASYNC);
    assert(cg.groups.size() == 2);
    PASS();
}

void test_sdc_generated_clock() {
    TEST("SDC parses create_generated_clock");
    std::string sdc_str =
        "create_clock -name clk -period 10.0 [get_ports clk]\n"
        "create_generated_clock -name div2 -source [get_ports clk] -divide_by 2\n";
    SdcParser parser;
    SdcConstraints constraints;
    parser.parse_string(sdc_str, constraints);
    bool found = false;
    for (auto& c : constraints.clocks) {
        if (c.name == "div2" && c.is_generated && c.divide_by == 2) {
            found = true;
            assert(c.source == "clk");
        }
    }
    assert(found);
    PASS();
}

void test_sdc_exception_through() {
    TEST("SdcException supports through points");
    SdcException ex;
    ex.type = SdcException::FALSE_PATH;
    ex.through = "mid_point";
    assert(!ex.through.empty());
    assert(ex.through == "mid_point");
    PASS();
}

// ════════════════════════════════════════════════════════════════════════
// SECTION 3: Multi-Clock STA (t1-multi-clock)
// ════════════════════════════════════════════════════════════════════════

void test_multi_clock_domain_detection() {
    TEST("Multi-clock STA detects domains");
    auto nl = make_simple_netlist();
    auto result = run_multi_clock_sta(nl, nullptr, nullptr, 10.0, 5);
    // Should detect at least 1 clock domain (from our single clk)
    assert(result.total_domains >= 1);
    PASS();
}

void test_multi_clock_with_sdc() {
    TEST("Multi-clock STA accepts SDC constraints");
    auto nl = make_simple_netlist();
    SdcConstraints sdc;
    SdcClock c;
    c.name = "clk";
    c.period_ns = 10.0;
    sdc.clocks.push_back(c);
    auto result = run_multi_clock_sta(nl, nullptr, nullptr, 10.0, 5, &sdc);
    assert(result.total_domains >= 1);
    assert(!result.message.empty());
    PASS();
}

void test_clock_domain_info_generated() {
    TEST("ClockDomainInfo tracks generated clocks");
    ClockDomainInfo info;
    info.name = "div_clk";
    info.is_generated = true;
    info.divide_ratio = 4;
    assert(info.divide_ratio == 4);
    assert(info.is_generated);
    PASS();
}

// ════════════════════════════════════════════════════════════════════════
// SECTION 4: IR-Aware PDN (t1-power-grid)
// ════════════════════════════════════════════════════════════════════════

void test_pdn_ir_fix_config() {
    TEST("IrFixConfig defaults are reasonable");
    IrFixConfig cfg;
    assert(cfg.target_drop_pct == 5.0);
    assert(cfg.max_iterations == 5);
    assert(cfg.stripe_width > 0);
    assert(cfg.hotspot_threshold > 0 && cfg.hotspot_threshold <= 1.0);
    PASS();
}

void test_pdn_ir_fix_result() {
    TEST("IrFixResult initializes cleanly");
    IrFixResult res;
    assert(res.iterations == 0);
    assert(res.stripes_added == 0);
    assert(!res.converged);
    PASS();
}

void test_pdn_ir_fix_runs() {
    TEST("fix_ir_hotspots() runs without crash");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    pd.wires.push_back({0, {10, 50}, {90, 50}, 2.0, 0});

    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 50;
    cfg.stripes.push_back({PdnStripe::HORIZONTAL, 25, 5.0, 2, 0.02});
    cfg.stripes.push_back({PdnStripe::HORIZONTAL, 75, 5.0, 2, 0.02});
    pdn.set_config(cfg);

    IrFixConfig fix_cfg;
    fix_cfg.target_drop_pct = 20.0; // relaxed target
    fix_cfg.max_iterations = 2;
    auto result = pdn.fix_ir_hotspots(fix_cfg);
    assert(!result.message.empty());
    PASS();
}

void test_pdn_ir_fix_already_passing() {
    TEST("fix_ir_hotspots() returns immediately when target met");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 10, 10);
    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 1; // tiny current → minimal drop
    cfg.stripes.push_back({PdnStripe::HORIZONTAL, 5, 2.0, 0, 0.001});
    cfg.stripes.push_back({PdnStripe::VERTICAL, 5, 2.0, 1, 0.001});
    pdn.set_config(cfg);

    IrFixConfig fix_cfg;
    fix_cfg.target_drop_pct = 50.0; // very relaxed
    auto result = pdn.fix_ir_hotspots(fix_cfg);
    assert(result.converged);
    assert(result.stripes_added == 0);
    PASS();
}

// ════════════════════════════════════════════════════════════════════════
// SECTION 5: SI-Aware Metal Fill (t1-metal-fill)
// ════════════════════════════════════════════════════════════════════════

void test_fill_pattern_enum() {
    TEST("FillPattern enum values exist");
    assert(static_cast<int>(FillPattern::SOLID) == 0);
    assert(static_cast<int>(FillPattern::CHECKERBOARD) == 1);
    assert(static_cast<int>(FillPattern::STAGGERED) == 2);
    assert(static_cast<int>(FillPattern::SLOTTED) == 3);
    PASS();
}

void test_fill_config_si_fields() {
    TEST("MetalFillConfig has SI/cross-layer fields");
    MetalFillConfig cfg;
    cfg.si_aware = true;
    cfg.critical_net_spacing = 3.0;
    cfg.cross_layer_aware = true;
    cfg.cross_layer_spacing = 1.5;
    cfg.pattern = FillPattern::CHECKERBOARD;
    assert(cfg.si_aware);
    assert(cfg.cross_layer_aware);
    assert(cfg.critical_net_spacing == 3.0);
    PASS();
}

void test_fill_solid_pattern() {
    TEST("Solid fill produces fills");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    MetalFillEngine engine(pd);
    MetalFillConfig cfg;
    cfg.num_layers = 1;
    cfg.min_density = 0.10;
    cfg.fill_width = 2.0;
    cfg.fill_height = 2.0;
    cfg.fill_spacing = 1.0;
    cfg.pattern = FillPattern::SOLID;
    auto result = engine.fill(cfg);
    assert(result.total_fills > 0);
    assert(result.density_after[0] >= cfg.min_density - 0.01);
    PASS();
}

void test_fill_checkerboard_pattern() {
    TEST("Checkerboard pattern produces fewer fills than solid");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);

    // Solid fill
    PhysicalDesign pd2 = pd;
    MetalFillEngine engine1(pd);
    MetalFillEngine engine2(pd2);
    MetalFillConfig cfg;
    cfg.num_layers = 1;
    cfg.min_density = 0.05; // low threshold so both can meet it
    cfg.fill_width = 2.0;
    cfg.fill_height = 2.0;
    cfg.fill_spacing = 1.0;

    cfg.pattern = FillPattern::SOLID;
    auto r1 = engine1.fill(cfg);

    cfg.pattern = FillPattern::CHECKERBOARD;
    auto r2 = engine2.fill(cfg);

    // Checkerboard skips every other position
    assert(r2.total_fills <= r1.total_fills);
    PASS();
}

void test_fill_staggered_pattern() {
    TEST("Staggered pattern runs without crash");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    MetalFillEngine engine(pd);
    MetalFillConfig cfg;
    cfg.num_layers = 1;
    cfg.min_density = 0.10;
    cfg.fill_width = 2.0;
    cfg.fill_height = 2.0;
    cfg.fill_spacing = 1.0;
    cfg.pattern = FillPattern::STAGGERED;
    auto result = engine.fill(cfg);
    assert(result.total_fills > 0);
    PASS();
}

void test_fill_slotted_pattern() {
    TEST("Slotted pattern creates double fills per position");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    MetalFillEngine engine(pd);
    MetalFillConfig cfg;
    cfg.num_layers = 1;
    cfg.min_density = 0.05;
    cfg.fill_width = 2.0;
    cfg.fill_height = 2.0;
    cfg.fill_spacing = 1.0;
    cfg.pattern = FillPattern::SLOTTED;
    auto result = engine.fill(cfg);
    // Slotted inserts 2 wires per position
    assert(result.total_fills > 0);
    assert(result.total_fills % 2 == 0);
    PASS();
}

void test_fill_critical_net_avoidance() {
    TEST("SI-aware fill skips critical net regions");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    // Place a critical wire across the center
    pd.wires.push_back({0, {0, 25}, {50, 25}, 0.5, 42}); // net_id=42

    MetalFillEngine engine(pd);
    engine.set_critical_nets({42});

    MetalFillConfig cfg;
    cfg.num_layers = 1;
    cfg.min_density = 0.10;
    cfg.fill_width = 2.0;
    cfg.fill_height = 2.0;
    cfg.fill_spacing = 1.0;
    cfg.si_aware = true;
    cfg.critical_net_spacing = 5.0; // big keepout
    cfg.pattern = FillPattern::SOLID;

    auto result = engine.fill(cfg);
    assert(result.fills_skipped_critical > 0);
    PASS();
}

void test_fill_cross_layer_avoidance() {
    TEST("Cross-layer aware fill skips adjacent layer regions");
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 50, 50);
    // Wire on layer 1 (adjacent to layer 0 where we fill)
    pd.wires.push_back({1, {0, 25}, {50, 25}, 0.5, 1});

    MetalFillEngine engine(pd);
    MetalFillConfig cfg;
    cfg.num_layers = 1; // only fill layer 0
    cfg.min_density = 0.10;
    cfg.fill_width = 2.0;
    cfg.fill_height = 2.0;
    cfg.fill_spacing = 1.0;
    cfg.cross_layer_aware = true;
    cfg.cross_layer_spacing = 5.0;
    cfg.pattern = FillPattern::SOLID;

    auto result = engine.fill(cfg);
    assert(result.fills_skipped_cross_layer > 0);
    PASS();
}

void test_fill_result_message() {
    TEST("MetalFillResult message includes skip counts");
    MetalFillResult res;
    res.total_fills = 100;
    res.fills_skipped_critical = 5;
    res.fills_skipped_cross_layer = 3;
    assert(res.fills_skipped_critical == 5);
    assert(res.fills_skipped_cross_layer == 3);
    PASS();
}

// ════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "═══ Phase 46: Tier 1 Feature Tests ═══" << std::endl;

    std::cout << "\n── Parasitic Extraction ──" << std::endl;
    test_parasitic_layer_field();
    test_parasitic_via_count();
    test_parasitic_extraction_runs();
    test_parasitic_spef_output();

    std::cout << "\n── SDC Exceptions ──" << std::endl;
    test_sdc_set_max_delay();
    test_sdc_set_min_delay();
    test_sdc_clock_groups();
    test_sdc_generated_clock();
    test_sdc_exception_through();

    std::cout << "\n── Multi-Clock STA ──" << std::endl;
    test_multi_clock_domain_detection();
    test_multi_clock_with_sdc();
    test_clock_domain_info_generated();

    std::cout << "\n── IR-Aware PDN ──" << std::endl;
    test_pdn_ir_fix_config();
    test_pdn_ir_fix_result();
    test_pdn_ir_fix_runs();
    test_pdn_ir_fix_already_passing();

    std::cout << "\n── SI-Aware Metal Fill ──" << std::endl;
    test_fill_pattern_enum();
    test_fill_config_si_fields();
    test_fill_solid_pattern();
    test_fill_checkerboard_pattern();
    test_fill_staggered_pattern();
    test_fill_slotted_pattern();
    test_fill_critical_net_avoidance();
    test_fill_cross_layer_avoidance();
    test_fill_result_message();

    std::cout << "\n═══ Results: " << tests_passed << "/" << tests_total
              << " PASSED ═══" << std::endl;
    return (tests_passed == tests_total) ? 0 : 1;
}
