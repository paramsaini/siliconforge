// SiliconForge — Phase 72: EM Enhancement & CDC MTBF Tests
#include "../src/timing/electromigration.hpp"
#include "../src/verify/cdc.hpp"
#include "../src/core/netlist.hpp"
#include "../src/pnr/physical.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

// ────────────────────────────────────────────────────────────────────
// Helpers
// ────────────────────────────────────────────────────────────────────

static PhysicalDesign make_em_design() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 500, 500};

    // Wire 0: short wire (5 μm) on layer 0
    WireSegment w0;
    w0.layer = 0;
    w0.start = {10, 10};
    w0.end   = {15, 10};
    w0.width = 0.14;
    w0.net_id = 0;
    pd.wires.push_back(w0);

    // Wire 1: long wire (200 μm) on layer 0
    WireSegment w1;
    w1.layer = 0;
    w1.start = {10, 20};
    w1.end   = {210, 20};
    w1.width = 0.14;
    w1.net_id = 1;
    pd.wires.push_back(w1);

    // Wire 2: medium wire (50 μm) on layer 1
    WireSegment w2;
    w2.layer = 1;
    w2.start = {20, 30};
    w2.end   = {70, 30};
    w2.width = 0.14;
    w2.net_id = 2;
    pd.wires.push_back(w2);

    // Vias
    Via v0; v0.position = {15, 10}; v0.lower_layer = 0; v0.upper_layer = 1;
    Via v1; v1.position = {70, 30}; v1.lower_layer = 1; v1.upper_layer = 2;
    Via v2; v2.position = {50, 50}; v2.lower_layer = 0; v2.upper_layer = 1;
    pd.vias.push_back(v0);
    pd.vias.push_back(v1);
    pd.vias.push_back(v2);

    return pd;
}

static Netlist make_em_netlist() {
    Netlist nl;
    // Net 0 — small fanout
    NetId n0 = nl.add_net("sig_a");
    NetId n1 = nl.add_net("sig_b");
    NetId n2 = nl.add_net("sig_c");
    nl.mark_input(n0);
    nl.mark_input(n1);
    nl.mark_input(n2);

    // Add a gate with fanout on net 0 so current estimation uses fanout
    NetId out0 = nl.add_net("out0");
    nl.add_gate(GateType::BUF, {n0}, out0);
    return nl;
}

static EmConfig make_em_config() {
    EmConfig cfg;
    cfg.supply_voltage   = 1.0;
    cfg.clock_freq_ghz   = 1.0;
    cfg.activity_factor  = 0.1;
    cfg.temperature_c    = 105.0;
    cfg.activation_energy_ev = 0.7;
    cfg.current_density_exponent = 2.0;
    cfg.jdc_margin       = 0.8;
    cfg.layer_rules      = EmAnalyzer::default_rules_sky130();
    return cfg;
}

// Build a CDC netlist with 2 clock domains and crossing DFFs
static Netlist make_cdc_netlist() {
    Netlist nl;
    NetId clk_a = nl.add_net("clk_a");
    NetId clk_b = nl.add_net("clk_b");
    nl.mark_input(clk_a);
    nl.mark_input(clk_b);

    // Domain A: DFF produces data
    NetId d_a  = nl.add_net("d_a");
    NetId q_a  = nl.add_net("q_a");
    nl.mark_input(d_a);
    nl.add_dff(d_a, clk_a, q_a, -1, "ff_a");

    // Domain B: Two-FF synchronizer chain (q_a → ff_sync1 → ff_sync2)
    NetId q_sync1 = nl.add_net("q_sync1");
    nl.add_dff(q_a, clk_b, q_sync1, -1, "ff_sync1");

    NetId q_sync2 = nl.add_net("q_sync2");
    nl.add_dff(q_sync1, clk_b, q_sync2, -1, "ff_sync2");
    nl.mark_output(q_sync2);

    return nl;
}

// ────────────────────────────────────────────────────────────────────
// EM Tests
// ────────────────────────────────────────────────────────────────────

void test_current_density_extraction() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    EmAnalyzer em(nl, pd, cfg);

    auto densities = em.extract_current_density();
    CHECK(!densities.empty(), "extract_current_density returns results");
    // Each wire should have a positive current density
    for (auto& cd : densities) {
        CHECK(cd.current_density_ma_per_um2 > 0,
              "current density > 0 for wire " + std::to_string(cd.wire_idx));
        CHECK(cd.limit_ma_per_um2 > 0,
              "limit > 0 for wire " + std::to_string(cd.wire_idx));
    }
}

void test_blech_short_wire_immune() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    EmAnalyzer em(nl, pd, cfg);

    auto blech = em.check_blech_effect();
    CHECK(!blech.empty(), "blech results non-empty");

    // Wire 0 is 5 μm — should be EM-immune (Blech length much larger)
    bool found_short = false;
    for (auto& b : blech) {
        if (b.wire_idx == 0) {
            found_short = true;
            CHECK(b.wire_length_um < 6.0, "short wire length ~5 μm");
            CHECK(b.blech_length_um > b.wire_length_um,
                  "Blech length exceeds short wire length");
            CHECK(b.is_immune, "short wire should be Blech-immune");
        }
    }
    CHECK(found_short, "found short wire in Blech results");
}

void test_blech_long_wire_not_immune() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    // High frequency + high activity → large J → small Blech length
    cfg.clock_freq_ghz  = 10.0;
    cfg.activity_factor = 1.0;
    EmAnalyzer em(nl, pd, cfg);

    auto blech = em.check_blech_effect();
    // Wire 1 is 200 μm — with high J the Blech length should be < 200 μm
    bool found_long = false;
    for (auto& b : blech) {
        if (b.wire_idx == 1) {
            found_long = true;
            CHECK(b.wire_length_um > 199.0, "long wire length ~200 μm");
            CHECK(!b.is_immune, "long wire should NOT be Blech-immune at high J");
        }
    }
    CHECK(found_long, "found long wire in Blech results");
}

void test_em_fix_widen_wire() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    // Set limit so ratio is between 1× and 2× (produces WIDEN_WIRE)
    for (auto& r : cfg.layer_rules) {
        if (!r.is_via) r.jdc_limit_ma_per_um = 0.15;
    }
    EmAnalyzer em(nl, pd, cfg);

    auto fixes = em.suggest_em_fixes();
    bool has_widen = false;
    for (auto& f : fixes) {
        if (f.type == EmFix::WIDEN_WIRE) {
            has_widen = true;
            CHECK(f.suggested_width > f.current_width,
                  "WIDEN_WIRE: suggested > current width");
        }
    }
    CHECK(has_widen, "at least one WIDEN_WIRE suggestion");
}

void test_em_fix_split_net() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    // Make limits very low so ratio > 2× → SPLIT_NET
    for (auto& r : cfg.layer_rules) {
        if (!r.is_via) r.jdc_limit_ma_per_um = 1e-6;
    }
    EmAnalyzer em(nl, pd, cfg);

    auto fixes = em.suggest_em_fixes();
    bool has_split = false;
    for (auto& f : fixes) {
        if (f.type == EmFix::SPLIT_NET) has_split = true;
    }
    CHECK(has_split, "at least one SPLIT_NET suggestion for extreme violation");
}

void test_redundant_via_counting() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    EmAnalyzer em(nl, pd, cfg);

    auto vr = em.add_redundant_vias();
    CHECK(vr.total_vias == 3, "total vias == 3");
    CHECK(vr.single_vias_at_risk == 3, "all single-cut vias flagged at risk");
    CHECK(vr.redundant_vias_added == 3, "1 redundant via per location");
}

void test_via_reliability_improvement() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    EmAnalyzer em(nl, pd, cfg);

    auto vr = em.add_redundant_vias();
    // N=2 → improvement = (1 - 1/4) * 100 = 75 %
    CHECK(std::abs(vr.reliability_improvement_pct - 75.0) < 0.1,
          "reliability improvement ~75 % for N=2");
}

void test_run_enhanced_full_flow() {
    auto nl  = make_em_netlist();
    auto pd  = make_em_design();
    auto cfg = make_em_config();
    EmAnalyzer em(nl, pd, cfg);

    auto r = em.run_enhanced();
    CHECK(r.total_nets_checked >= 0, "run_enhanced returns a valid EmResult");
    CHECK(r.summary.find("Enhanced") != std::string::npos,
          "summary contains Enhanced tag");
    CHECK(r.summary.find("Blech-immune") != std::string::npos,
          "summary contains Blech-immune count");
    CHECK(r.summary.find("redundant vias") != std::string::npos,
          "summary mentions redundant vias");
}

// ────────────────────────────────────────────────────────────────────
// CDC MTBF Tests
// ────────────────────────────────────────────────────────────────────

void test_mtbf_basic() {
    auto nl = make_cdc_netlist();
    CdcAnalyzer cdc(nl);

    // 100 MHz src, 200 MHz dst, 2 sync stages
    auto m = cdc.compute_mtbf(100e6, 200e6, 2, 50.0);
    CHECK(m.mtbf_years > 0, "MTBF > 0 for basic case");
    CHECK(m.clock_freq_hz == 200e6, "clock_freq_hz matches dst");
    CHECK(m.sync_stages == 2, "sync_stages == 2");
    CHECK(!m.risk_level.empty(), "risk_level is set");
}

void test_mtbf_different_frequencies() {
    auto nl = make_cdc_netlist();
    CdcAnalyzer cdc(nl);

    // Lower frequencies → longer MTBF
    auto m_low  = cdc.compute_mtbf(10e6, 20e6, 2, 50.0);
    auto m_high = cdc.compute_mtbf(1e9, 2e9, 2, 50.0);
    CHECK(m_low.mtbf_years > m_high.mtbf_years,
          "lower frequency yields higher MTBF");
}

void test_mtbf_resolution_window() {
    auto nl = make_cdc_netlist();
    CdcAnalyzer cdc(nl);

    // Larger metastability window → worse MTBF
    auto m_small = cdc.compute_mtbf(100e6, 200e6, 2, 10.0);
    auto m_large = cdc.compute_mtbf(100e6, 200e6, 2, 200.0);
    CHECK(m_small.mtbf_years >= m_large.mtbf_years,
          "smaller met window yields >= MTBF");
}

void test_cdc_analyze_with_mtbf() {
    auto nl = make_cdc_netlist();
    CdcAnalyzer cdc(nl);
    cdc.set_clock_domain("clk_a", "domA");
    cdc.set_clock_domain("clk_b", "domB");
    cdc.auto_detect_domains();

    auto r = cdc.analyze();
    CHECK(r.domains_found >= 2, "at least 2 clock domains detected");

    // Run MTBF for the crossing
    auto m = cdc.compute_mtbf(100e6, 200e6, 2, 50.0);
    CHECK(m.mtbf_years > 0, "MTBF positive after CDC analyze");
    CHECK(m.risk_level == "SAFE" || m.risk_level == "MARGINAL"
          || m.risk_level == "CRITICAL", "valid risk level");
}

// ────────────────────────────────────────────────────────────────────
int main() {
    // EM tests
    test_current_density_extraction();
    test_blech_short_wire_immune();
    test_blech_long_wire_not_immune();
    test_em_fix_widen_wire();
    test_em_fix_split_net();
    test_redundant_via_counting();
    test_via_reliability_improvement();
    test_run_enhanced_full_flow();

    // CDC MTBF tests
    test_mtbf_basic();
    test_mtbf_different_frequencies();
    test_mtbf_resolution_window();
    test_cdc_analyze_with_mtbf();

    std::cout << tests_passed << "/" << tests_run << " tests passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
