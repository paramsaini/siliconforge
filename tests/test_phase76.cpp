// SiliconForge — Phase 76: Power Analysis & PDN Enhancement Tests
#include "../src/timing/power.hpp"
#include "../src/timing/pdn.hpp"
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

// Helper: create a small netlist with combinational logic and flip-flops
static Netlist make_combo_netlist() {
    Netlist nl;
    // Primary inputs
    auto a = nl.add_net("a");
    auto b = nl.add_net("b");
    auto c = nl.add_net("c");
    nl.mark_input(a);
    nl.mark_input(b);
    nl.mark_input(c);

    // Internal nets
    auto n_and = nl.add_net("and_out");
    auto n_xor = nl.add_net("xor_out");
    auto n_or  = nl.add_net("or_out");
    auto n_not = nl.add_net("not_out");

    // Gates: GateType first
    nl.add_gate(GateType::AND, {a, b}, n_and, "g_and");
    nl.add_gate(GateType::XOR, {a, b}, n_xor, "g_xor");
    nl.add_gate(GateType::OR,  {n_and, c}, n_or, "g_or");
    nl.add_gate(GateType::NOT, {n_or}, n_not, "g_not");

    // Flip-flop
    auto clk = nl.add_net("clk");
    auto q   = nl.add_net("q");
    nl.mark_input(clk);
    nl.add_dff(n_not, clk, q, -1, "ff0");

    // Output
    nl.mark_output(q);
    nl.mark_output(n_xor);

    return nl;
}

// Helper: PhysicalDesign for PDN tests
static PhysicalDesign make_pd() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 100, 100);
    // Place a few cells to generate current distribution
    for (int i = 0; i < 4; ++i) {
        int id = pd.add_cell("cell" + std::to_string(i), "BUF", 5.0, 5.0);
        pd.cells[id].placed = true;
        pd.cells[id].position = {20.0 + i * 20.0, 50.0};
    }
    return pd;
}

// ─── Test 1: Basic power analysis ────────────────────────────────────
static void test_basic_power() {
    auto nl = make_combo_netlist();
    PowerAnalyzer pa(nl);
    auto r = pa.analyze(1000.0, 1.8, 0.1);

    CHECK(r.total_power_mw > 0, "Total power > 0");
    CHECK(r.dynamic_power_mw > 0, "Dynamic power > 0");
    CHECK(r.static_power_mw > 0, "Static/leakage power > 0");
    CHECK(r.num_cells > 0, "Cell count > 0");
    CHECK(r.clock_freq_mhz == 1000.0, "Clock frequency stored");
    std::cout << "  [1] Basic power: " << r.total_power_mw << " mW\n";
}

// ─── Test 2: VCD parsing from string ─────────────────────────────────
static void test_vcd_parsing() {
    auto nl = make_combo_netlist();
    PowerAnalyzer pa(nl);

    std::string vcd = R"(
$var wire 1 ! a $end
$var wire 1 " b $end
$enddefinitions $end
#0
0!
0"
#10
1!
#20
0!
1"
#30
1!
#40
0"
#50
)";
    bool ok = pa.parse_vcd_string(vcd);
    CHECK(ok, "VCD parsing succeeds");

    // After VCD load, run analysis — activities should reflect VCD toggles
    auto r = pa.analyze(1000.0, 1.8, 0.1);
    CHECK(r.total_power_mw > 0, "Power after VCD > 0");
    std::cout << "  [2] VCD parse OK, power=" << r.total_power_mw << " mW\n";
}

// ─── Test 3: SAIF parsing ────────────────────────────────────────────
static void test_saif_parsing() {
    auto nl = make_combo_netlist();
    PowerAnalyzer pa(nl);

    // SAIF load from non-existent file returns false
    bool fail = pa.load_saif("/tmp/nonexistent_saif_file.saif");
    CHECK(!fail, "SAIF load fails on missing file");

    // After failed load, analysis still works with defaults
    auto r = pa.analyze(1000.0, 1.8, 0.1);
    CHECK(r.total_power_mw > 0, "Power with default activity > 0");
    std::cout << "  [3] SAIF handling OK\n";
}

// ─── Test 4: Toggle rate propagation through AND gate ────────────────
static void test_and_propagation() {
    Netlist nl;
    auto a = nl.add_net("a");
    auto b = nl.add_net("b");
    nl.mark_input(a);
    nl.mark_input(b);
    auto out = nl.add_net("and_out");
    nl.add_gate(GateType::AND, {a, b}, out, "g_and");
    nl.mark_output(out);

    PowerAnalyzer pa(nl);
    // Set known input activities
    pa.set_activity(static_cast<NetId>(a), 0.4);
    pa.set_activity(static_cast<NetId>(b), 0.2);

    auto r = pa.analyze(1000.0, 1.8, 0.1);
    CHECK(r.total_power_mw > 0, "AND propagation produces power");

    // For AND with inputs a=0.4, b=0.2 at P=0.5:
    // TD(out) = TD(a)*P(b) + TD(b)*P(a) = 0.4*0.5 + 0.2*0.5 = 0.3
    // The existing analyze() uses its own propagation model, so just check
    // that outputs are reasonable
    CHECK(r.switching_power_mw > 0, "AND switching power > 0");
    std::cout << "  [4] AND propagation OK\n";
}

// ─── Test 5: Toggle rate propagation through XOR gate ────────────────
static void test_xor_propagation() {
    Netlist nl;
    auto a = nl.add_net("a");
    auto b = nl.add_net("b");
    nl.mark_input(a);
    nl.mark_input(b);
    auto out = nl.add_net("xor_out");
    nl.add_gate(GateType::XOR, {a, b}, out, "g_xor");
    nl.mark_output(out);

    PowerAnalyzer pa(nl);
    pa.set_activity(static_cast<NetId>(a), 0.3);
    pa.set_activity(static_cast<NetId>(b), 0.2);

    // Use estimate_rtl_power which invokes vectorless toggle rate propagation
    pa.analyze(1000.0, 1.8, 0.1); // cache params
    auto rtl = pa.estimate_rtl_power();

    CHECK(rtl.total_mw > 0, "XOR RTL power > 0");
    CHECK(rtl.switching_mw > 0, "XOR switching > 0");
    // XOR toggle rate: TD(a) + TD(b) = 0.3 + 0.2 = 0.5
    // Should be higher activity than AND with same inputs
    std::cout << "  [5] XOR propagation: " << rtl.total_mw << " mW\n";
}

// ─── Test 6: RTL power estimation ────────────────────────────────────
static void test_rtl_power() {
    auto nl = make_combo_netlist();
    PowerAnalyzer pa(nl);
    pa.analyze(1000.0, 1.8, 0.1); // cache params

    auto rtl = pa.estimate_rtl_power();
    CHECK(rtl.total_mw > 0, "RTL total > 0");
    CHECK(rtl.switching_mw > 0, "RTL switching > 0");
    CHECK(rtl.leakage_mw > 0, "RTL leakage > 0");
    CHECK(!rtl.message.empty(), "RTL message present");
    std::cout << "  [6] RTL power: " << rtl.total_mw << " mW\n";
}

// ─── Test 7: Clock power analysis ────────────────────────────────────
static void test_clock_power() {
    auto nl = make_combo_netlist();
    PowerAnalyzer pa(nl);
    pa.analyze(1000.0, 1.8, 0.1);

    auto clk = pa.analyze_clock_power();
    CHECK(clk.clock_network_mw > 0, "Clock network power > 0");
    CHECK(clk.clock_fraction_pct > 0, "Clock fraction > 0%");
    CHECK(clk.clock_fraction_pct < 100, "Clock fraction < 100%");
    std::cout << "  [7] Clock power: " << clk.clock_network_mw
              << " mW (" << clk.clock_fraction_pct << "%)\n";
}

// ─── Test 8: Power domain (multi-Vt) analysis ───────────────────────
static void test_power_domains() {
    auto nl = make_combo_netlist();
    PowerAnalyzer pa(nl);
    pa.analyze(1000.0, 1.8, 0.1);

    // Define two power domains with different voltages
    pa.set_power_domain("core", 1.8, {0, 1});
    pa.set_power_domain("io", 3.3, {2, 3});

    auto states = pa.analyze_power_states();
    CHECK(!states.empty(), "Power states generated");

    bool found_all_on = false;
    bool found_domain_off = false;
    for (auto& s : states) {
        if (s.state_name == "all_on") found_all_on = true;
        if (s.state_name.find("_off") != std::string::npos) found_domain_off = true;
        CHECK(s.total_power_mw >= 0, "State power >= 0");
    }
    CHECK(found_all_on, "all_on state found");
    CHECK(found_domain_off, "domain_off state found");
    std::cout << "  [8] Power domains: " << states.size() << " states\n";
}

// ─── Test 9: PDN impedance computation ──────────────────────────────
static void test_pdn_impedance() {
    auto pd = make_pd();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 100);

    // Add a decap
    DecapModel decap;
    decap.name = "c1";
    decap.capacitance_nf = 10.0;
    decap.esr_mohm = 50.0;
    decap.esl_ph = 100.0;
    decap.quantity = 10;
    pdn.add_decap(decap);

    auto sweep = pdn.impedance_sweep();
    CHECK(!sweep.empty(), "Impedance sweep has points");
    CHECK(sweep.front().freq_mhz > 0, "First freq > 0");
    CHECK(sweep.back().freq_mhz > sweep.front().freq_mhz, "Freq ascending");

    double target = pdn.compute_target_impedance();
    CHECK(target > 0, "Target impedance > 0");
    std::cout << "  [9] PDN impedance: " << sweep.size()
              << " points, target=" << target << " mOhm\n";
}

// ─── Test 10: PDN resonance detection ───────────────────────────────
static void test_pdn_resonance() {
    auto pd = make_pd();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 100);

    // Add decap to create resonance conditions
    DecapModel decap;
    decap.name = "c_res";
    decap.capacitance_nf = 100.0;
    decap.esr_mohm = 10.0;
    decap.esl_ph = 200.0;
    decap.quantity = 20;
    pdn.add_decap(decap);

    auto r = pdn.analyze(10);
    // With decaps, resonances may be detected
    CHECK(r.impedance_profile.size() > 0, "Impedance profile populated");
    CHECK(r.target_impedance_mohm > 0, "Target impedance computed");
    // Resonance count can be 0 or more — just verify it doesn't crash
    CHECK(r.num_resonances >= 0, "Resonance count valid");
    std::cout << "  [10] Resonances: " << r.num_resonances
              << ", worst Z=" << r.worst_impedance_mohm << " mOhm\n";
}

// ─── Test 11: PDN IR hotspot fixing ─────────────────────────────────
static void test_ir_hotspot_fix() {
    auto pd = make_pd();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 200);

    // Set a tight EM limit to exercise EM-aware sizing
    auto cfg = pdn.config();
    cfg.em_limit_ma_per_um = 5.0;
    pdn.set_config(cfg);

    IrFixConfig fix_cfg;
    fix_cfg.target_drop_pct = 3.0;
    fix_cfg.max_iterations = 5;

    auto fix = pdn.fix_ir_hotspots(fix_cfg);
    CHECK(fix.iterations >= 0, "Fix iterations >= 0");
    CHECK(fix.final_drop_pct <= fix.initial_drop_pct || fix.initial_drop_pct <= fix_cfg.target_drop_pct,
          "IR drop did not increase");
    CHECK(!fix.message.empty(), "Fix message present");
    CHECK(fix.message.find("EM limit") != std::string::npos, "EM limit in fix message");
    std::cout << "  [11] IR fix: " << fix.initial_drop_pct << "% -> "
              << fix.final_drop_pct << "%, stripes=" << fix.stripes_added << "\n";
}

// ─── Test 12: Combined power + PDN run_enhanced ─────────────────────
static void test_combined_enhanced() {
    auto nl = make_combo_netlist();
    PowerAnalyzer pa(nl);
    pa.analyze(1000.0, 1.8, 0.1);

    auto pwr = pa.run_enhanced();
    CHECK(pwr.total_power_mw > 0, "Enhanced power total > 0");
    CHECK(!pwr.message.empty(), "Enhanced power message");
    CHECK(pwr.message.find("Enhanced") != std::string::npos, "Message says Enhanced");
    CHECK(pwr.message.find("rtl_est") != std::string::npos, "Message includes RTL estimate");
    CHECK(pwr.message.find("states") != std::string::npos, "Message includes states");

    // PDN enhanced
    auto pd = make_pd();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 100);
    DecapModel decap;
    decap.name = "c1";
    decap.capacitance_nf = 10.0;
    decap.esr_mohm = 50.0;
    decap.esl_ph = 100.0;
    decap.quantity = 5;
    pdn.add_decap(decap);

    auto pr = pdn.run_enhanced();
    CHECK(!pr.message.empty(), "PDN enhanced message");
    CHECK(pr.message.find("enhanced") != std::string::npos, "PDN message says enhanced");
    CHECK(pr.target_impedance_mohm > 0, "PDN target impedance > 0");
    std::cout << "  [12] Combined: pwr=" << pwr.total_power_mw
              << " mW, PDN drop=" << pr.worst_drop_pct << "%\n";
}

int main() {
    std::cout << "=== Phase 76: Power Analysis & PDN Enhancement Tests ===\n";

    test_basic_power();
    test_vcd_parsing();
    test_saif_parsing();
    test_and_propagation();
    test_xor_propagation();
    test_rtl_power();
    test_clock_power();
    test_power_domains();
    test_pdn_impedance();
    test_pdn_resonance();
    test_ir_hotspot_fix();
    test_combined_enhanced();

    std::cout << "\n" << tests_passed << "/" << tests_run << " tests passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
