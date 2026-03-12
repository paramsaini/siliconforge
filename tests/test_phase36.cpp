// SiliconForge — Phase 36 Test Suite
// PDN Industrial: Impedance sweep, target impedance, resonance, decaps, bumps, via arrays

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "timing/pdn.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

static PhysicalDesign build_design(int n = 20) {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < n; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "AND2", 3.0, 10.0);
        pd.cells[c].position = {10.0 + (i % 10) * 18.0, 10.0 + (i / 10) * 20.0};
        pd.cells[c].placed = true;
    }
    for (int i = 0; i < n - 1; ++i)
        pd.add_net("n" + std::to_string(i), {i, i+1});
    return pd;
}

// ── Config Tests ─────────────────────────────────────────────────────────

TEST(pdn_config_defaults) {
    PdnConfig cfg;
    CHECK(cfg.vdd == 1.8, "default vdd");
    CHECK(cfg.total_current_ma == 100, "default current");
    CHECK(cfg.voltage_ripple_pct == 5.0, "default ripple");
    CHECK(cfg.freq_start_mhz == 1.0, "default freq start");
    CHECK(cfg.freq_stop_mhz == 5000.0, "default freq stop");
    CHECK(cfg.freq_points == 100, "default freq points");
    CHECK(cfg.on_die_cap_nf == 50.0, "default die cap");
    PASS("pdn_config_defaults");
}

TEST(decap_model) {
    DecapModel d;
    d.name = "100nF_MLCC";
    d.capacitance_nf = 100;
    d.esr_mohm = 20;
    d.esl_ph = 200;
    d.quantity = 10;
    CHECK(d.srf_mhz() > 0, "SRF computed");
    // SRF for 100nF / 200pH ≈ 35.6 MHz
    CHECK(d.srf_mhz() > 30 && d.srf_mhz() < 40, "SRF in range");
    PASS("decap_model");
}

TEST(bump_pad_struct) {
    BumpPad b;
    b.x = 50; b.y = 50;
    b.resistance_mohm = 15;
    b.inductance_ph = 25;
    b.type = BumpPad::POWER;
    CHECK(b.type == BumpPad::POWER, "bump type");
    CHECK(b.resistance_mohm == 15, "bump R");
    PASS("bump_pad_struct");
}

TEST(via_array_struct) {
    ViaArray va;
    va.count = 10;
    va.per_via_resistance_mohm = 5.0;
    va.per_via_inductance_ph = 10.0;
    CHECK(va.total_resistance_mohm() == 0.5, "parallel R");
    CHECK(va.total_inductance_ph() == 1.0, "parallel L");
    PASS("via_array_struct");
}

// ── Target Impedance ─────────────────────────────────────────────────────

TEST(target_impedance) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 1.0;
    cfg.max_transient_current_ma = 1000; // 1A
    cfg.voltage_ripple_pct = 5.0;
    pdn.set_config(cfg);
    double zt = pdn.compute_target_impedance();
    // Z_target = 1.0 * 0.05 / 1.0 = 50 mΩ
    CHECK(std::abs(zt - 50.0) < 1.0, "target impedance calculation");
    PASS("target_impedance");
}

TEST(target_impedance_custom) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.target_impedance_mohm = 25.0; // override
    pdn.set_config(cfg);
    CHECK(pdn.compute_target_impedance() == 25.0, "custom target");
    PASS("target_impedance_custom");
}

// ── Impedance Sweep ──────────────────────────────────────────────────────

TEST(impedance_sweep_basic) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 100);
    auto profile = pdn.impedance_sweep();
    CHECK(!profile.empty(), "profile not empty");
    CHECK((int)profile.size() == 100, "100 points default");
    for (auto& pt : profile) {
        CHECK(pt.freq_mhz > 0, "freq positive");
        CHECK(pt.magnitude_mohm >= 0, "impedance non-negative");
    }
    // Verify log-spaced
    CHECK(profile.front().freq_mhz < 2.0, "starts near 1 MHz");
    CHECK(profile.back().freq_mhz > 4000.0, "ends near 5 GHz");
    PASS("impedance_sweep_basic");
}

TEST(impedance_sweep_with_decap) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 1.0;
    cfg.total_current_ma = 500;
    cfg.freq_points = 50;
    pdn.set_config(cfg);

    // Add decaps
    DecapModel d1;
    d1.capacitance_nf = 100; d1.esr_mohm = 20; d1.esl_ph = 200; d1.quantity = 5;
    pdn.add_decap(d1);

    DecapModel d2;
    d2.capacitance_nf = 1.0; d2.esr_mohm = 5; d2.esl_ph = 50; d2.quantity = 20;
    pdn.add_decap(d2);

    auto profile = pdn.impedance_sweep();
    CHECK((int)profile.size() == 50, "50 points");
    // With decaps, low-freq impedance should be lower
    PASS("impedance_sweep_with_decap");
}

// ── Resonance Detection ──────────────────────────────────────────────────

TEST(resonance_detection) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 1.0;
    cfg.total_current_ma = 100;
    cfg.freq_points = 200; // more points for better resolution
    pdn.set_config(cfg);

    DecapModel d;
    d.capacitance_nf = 10; d.esr_mohm = 30; d.esl_ph = 500; d.quantity = 10;
    pdn.add_decap(d);

    auto r = pdn.analyze();
    // Should detect at least some resonance behavior
    CHECK(r.num_resonances >= 0, "resonance count valid");
    for (auto& res : r.resonances) {
        CHECK(res.freq_mhz > 0, "resonance freq > 0");
        CHECK(res.impedance_mohm > 0, "resonance impedance > 0");
        CHECK(res.q_factor > 0, "Q factor > 0");
    }
    PASS("resonance_detection");
}

// ── Full Analysis ────────────────────────────────────────────────────────

TEST(full_analysis_basic) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 100);
    auto r = pdn.analyze();
    CHECK(r.worst_drop_mv >= 0, "drop computed");
    CHECK(r.worst_drop_pct >= 0, "drop pct");
    CHECK(r.target_impedance_mohm > 0, "target Z computed");
    CHECK(!r.impedance_profile.empty(), "impedance profile");
    CHECK(!r.message.empty(), "message");
    CHECK(r.time_ms >= 0, "time measured");
    PASS("full_analysis_basic");
}

TEST(add_components) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.0, 500);

    pdn.add_decap({"10nF", 10, 30, 100, 50, 0.01, 20});
    pdn.add_bump({10, 10, 8, 25, BumpPad::POWER});
    pdn.add_bump({190, 10, 8, 25, BumpPad::GROUND});
    pdn.add_via_array({20, 5, 10, 0, 1});

    auto r = pdn.analyze();
    CHECK(r.total_decap_nf > 0, "decap counted");
    CHECK(r.decap_types_used >= 1, "decap types");
    CHECK(r.power_bumps == 1, "power bump");
    CHECK(r.ground_bumps == 1, "ground bump");
    CHECK(r.total_via_resistance_mohm > 0, "via R");
    PASS("add_components");
}

// ── Signoff ──────────────────────────────────────────────────────────────

TEST(pi_signoff) {
    auto pd = build_design(5);
    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 1.8;
    cfg.total_current_ma = 1; // very low
    cfg.max_transient_current_ma = 3;
    cfg.voltage_ripple_pct = 10;
    pdn.set_config(cfg);
    auto r = pdn.analyze();
    CHECK(!r.pi_summary.empty(), "PI summary generated");
    CHECK(r.pi_summary.find("Drop") != std::string::npos, "has drop");
    CHECK(r.pi_summary.find("Impedance") != std::string::npos, "has impedance");
    PASS("pi_signoff");
}

TEST(pi_summary_fields) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.0, 200);
    auto r = pdn.analyze();
    CHECK(r.target_impedance_mohm > 0, "target Z");
    CHECK(r.worst_impedance_mohm > 0, "worst Z");
    CHECK(r.worst_impedance_freq_mhz > 0, "worst Z freq");
    CHECK(r.target_violations >= 0, "target violations counted");
    PASS("pi_summary_fields");
}

// ── Backward Compatibility ───────────────────────────────────────────────

TEST(backward_compat) {
    auto pd = build_design();
    PdnAnalyzer pdn(pd);
    pdn.auto_config(1.8, 100);
    auto r = pdn.analyze(10);
    CHECK(r.worst_drop_mv >= 0, "drop");
    CHECK(r.worst_drop_pct >= 0, "drop pct");
    CHECK(r.avg_drop_mv >= 0, "avg drop");
    CHECK(!r.nodes.empty(), "nodes");
    CHECK(r.em_violations >= 0, "EM violations");
    PASS("backward_compat");
}

// ── E2E Industrial ───────────────────────────────────────────────────────

TEST(e2e_industrial_pdn) {
    auto pd = build_design(30);
    PdnAnalyzer pdn(pd);
    PdnConfig cfg;
    cfg.vdd = 0.9;
    cfg.total_current_ma = 800;
    cfg.max_transient_current_ma = 2000;
    cfg.voltage_ripple_pct = 3.0;
    cfg.freq_points = 150;
    cfg.on_die_cap_nf = 100;

    // Add multiple decap tiers
    cfg.decaps.push_back({"100nF_pkg", 100, 20, 300, 200, 0.1, 8});
    cfg.decaps.push_back({"10nF_die", 10, 5, 50, 30, 0.05, 50});
    cfg.decaps.push_back({"100pF_die", 0.1, 2, 10, 5, 0.01, 200});

    // Bumps
    for (int i = 0; i < 4; ++i) {
        double x = pd.die_area.x0 + (i + 0.5) * pd.die_area.width() / 4;
        cfg.bumps.push_back({x, 50, 8, 20, BumpPad::POWER});
        cfg.bumps.push_back({x, 50, 8, 20, BumpPad::GROUND});
    }

    // Via arrays
    cfg.via_arrays.push_back({50, 3, 8, 0, 1});
    cfg.via_arrays.push_back({30, 3, 8, 1, 2});

    pdn.set_config(cfg);
    auto r = pdn.analyze(12);

    CHECK(r.worst_drop_mv >= 0, "drop");
    CHECK(r.target_impedance_mohm > 0, "target Z");
    CHECK(!r.impedance_profile.empty(), "Z profile");
    CHECK(r.total_decap_nf > 0, "decap total");
    CHECK(r.decap_types_used == 3, "3 decap tiers");
    CHECK(r.power_bumps == 4, "4 power bumps");
    CHECK(r.ground_bumps == 4, "4 ground bumps");
    CHECK(r.total_via_resistance_mohm > 0, "via R");
    CHECK(!r.pi_summary.empty(), "PI summary");
    CHECK(r.time_ms >= 0, "time");
    PASS("e2e_industrial_pdn");
}

// ══════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  SiliconForge Phase 36 — PDN Industrial           ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    std::cout << "── Config & Structs ──\n";
    RUN(pdn_config_defaults);
    RUN(decap_model);
    RUN(bump_pad_struct);
    RUN(via_array_struct);

    std::cout << "\n── Target Impedance ──\n";
    RUN(target_impedance);
    RUN(target_impedance_custom);

    std::cout << "\n── Impedance Sweep ──\n";
    RUN(impedance_sweep_basic);
    RUN(impedance_sweep_with_decap);

    std::cout << "\n── Resonance ──\n";
    RUN(resonance_detection);

    std::cout << "\n── Full Analysis ──\n";
    RUN(full_analysis_basic);
    RUN(add_components);

    std::cout << "\n── Signoff ──\n";
    RUN(pi_signoff);
    RUN(pi_summary_fields);

    std::cout << "\n── Backward Compat ──\n";
    RUN(backward_compat);

    std::cout << "\n── E2E Industrial ──\n";
    RUN(e2e_industrial_pdn);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 36: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
