// SiliconForge — Phase 71: Noise & Signal Integrity Enhancement Tests
#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "timing/noise.hpp"
#include "timing/signal_integrity.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "  FAIL: " << msg << " [line " << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

#define TEST(name) static void test_##name()
#define RUN(name) do { std::cout << "  Running: " #name "\n"; test_##name(); } while(0)

// ── Helpers ──────────────────────────────────────────────────────────────

static PhysicalDesign build_noise_design() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < 10; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "AND2", 3.0, 10.0);
        pd.cells[c].position = Point(10.0 + (i % 5) * 36.0,
                                     10.0 + (i / 5) * 40.0);
        pd.cells[c].placed = true;
    }
    for (int i = 0; i < 9; ++i)
        pd.add_net("n" + std::to_string(i), {i, i + 1});
    return pd;
}

static PhysicalDesign build_coupled_design() {
    PhysicalDesign pd;
    pd.die_area = Rect(0, 0, 200, 100);
    pd.row_height = 10.0;
    for (int i = 0; i < 6; ++i) {
        int c = pd.add_cell("c" + std::to_string(i), "INV", 2.0, 10.0);
        pd.cells[c].position = Point(20.0 + i * 30.0, 30.0);
        pd.cells[c].placed = true;
    }
    pd.add_net("n0", {0, 1});
    pd.add_net("n1", {2, 3});
    pd.add_net("n2", {4, 5});

    // Two parallel horizontal wires on same layer, spacing 1.0 um, length 80 um
    WireSegment w1;
    w1.layer = 1; w1.start = Point(10, 20); w1.end = Point(90, 20);
    w1.width = 0.1; w1.net_id = 0;
    WireSegment w2;
    w2.layer = 1; w2.start = Point(10, 21); w2.end = Point(90, 21);
    w2.width = 0.1; w2.net_id = 1;
    pd.wires.push_back(w1);
    pd.wires.push_back(w2);
    return pd;
}

// ── Test 1: PSN noise computation (Ldi/dt) ───────────────────────────────

TEST(psn_noise) {
    auto pd = build_noise_design();
    NoiseConfig cfg;
    cfg.num_switching_outputs = 8;
    cfg.di_dt_ma_per_ns = 20.0;
    cfg.output_rise_time_ns = 0.5;
    NoiseAnalyzer na(pd, cfg);

    auto psn = na.analyze_power_supply_noise(2.0);
    // di_dt_max = 8 * 20 = 160 mA/ns
    // peak_noise = 2.0 nH * 160 mA/ns = 320 mV
    CHECK(std::abs(psn.peak_noise_mv - 320.0) < 0.01, "PSN peak = 320 mV");
    CHECK(std::abs(psn.inductance_nh - 2.0) < 0.01, "PSN inductance stored");
    CHECK(std::abs(psn.ldi_dt_max - 160.0) < 0.01, "PSN ldi_dt_max = 160");
    CHECK(psn.noise_waveform.size() == 50, "waveform has 50 points");
    CHECK(std::abs(psn.noise_waveform.front().second) < 1.0,
          "waveform starts near 0");
}

// ── Test 2: SSO ground bounce calculation ────────────────────────────────

TEST(sso_bounce) {
    auto pd = build_noise_design();
    NoiseConfig cfg;
    cfg.num_switching_outputs = 8;
    cfg.pkg_inductance_nh = 1.0;
    cfg.di_dt_ma_per_ns = 10.0;
    NoiseAnalyzer na(pd, cfg);

    auto sso = na.analyze_sso(100.0);
    // ground_bounce = 8 * 1.0 * 10.0 = 80 mV
    CHECK(std::abs(sso.ground_bounce_mv - 80.0) < 0.01, "SSO bounce = 80 mV");
    // vdd_droop = 80 * 0.7 = 56 mV
    CHECK(std::abs(sso.vdd_droop_mv - 56.0) < 0.01, "SSO droop = 56 mV");
    CHECK(sso.within_budget == true, "SSO within 100 mV budget");
    CHECK(sso.simultaneous_switching == 8, "SSO count = 8");

    // Budget violation when max_bounce is small
    auto sso2 = na.analyze_sso(50.0);
    CHECK(sso2.within_budget == false, "SSO exceeds 50 mV budget");
}

// ── Test 3: Noise margin verification (passing) ─────────────────────────

TEST(noise_margin_pass) {
    auto pd = build_noise_design();
    NoiseConfig cfg;
    cfg.vdd = 1.8;
    cfg.noise_margin_pct = 20.0;   // generous margin = 360 mV
    NoiseAnalyzer na(pd, cfg);

    auto nmr = na.verify_noise_margins();
    CHECK(nmr.total_nets > 0, "nets checked");
    CHECK(nmr.failing_nets == 0, "no failing nets with 20% margin");
}

// ── Test 4: Noise margin verification (failing) ─────────────────────────

TEST(noise_margin_fail) {
    auto pd = build_noise_design();
    NoiseConfig cfg;
    cfg.vdd = 1.8;
    cfg.noise_margin_pct = 0.1;    // tiny margin = 1.8 mV
    NoiseAnalyzer na(pd, cfg);

    auto nmr = na.verify_noise_margins();
    CHECK(nmr.total_nets > 0, "nets checked");
    CHECK(nmr.failing_nets > 0, "failures detected with tiny margin");
    CHECK(nmr.worst_margin < 0, "worst margin is negative");
}

// ── Test 5: Jitter analysis (RJ + DJ) ───────────────────────────────────

TEST(jitter_analysis) {
    auto pd = build_noise_design();
    NoiseConfig cfg;
    cfg.vdd = 1.0;
    cfg.pkg_inductance_nh = 0.5;
    cfg.di_dt_ma_per_ns = 10.0;
    cfg.on_die_decap_nf = 50.0;
    cfg.pkg_decap_nf = 100.0;
    NoiseAnalyzer na(pd, cfg);

    auto jr = na.analyze_jitter(1e9);
    CHECK(jr.rj_rms_ps > 0, "RJ > 0");
    CHECK(jr.dj_pp_ps > 0, "DJ > 0");
    CHECK(jr.tj_at_ber_ps > jr.dj_pp_ps, "TJ > DJ (includes RJ)");
    CHECK(jr.source.size() > 0, "jitter source described");
}

// ── Test 6: Noise run_enhanced() ─────────────────────────────────────────

TEST(noise_run_enhanced) {
    auto pd = build_noise_design();
    NoiseConfig cfg;
    cfg.vdd = 1.8;
    cfg.num_switching_outputs = 16;
    cfg.pkg_inductance_nh = 0.5;
    cfg.di_dt_ma_per_ns = 10.0;
    NoiseAnalyzer na(pd, cfg);

    auto r = na.run_enhanced();
    CHECK(r.message.find("Enhanced") != std::string::npos,
          "enhanced message present");
    CHECK(r.noise_margin_mv > 0, "noise margin computed");
    CHECK(r.ssn.peak_ssn_mv > 0, "SSN in enhanced result");
    CHECK(r.time_ms >= 0, "timing measured");
}

// ── Test 7: Aggressor identification ─────────────────────────────────────

TEST(aggressor_id) {
    auto pd = build_coupled_design();
    SignalIntegrityAnalyzer si(pd, 1.8);

    auto aggressors = si.identify_all_aggressors();
    CHECK(aggressors.size() > 0, "aggressors found");
    for (auto& ag : aggressors) {
        CHECK(ag.victim_net != ag.aggressor_net, "victim != aggressor");
        CHECK(ag.coupling_cap > 0, "positive coupling cap");
        CHECK(ag.parallel_length > 0, "parallel length estimated");
    }
}

// ── Test 8: Functional filtering ─────────────────────────────────────────

TEST(func_filtering) {
    auto pd = build_coupled_design();
    SignalIntegrityAnalyzer si(pd, 1.8);

    auto filters = si.functional_filtering();
    CHECK(filters.size() > 0, "filters produced");
    bool found_non_switching = false;
    for (auto& ff : filters) {
        CHECK(ff.reason.size() > 0, "reason provided");
        if (!ff.can_switch_simultaneously) found_non_switching = true;
    }
    CHECK(found_non_switching, "some aggressor pairs filtered");
}

// ── Test 9: Noise propagation through gates ──────────────────────────────

TEST(noise_propagation) {
    auto pd = build_coupled_design();
    SignalIntegrityAnalyzer si(pd, 1.8);

    auto prop = si.propagate_noise();
    CHECK(prop.size() > 0, "propagation results exist");
    for (auto& np : prop) {
        CHECK(np.noise_amplitude_mv > 0, "positive noise amplitude");
        CHECK(np.attenuated_amplitude_mv <= np.noise_amplitude_mv,
              "attenuation reduces noise");
        CHECK(np.gates_traversed >= 1, "at least 1 gate traversed");
    }
}

// ── Test 10: Glitch energy check ─────────────────────────────────────────

TEST(glitch_energy) {
    auto pd = build_coupled_design();
    SignalIntegrityAnalyzer si(pd, 1.8);

    auto glitches = si.check_glitch_energy();
    CHECK(glitches.size() > 0, "glitch results exist");
    for (auto& g : glitches) {
        CHECK(g.glitch_energy_fj > 0, "positive glitch energy");
        CHECK(g.threshold_fj > 0, "positive threshold");
    }
}

// ── Test 11: SI hold slack impact ────────────────────────────────────────

TEST(si_hold) {
    auto pd = build_coupled_design();
    SignalIntegrityAnalyzer si(pd, 1.8);

    auto holds = si.check_si_hold();
    CHECK(holds.size() > 0, "hold results exist");
    for (auto& h : holds) {
        CHECK(h.original_hold_slack > 0, "positive original slack");
        if (h.needs_fix) {
            CHECK(h.si_hold_slack < 0, "negative SI slack when fix needed");
            CHECK(h.buffer_delay_needed > 0, "buffer delay computed");
        }
    }
}

// ── Test 12: SI run_enhanced() ───────────────────────────────────────────

TEST(si_run_enhanced) {
    auto pd = build_coupled_design();
    SignalIntegrityAnalyzer si(pd, 1.8);

    auto r = si.run_enhanced();
    CHECK(r.message.find("Enhanced") != std::string::npos,
          "enhanced message present");
    CHECK(r.nets_analyzed > 0, "nets analyzed");
    CHECK(r.time_ms >= 0, "timing measured");
}

// ── main ─────────────────────────────────────────────────────────────────

int main() {
    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 71: Noise & Signal Integrity Enhancement\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    std::cout << "── Noise Enhancement ──\n";
    RUN(psn_noise);
    RUN(sso_bounce);
    RUN(noise_margin_pass);
    RUN(noise_margin_fail);
    RUN(jitter_analysis);
    RUN(noise_run_enhanced);

    std::cout << "\n── Signal Integrity Enhancement ──\n";
    RUN(aggressor_id);
    RUN(func_filtering);
    RUN(noise_propagation);
    RUN(glitch_energy);
    RUN(si_hold);
    RUN(si_run_enhanced);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 71: " << tests_passed << "/" << tests_run
              << " checks passed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
