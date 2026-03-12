// SiliconForge — Phase 37 Test Suite
// Signal Integrity Industrial: Miller factor, timing windows, CID, glitch filtering,
// SSN, frequency-domain noise, noise config, backward compat, E2E

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"
#include "timing/signal_integrity.hpp"
#include "timing/noise.hpp"
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

// ── SiConfig Tests ───────────────────────────────────────────────────────

TEST(si_config_defaults) {
    SiConfig cfg;
    CHECK(cfg.vdd == 1.8, "default vdd");
    CHECK(cfg.noise_threshold_pct == 15.0, "default noise threshold");
    CHECK(cfg.coupling_distance_um == 5.0, "default coupling distance");
    CHECK(cfg.miller_factor == 0.75, "default Miller factor");
    CHECK(cfg.glitch_filter_ps == 20.0, "default glitch filter");
    CHECK(cfg.gate_prop_delay_ps == 30.0, "default gate prop delay");
    CHECK(cfg.victim_base_cap_ff == 0.5, "default victim base cap");
    CHECK(cfg.pin_cap_ff == 0.002, "default pin cap");
    PASS("si_config_defaults");
}

TEST(si_config_custom) {
    SiConfig cfg;
    cfg.vdd = 0.9;
    cfg.miller_factor = 0.5;
    cfg.coupling_distance_um = 3.0;
    auto pd = build_design(5);
    SignalIntegrityAnalyzer si(pd, cfg);
    CHECK(si.config().vdd == 0.9, "custom vdd");
    CHECK(si.config().miller_factor == 0.5, "custom miller");
    CHECK(si.config().coupling_distance_um == 3.0, "custom coupling dist");
    PASS("si_config_custom");
}

TEST(si_set_config) {
    auto pd = build_design(5);
    SignalIntegrityAnalyzer si(pd, 1.8);
    CHECK(si.config().vdd == 1.8, "initial vdd");
    SiConfig cfg;
    cfg.vdd = 1.0;
    cfg.miller_factor = 0.6;
    si.set_config(cfg);
    CHECK(si.config().vdd == 1.0, "updated vdd");
    CHECK(si.config().miller_factor == 0.6, "updated miller");
    PASS("si_set_config");
}

// ── Miller Factor Tests ──────────────────────────────────────────────────

TEST(miller_factor_opposite) {
    // Opposite switching: Cc_eff = Cc × (1 + Miller)
    double cc = 1.0;
    double miller = 0.75;
    double eff = SignalIntegrityAnalyzer::effective_coupling(cc, miller, true);
    CHECK(std::abs(eff - 1.75) < 0.001, "opposite: 1.0 × 1.75 = 1.75");
    PASS("miller_factor_opposite");
}

TEST(miller_factor_same_dir) {
    // Same direction: Cc_eff = Cc × (1 - Miller)
    double cc = 1.0;
    double miller = 0.75;
    double eff = SignalIntegrityAnalyzer::effective_coupling(cc, miller, false);
    CHECK(std::abs(eff - 0.25) < 0.001, "same dir: 1.0 × 0.25 = 0.25");
    PASS("miller_factor_same_dir");
}

TEST(miller_factor_zero) {
    double cc = 2.0;
    double eff_opp = SignalIntegrityAnalyzer::effective_coupling(cc, 0.0, true);
    double eff_same = SignalIntegrityAnalyzer::effective_coupling(cc, 0.0, false);
    CHECK(std::abs(eff_opp - 2.0) < 0.001, "miller=0 opposite");
    CHECK(std::abs(eff_same - 2.0) < 0.001, "miller=0 same");
    PASS("miller_factor_zero");
}

// ── Timing Window Tests ─────────────────────────────────────────────────

TEST(timing_window_full_overlap) {
    double ol = SignalIntegrityAnalyzer::timing_overlap(0, 100, 0, 100);
    CHECK(std::abs(ol - 1.0) < 0.001, "full overlap = 1.0");
    PASS("timing_window_full_overlap");
}

TEST(timing_window_no_overlap) {
    double ol = SignalIntegrityAnalyzer::timing_overlap(0, 50, 100, 200);
    CHECK(ol == 0.0, "no overlap = 0.0");
    PASS("timing_window_no_overlap");
}

TEST(timing_window_partial) {
    // aggressor [30, 80], victim [50, 100] → overlap [50,80] = 30, victim window = 50
    double ol = SignalIntegrityAnalyzer::timing_overlap(30, 80, 50, 100);
    CHECK(std::abs(ol - 0.6) < 0.001, "partial overlap = 0.6");
    PASS("timing_window_partial");
}

// ── CID Tests ────────────────────────────────────────────────────────────

TEST(cid_basic) {
    // CID = (Cc_eff / C_victim) × slew_aggressor
    double cid = SignalIntegrityAnalyzer::compute_cid(1.75, 0.5, 50.0);
    // 1.75/0.5 × 50 = 175 ps
    CHECK(std::abs(cid - 175.0) < 0.01, "CID = 175 ps");
    PASS("cid_basic");
}

TEST(cid_zero_victim_cap) {
    double cid = SignalIntegrityAnalyzer::compute_cid(1.0, 0.0, 50.0);
    CHECK(cid == 0.0, "CID with zero victim cap = 0");
    PASS("cid_zero_victim_cap");
}

// ── Glitch Filtering Tests ───────────────────────────────────────────────

TEST(glitch_passes) {
    CHECK(SignalIntegrityAnalyzer::glitch_passes_filter(40.0, 30.0) == true,
          "40ps > 30ps gate delay → passes");
    PASS("glitch_passes");
}

TEST(glitch_filtered) {
    CHECK(SignalIntegrityAnalyzer::glitch_passes_filter(15.0, 30.0) == false,
          "15ps < 30ps gate delay → filtered");
    PASS("glitch_filtered");
}

// ── NIC Table Tests ──────────────────────────────────────────────────────

TEST(nic_table) {
    auto nic = SignalIntegrityAnalyzer::default_nic_table(1.8);
    CHECK(nic.size() >= 4, "NIC table has entries");
    // INV at 1.8V: dc_margin = 1800*0.35 = 630 mV
    bool found_inv = false;
    for (auto& e : nic) {
        if (e.cell_type == "INV") {
            found_inv = true;
            CHECK(std::abs(e.dc_noise_margin_mv - 630.0) < 1.0, "INV dc margin");
            CHECK(e.ac_noise_margin_mv > e.dc_noise_margin_mv, "AC > DC margin");
        }
    }
    CHECK(found_inv, "INV found in NIC table");
    PASS("nic_table");
}

// ── Noise / SSN Tests ────────────────────────────────────────────────────

TEST(noise_config_defaults) {
    NoiseConfig cfg;
    CHECK(cfg.vdd == 1.8, "default vdd");
    CHECK(cfg.num_switching_outputs == 16, "default SSN outputs");
    CHECK(cfg.pkg_inductance_nh == 0.5, "default pkg inductance");
    CHECK(cfg.di_dt_ma_per_ns == 10.0, "default di/dt");
    CHECK(cfg.freq_points == 64, "default freq points");
    CHECK(cfg.on_die_decap_nf == 50.0, "default on-die decap");
    CHECK(cfg.clock_period_ps == 1000.0, "default clock period");
    PASS("noise_config_defaults");
}

TEST(ssn_basic) {
    // V = N × L × di/dt = 16 × 0.5 nH × 10 mA/ns = 80 mV
    auto s = NoiseAnalyzer::compute_ssn(16, 0.5, 10.0);
    CHECK(s.num_outputs == 16, "SSN outputs");
    CHECK(std::abs(s.l_di_dt_mv - 5.0) < 0.01, "L×di/dt = 5 mV");
    CHECK(std::abs(s.peak_ssn_mv - 80.0) < 0.01, "peak SSN = 80 mV");
    PASS("ssn_basic");
}

TEST(ssn_single_output) {
    auto s = NoiseAnalyzer::compute_ssn(1, 1.0, 20.0);
    CHECK(std::abs(s.peak_ssn_mv - 20.0) < 0.01, "single output SSN");
    PASS("ssn_single_output");
}

TEST(freq_sweep) {
    auto pd = build_design(5);
    NoiseConfig cfg;
    cfg.freq_points = 10;
    NoiseAnalyzer na(pd, cfg);
    auto spectrum = na.frequency_sweep();
    CHECK((int)spectrum.size() == 10, "10 frequency points");
    CHECK(spectrum.front().freq_mhz >= 0.9, "first freq near 1 MHz");
    CHECK(spectrum.back().freq_mhz >= 4000.0, "last freq near 5000 MHz");
    // Impedance should be positive at all points
    for (auto& fp : spectrum) {
        CHECK(fp.impedance_mohm >= 0, "non-negative impedance");
        CHECK(fp.noise_mv >= 0, "non-negative noise");
    }
    PASS("freq_sweep");
}

TEST(timing_derating) {
    // 100 mV on 1800 mV → 5.56%
    double d = NoiseAnalyzer::timing_derating(100.0, 1800.0);
    CHECK(std::abs(d - 5.556) < 0.1, "derating ~5.6%");
    // Cap at 25%
    double d2 = NoiseAnalyzer::timing_derating(900.0, 1800.0);
    CHECK(std::abs(d2 - 25.0) < 0.01, "derating capped at 25%");
    PASS("timing_derating");
}

// ── Backward Compatibility ───────────────────────────────────────────────

TEST(backward_compat_si) {
    auto pd = build_design(15);
    // Legacy constructor
    SignalIntegrityAnalyzer si(pd, 1.8);
    auto r = si.analyze();
    CHECK(r.nets_analyzed > 0, "nets analyzed");
    CHECK(r.time_ms >= 0, "time measured");
    // Legacy fields still populated
    CHECK(r.message.size() > 0, "message populated");
    PASS("backward_compat_si");
}

TEST(backward_compat_noise) {
    auto pd = build_design(15);
    // Legacy constructor
    NoiseAnalyzer na(pd, 1.8, 10.0);
    auto r = na.analyze();
    CHECK(r.noise_margin_mv > 0, "margin set");
    CHECK(r.time_ms >= 0, "time measured");
    CHECK(r.message.size() > 0, "message populated");
    // New fields also present
    CHECK(r.ssn.num_outputs > 0, "SSN computed even in legacy mode");
    CHECK(r.freq_spectrum.size() > 0, "freq spectrum computed");
    PASS("backward_compat_noise");
}

// ── E2E Industrial ───────────────────────────────────────────────────────

TEST(e2e_industrial) {
    auto pd = build_design(20);

    // Signal Integrity with industrial config
    SiConfig si_cfg;
    si_cfg.vdd = 1.0;
    si_cfg.miller_factor = 0.8;
    si_cfg.noise_threshold_pct = 12.0;
    si_cfg.glitch_filter_ps = 15.0;
    si_cfg.coupling_distance_um = 6.0;
    SignalIntegrityAnalyzer si(pd, si_cfg);
    auto sir = si.analyze();

    CHECK(sir.nets_analyzed == 19, "all nets analyzed");
    CHECK(sir.nic_table.size() >= 4, "NIC table built");
    CHECK(sir.worst_cid_ps >= 0, "CID metric present");
    CHECK(sir.time_ms >= 0, "SI time measured");

    // Noise with industrial config
    NoiseConfig n_cfg;
    n_cfg.vdd = 1.0;
    n_cfg.num_switching_outputs = 32;
    n_cfg.pkg_inductance_nh = 0.3;
    n_cfg.di_dt_ma_per_ns = 15.0;
    n_cfg.freq_points = 32;
    n_cfg.on_die_decap_nf = 80.0;
    n_cfg.clock_period_ps = 500.0;
    NoiseAnalyzer na(pd, n_cfg);
    auto nr = na.analyze();

    CHECK(nr.ssn.peak_ssn_mv > 0, "SSN computed");
    // 32 × 0.3 × 15 = 144 mV
    CHECK(std::abs(nr.ssn.peak_ssn_mv - 144.0) < 0.1, "SSN = 144 mV");
    CHECK((int)nr.freq_spectrum.size() == 32, "32 freq points");
    CHECK(nr.noise_budget_mv > 0, "noise budget set");
    CHECK(nr.timing_derating_pct >= 0, "derating computed");
    CHECK(nr.time_ms >= 0, "noise time measured");

    PASS("e2e_industrial");
}

// ── main ─────────────────────────────────────────────────────────────────

int main() {
    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 37: Signal Integrity Industrial\n";
    std::cout << "══════════════════════════════════════════════════\n\n";

    std::cout << "── SI Config ──\n";
    RUN(si_config_defaults);
    RUN(si_config_custom);
    RUN(si_set_config);

    std::cout << "\n── Miller Factor ──\n";
    RUN(miller_factor_opposite);
    RUN(miller_factor_same_dir);
    RUN(miller_factor_zero);

    std::cout << "\n── Timing Windows ──\n";
    RUN(timing_window_full_overlap);
    RUN(timing_window_no_overlap);
    RUN(timing_window_partial);

    std::cout << "\n── CID ──\n";
    RUN(cid_basic);
    RUN(cid_zero_victim_cap);

    std::cout << "\n── Glitch Filtering ──\n";
    RUN(glitch_passes);
    RUN(glitch_filtered);

    std::cout << "\n── NIC Table ──\n";
    RUN(nic_table);

    std::cout << "\n── Noise Config & SSN ──\n";
    RUN(noise_config_defaults);
    RUN(ssn_basic);
    RUN(ssn_single_output);
    RUN(freq_sweep);
    RUN(timing_derating);

    std::cout << "\n── Backward Compat ──\n";
    RUN(backward_compat_si);
    RUN(backward_compat_noise);

    std::cout << "\n── E2E Industrial ──\n";
    RUN(e2e_industrial);

    std::cout << "\n══════════════════════════════════════════════════\n";
    std::cout << "  Phase 37: " << passed << " passed, " << failed << " failed\n";
    std::cout << "══════════════════════════════════════════════════\n\n";
    return failed > 0 ? 1 : 0;
}
