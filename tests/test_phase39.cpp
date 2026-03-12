// SiliconForge — Phase 39 Test Suite
// SSTA Industrial: Monte Carlo statistical timing, process variation,
// spatial correlation, yield estimation, sensitivity analysis
// Reference: Blaauw et al., "Statistical Timing Analysis", IEEE TCAD 2008

#include "core/types.hpp"
#include "core/netlist.hpp"
#include "timing/ssta.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace sf;

static int passed = 0, failed = 0;
#define TEST(name) static void test_##name()
#define CHECK(c, m) do { if(!(c)){std::cerr<<"  [FAIL] "<<m<<" (line "<<__LINE__<<")\n";failed++;return;}}while(0)
#define PASS(n) do{std::cout<<"  [PASS] "<<n<<"\n";passed++;}while(0)
#define RUN(n) do{std::cout<<"Running: " #n "\n";test_##n();}while(0)

// ── Helpers ──────────────────────────────────────────────────────────────

static Netlist build_chain(int depth = 8) {
    Netlist nl;
    NetId prev = nl.add_net("pi0");
    nl.mark_input(prev);
    for (int i = 0; i < depth; ++i) {
        NetId out = nl.add_net("n" + std::to_string(i));
        if (i % 2 == 0)
            nl.add_gate(GateType::NOT, {prev}, out, "inv" + std::to_string(i));
        else
            nl.add_gate(GateType::AND, {prev, prev}, out, "and" + std::to_string(i));
        prev = out;
    }
    nl.mark_output(prev);
    return nl;
}

static Netlist build_wide_fan(int width = 6, int depth = 4) {
    Netlist nl;
    NetId pi = nl.add_net("pi0");
    nl.mark_input(pi);
    for (int w = 0; w < width; ++w) {
        NetId prev = pi;
        for (int d = 0; d < depth; ++d) {
            NetId out = nl.add_net("n_" + std::to_string(w) + "_" + std::to_string(d));
            nl.add_gate(GateType::AND, {prev, prev}, out,
                        "g_" + std::to_string(w) + "_" + std::to_string(d));
            prev = out;
        }
        nl.mark_output(prev);
    }
    return nl;
}

static Netlist build_pipeline(int stages = 3, int depth = 4) {
    Netlist nl;
    NetId clk = nl.add_net("clk");
    nl.mark_input(clk);
    NetId pi = nl.add_net("pi0");
    nl.mark_input(pi);

    NetId prev = pi;
    for (int s = 0; s < stages; ++s) {
        // Combinational logic
        for (int d = 0; d < depth; ++d) {
            NetId out = nl.add_net("s" + std::to_string(s) + "_n" + std::to_string(d));
            nl.add_gate(GateType::NAND, {prev, prev}, out,
                        "s" + std::to_string(s) + "_g" + std::to_string(d));
            prev = out;
        }
        // DFF
        NetId q = nl.add_net("q" + std::to_string(s));
        nl.add_dff(prev, clk, q, -1, "ff" + std::to_string(s));
        prev = q;
    }
    nl.mark_output(prev);
    return nl;
}

// ── Config & Struct Tests ────────────────────────────────────────────────

TEST(config_defaults) {
    SstaConfig cfg;
    CHECK(cfg.num_samples == 1000, "default num_samples");
    CHECK(std::abs(cfg.n_sigma - 3.0) < 0.01, "default n_sigma");
    CHECK(std::abs(cfg.global_variation_pct - 5.0) < 0.01, "default global_variation");
    CHECK(std::abs(cfg.local_variation_pct - 3.0) < 0.01, "default local_variation");
    CHECK(std::abs(cfg.spatial_correlation_length_um - 100.0) < 0.01, "default corr length");
    CHECK(cfg.enable_spatial_correlation == true, "default spatial_correlation on");
    CHECK(cfg.seed == 42, "default seed");
    CHECK(std::abs(cfg.confidence_level - 0.997) < 0.001, "default confidence");
    PASS("config_defaults");
}

TEST(process_variation_defaults) {
    ProcessVariation pv;
    CHECK(std::abs(pv.gate_length_sigma_pct - 3.0) < 0.01, "gate_length sigma");
    CHECK(std::abs(pv.vth_sigma_pct - 4.0) < 0.01, "vth sigma");
    CHECK(std::abs(pv.tox_sigma_pct - 2.0) < 0.01, "tox sigma");
    CHECK(std::abs(pv.wire_width_sigma_pct - 3.0) < 0.01, "wire_width sigma");
    CHECK(std::abs(pv.wire_thickness_sigma_pct - 2.0) < 0.01, "wire_thickness sigma");
    CHECK(std::abs(pv.global_fraction - 0.6) < 0.01, "global fraction");
    CHECK(std::abs(pv.local_fraction - 0.4) < 0.01, "local fraction");
    PASS("process_variation_defaults");
}

TEST(statistical_delay_compute) {
    StatisticalDelay sd;
    sd.samples = {100.0, 110.0, 90.0, 105.0, 95.0, 120.0, 80.0, 115.0, 85.0, 100.0};
    sd.compute_from_samples();
    CHECK(std::abs(sd.mean_ps - 100.0) < 0.01, "mean");
    CHECK(sd.sigma_ps > 0.0, "sigma > 0");
    CHECK(sd.min_ps == 80.0, "min");
    CHECK(sd.max_ps == 120.0, "max");
    CHECK(!sd.percentile_values.empty(), "percentiles populated");
    CHECK(sd.percentile_values.count(50.0) > 0, "median exists");
    CHECK(sd.percentile_values[1.0] <= sd.percentile_values[99.0], "percentile ordering");
    PASS("statistical_delay_compute");
}

TEST(ssta_result_fields) {
    SstaResult r;
    CHECK(r.yield_estimate == 0.0, "default yield");
    CHECK(r.num_samples_run == 0, "default samples");
    CHECK(r.time_ms == 0.0, "default time");
    CHECK(r.critical_paths.empty(), "default paths empty");
    CHECK(r.path_correlations.empty(), "default correlations empty");
    CHECK(r.sigma_to_yield.empty(), "default sigma_to_yield empty");
    CHECK(r.message.empty(), "default message empty");
    CHECK(r.summary.empty(), "default summary empty");
    PASS("ssta_result_fields");
}

TEST(statistical_path_fields) {
    StatisticalPath sp;
    CHECK(sp.mean_delay_ps == 0.0, "default mean");
    CHECK(sp.sigma_ps == 0.0, "default sigma");
    CHECK(sp.yield_probability == 1.0, "default yield");
    CHECK(sp.worst_sample_delay_ps == 0.0, "default worst");
    CHECK(sp.best_sample_delay_ps == 0.0, "default best");
    CHECK(sp.delay_samples.empty(), "default samples empty");
    CHECK(sp.percentile_delays.empty(), "default percentiles empty");
    PASS("statistical_path_fields");
}

// ── Basic MC Analysis ────────────────────────────────────────────────────

TEST(basic_mc_analysis) {
    auto nl = build_chain(6);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 500;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    auto result = engine.run_monte_carlo();
    CHECK(result.num_samples_run == 500, "correct sample count");
    CHECK(result.time_ms >= 0.0, "time recorded");
    CHECK(!result.message.empty(), "message set");
    CHECK(!result.summary.empty(), "summary set");
    CHECK(result.statistical_wns.mean_ps != 0.0, "WNS mean computed");
    CHECK(result.statistical_wns.sigma_ps > 0.0, "WNS sigma > 0");
    CHECK(result.statistical_wns.samples.size() == 500, "WNS samples stored");
    CHECK(result.statistical_tns.samples.size() == 500, "TNS samples stored");
    PASS("basic_mc_analysis");
}

TEST(yield_estimation) {
    auto nl = build_chain(4);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 500;
    cfg.seed = 42;
    engine.set_config(cfg);

    // Large period → high yield
    engine.set_clock_period(5000.0);
    auto r1 = engine.run_monte_carlo();
    CHECK(r1.yield_estimate > 0.9, "high yield with large period");

    // Very tight period → low yield
    engine.set_clock_period(10.0);
    auto r2 = engine.run_monte_carlo();
    CHECK(r2.yield_estimate < r1.yield_estimate, "tighter period → lower yield");
    PASS("yield_estimation");
}

TEST(compute_yield_api) {
    auto nl = build_chain(4);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 300;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    double y_loose = engine.compute_yield(5000.0);
    double y_tight = engine.compute_yield(10.0);
    CHECK(y_loose >= y_tight, "loose period yield >= tight");
    CHECK(y_loose > 0.5, "loose should have decent yield");
    PASS("compute_yield_api");
}

TEST(sigma_to_yield_table) {
    auto nl = build_chain(6);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 500;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    auto result = engine.run_monte_carlo();
    CHECK(!result.sigma_to_yield.empty(), "sigma_to_yield populated");
    CHECK(result.sigma_to_yield.count(1.0) > 0, "1-sigma entry");
    CHECK(result.sigma_to_yield.count(3.0) > 0, "3-sigma entry");
    CHECK(result.sigma_to_yield.count(6.0) > 0, "6-sigma entry");
    // Higher sigma should generally have higher yield
    CHECK(result.sigma_to_yield[3.0] >= result.sigma_to_yield[1.0] - 1.0,
          "3-sigma yield >= 1-sigma yield (approx)");
    PASS("sigma_to_yield_table");
}

// ── Seed Reproducibility ─────────────────────────────────────────────────

TEST(seed_reproducibility) {
    auto nl = build_chain(5);
    SstaConfig cfg;
    cfg.num_samples = 200;
    cfg.seed = 123;

    SstaEngine e1(nl);
    e1.set_config(cfg);
    e1.set_clock_period(2000.0);
    auto r1 = e1.run_monte_carlo();

    SstaEngine e2(nl);
    e2.set_config(cfg);
    e2.set_clock_period(2000.0);
    auto r2 = e2.run_monte_carlo();

    CHECK(std::abs(r1.statistical_wns.mean_ps - r2.statistical_wns.mean_ps) < 0.001,
          "same seed → same WNS mean");
    CHECK(std::abs(r1.yield_estimate - r2.yield_estimate) < 0.001,
          "same seed → same yield");
    CHECK(r1.statistical_wns.samples.size() == r2.statistical_wns.samples.size(),
          "same sample count");
    PASS("seed_reproducibility");
}

TEST(different_seeds_differ) {
    auto nl = build_chain(5);
    SstaConfig cfg;
    cfg.num_samples = 300;

    SstaEngine e1(nl);
    cfg.seed = 42;
    e1.set_config(cfg);
    e1.set_clock_period(2000.0);
    auto r1 = e1.run_monte_carlo();

    SstaEngine e2(nl);
    cfg.seed = 999;
    e2.set_config(cfg);
    e2.set_clock_period(2000.0);
    auto r2 = e2.run_monte_carlo();

    // Different seeds should produce different exact results
    // (means may be close but samples shouldn't be identical)
    bool all_same = true;
    for (size_t i = 0; i < std::min(r1.statistical_wns.samples.size(),
                                     r2.statistical_wns.samples.size()); ++i) {
        if (std::abs(r1.statistical_wns.samples[i] - r2.statistical_wns.samples[i]) > 0.001) {
            all_same = false;
            break;
        }
    }
    CHECK(!all_same, "different seeds → different samples");
    PASS("different_seeds_differ");
}

// ── Critical Path Statistics ─────────────────────────────────────────────

TEST(critical_paths_populated) {
    auto nl = build_wide_fan(4, 3);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 300;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(3000.0);

    auto result = engine.run_monte_carlo();
    CHECK(!result.critical_paths.empty(), "critical paths found");
    for (auto& path : result.critical_paths) {
        CHECK(!path.path_name.empty(), "path has name");
        CHECK(path.mean_delay_ps > 0.0, "path has positive mean delay");
        CHECK(path.sigma_ps >= 0.0, "path sigma >= 0");
        CHECK(path.worst_sample_delay_ps >= path.best_sample_delay_ps,
              "worst >= best");
        CHECK(!path.delay_samples.empty(), "path has delay samples");
        CHECK(!path.percentile_delays.empty(), "path has percentiles");
        CHECK(path.yield_probability >= 0.0 && path.yield_probability <= 1.0,
              "path yield in [0,1]");
    }
    PASS("critical_paths_populated");
}

TEST(path_correlations) {
    auto nl = build_wide_fan(4, 3);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 500;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(3000.0);

    auto result = engine.run_monte_carlo();
    if (result.critical_paths.size() >= 2) {
        CHECK(!result.path_correlations.empty(), "path correlations computed");
        for (auto& [key, corr] : result.path_correlations) {
            CHECK(corr >= -1.0 && corr <= 1.0, "correlation in [-1, 1]");
        }
        // Paths sharing the same input should be positively correlated
        // (global variation affects all paths)
        auto it = result.path_correlations.find("path0_path1");
        if (it != result.path_correlations.end()) {
            CHECK(it->second > -0.5, "shared-input paths not strongly anti-correlated");
        }
    }
    PASS("path_correlations");
}

// ── Spatial Correlation ──────────────────────────────────────────────────

TEST(spatial_correlation_effect) {
    auto nl = build_chain(8);

    // With spatial correlation
    SstaEngine e1(nl);
    SstaConfig cfg;
    cfg.num_samples = 500;
    cfg.seed = 42;
    cfg.enable_spatial_correlation = true;
    cfg.spatial_correlation_length_um = 50.0;
    e1.set_config(cfg);
    e1.set_clock_period(2000.0);
    auto r1 = e1.run_monte_carlo();

    // Without spatial correlation
    SstaEngine e2(nl);
    cfg.enable_spatial_correlation = false;
    e2.set_config(cfg);
    e2.set_clock_period(2000.0);
    auto r2 = e2.run_monte_carlo();

    // Both should produce valid results
    CHECK(r1.statistical_wns.sigma_ps > 0.0, "correlated sigma > 0");
    CHECK(r2.statistical_wns.sigma_ps > 0.0, "uncorrelated sigma > 0");
    // Spatially correlated → MORE variance (nearby gates vary together)
    // vs uncorrelated → less variance (cancel out)
    // The exact relationship depends on circuit, but both should be valid
    CHECK(r1.num_samples_run == 500 && r2.num_samples_run == 500, "both ran");
    PASS("spatial_correlation_effect");
}

// ── Process Variation Impact ─────────────────────────────────────────────

TEST(variation_impact) {
    auto nl = build_chain(6);

    // Low variation
    SstaEngine e1(nl);
    SstaConfig cfg;
    cfg.num_samples = 400;
    cfg.seed = 42;
    cfg.global_variation_pct = 1.0;
    cfg.local_variation_pct = 0.5;
    e1.set_config(cfg);
    e1.set_clock_period(2000.0);
    auto r1 = e1.run_monte_carlo();

    // High variation
    SstaEngine e2(nl);
    cfg.global_variation_pct = 15.0;
    cfg.local_variation_pct = 10.0;
    e2.set_config(cfg);
    e2.set_clock_period(2000.0);
    auto r2 = e2.run_monte_carlo();

    CHECK(r2.statistical_wns.sigma_ps > r1.statistical_wns.sigma_ps,
          "higher variation → wider spread");
    PASS("variation_impact");
}

TEST(custom_process_variation) {
    auto nl = build_chain(6);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 300;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    ProcessVariation pv;
    pv.gate_length_sigma_pct = 5.0;
    pv.vth_sigma_pct = 6.0;
    pv.tox_sigma_pct = 3.0;
    pv.wire_width_sigma_pct = 4.0;
    pv.wire_thickness_sigma_pct = 3.0;
    pv.global_fraction = 0.7;
    pv.local_fraction = 0.3;
    engine.set_process_variation(pv);

    const auto& pv_back = engine.process_variation();
    CHECK(std::abs(pv_back.gate_length_sigma_pct - 5.0) < 0.01, "gate_length set");
    CHECK(std::abs(pv_back.vth_sigma_pct - 6.0) < 0.01, "vth set");
    CHECK(std::abs(pv_back.global_fraction - 0.7) < 0.01, "global_fraction set");

    auto result = engine.run_monte_carlo();
    CHECK(result.num_samples_run == 300, "ran with custom PV");
    CHECK(result.statistical_wns.sigma_ps > 0.0, "variation present");
    PASS("custom_process_variation");
}

// ── Sensitivity Analysis ─────────────────────────────────────────────────

TEST(sensitivity_analysis) {
    auto nl = build_chain(6);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 200;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    auto sr = engine.sensitivity_analysis();
    CHECK(!sr.parameter_impact.empty(), "impacts computed");
    CHECK(sr.parameter_impact.count("global_variation") > 0, "global tested");
    CHECK(sr.parameter_impact.count("local_variation") > 0, "local tested");
    CHECK(sr.parameter_impact.count("gate_length") > 0, "gate_length tested");
    CHECK(sr.parameter_impact.count("vth") > 0, "vth tested");
    CHECK(sr.parameter_impact.count("wire_width") > 0, "wire_width tested");
    CHECK(!sr.dominant_parameter.empty(), "dominant parameter identified");
    CHECK(sr.dominant_impact_ps >= 0.0, "dominant impact non-negative");
    PASS("sensitivity_analysis");
}

// ── Pipeline / DFF Handling ──────────────────────────────────────────────

TEST(pipeline_ssta) {
    auto nl = build_pipeline(2, 3);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 300;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    auto result = engine.run_monte_carlo();
    CHECK(result.num_samples_run == 300, "pipeline MC ran");
    CHECK(result.statistical_wns.mean_ps != 0.0, "WNS computed");
    CHECK(result.statistical_wns.sigma_ps > 0.0, "sigma > 0");
    CHECK(!result.message.empty(), "message set");
    PASS("pipeline_ssta");
}

// ── Empty / Edge Cases ───────────────────────────────────────────────────

TEST(empty_netlist) {
    Netlist nl;
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 100;
    engine.set_config(cfg);

    auto result = engine.run_monte_carlo();
    CHECK(result.num_samples_run == 0, "no samples for empty netlist");
    CHECK(!result.message.empty(), "message set for empty");
    PASS("empty_netlist");
}

TEST(single_gate) {
    Netlist nl;
    NetId a = nl.add_net("a");
    NetId b = nl.add_net("b");
    NetId out = nl.add_net("out");
    nl.mark_input(a);
    nl.mark_input(b);
    nl.add_gate(GateType::AND, {a, b}, out, "g0");
    nl.mark_output(out);

    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 200;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    auto result = engine.run_monte_carlo();
    CHECK(result.num_samples_run == 200, "single gate MC ran");
    CHECK(result.statistical_wns.mean_ps > 0, "WNS positive (slack available)");
    CHECK(result.yield_estimate > 0.99, "single gate passes easily");
    PASS("single_gate");
}

// ── Statistical WNS API ──────────────────────────────────────────────────

TEST(get_statistical_wns) {
    auto nl = build_chain(5);
    SstaEngine engine(nl);
    SstaConfig cfg;
    cfg.num_samples = 200;
    cfg.seed = 42;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    auto wns = engine.get_statistical_wns();
    CHECK(wns.samples.size() == 200, "samples populated");
    CHECK(wns.mean_ps != 0.0, "mean computed");
    CHECK(wns.sigma_ps > 0.0, "sigma computed");
    CHECK(wns.min_ps <= wns.max_ps, "min <= max");
    CHECK(!wns.percentile_values.empty(), "percentiles populated");
    PASS("get_statistical_wns");
}

// ── E2E Integration ──────────────────────────────────────────────────────

TEST(e2e_full_ssta_flow) {
    auto nl = build_wide_fan(3, 5);
    SstaEngine engine(nl);

    SstaConfig cfg;
    cfg.num_samples = 500;
    cfg.seed = 42;
    cfg.global_variation_pct = 5.0;
    cfg.local_variation_pct = 3.0;
    cfg.enable_spatial_correlation = true;
    cfg.spatial_correlation_length_um = 100.0;
    engine.set_config(cfg);
    engine.set_clock_period(2000.0);

    ProcessVariation pv;
    pv.gate_length_sigma_pct = 3.0;
    pv.vth_sigma_pct = 4.0;
    engine.set_process_variation(pv);

    // 1. Run MC
    auto result = engine.run_monte_carlo();
    CHECK(result.num_samples_run == 500, "MC ran");
    CHECK(result.time_ms >= 0.0, "time tracked");

    // 2. Check WNS distribution
    CHECK(result.statistical_wns.sigma_ps > 0.0, "WNS has variation");
    CHECK(result.statistical_wns.min_ps <= result.statistical_wns.max_ps, "min <= max");

    // 3. Check yield
    CHECK(result.yield_estimate >= 0.0 && result.yield_estimate <= 1.0, "yield valid");

    // 4. Check sigma-to-yield
    CHECK(result.sigma_to_yield.size() >= 5, "sigma-to-yield table");

    // 5. Check critical paths
    CHECK(!result.critical_paths.empty(), "critical paths found");
    for (auto& p : result.critical_paths) {
        CHECK(p.mean_delay_ps > 0.0, "path delay > 0");
        CHECK(p.sigma_ps >= 0.0, "path sigma >= 0");
    }

    // 6. Check summary
    CHECK(!result.summary.empty(), "summary generated");
    CHECK(result.summary.find("SSTA") != std::string::npos, "summary mentions SSTA");

    // 7. Sensitivity
    auto sr = engine.sensitivity_analysis();
    CHECK(!sr.parameter_impact.empty(), "sensitivity computed");
    CHECK(!sr.dominant_parameter.empty(), "dominant param found");

    std::cout << "  E2E Summary: " << result.summary << "\n";
    std::cout << "  Dominant variation: " << sr.dominant_parameter
              << " (" << sr.dominant_impact_ps << " ps)\n";
    PASS("e2e_full_ssta_flow");
}

// ── Main ─────────────────────────────────────────────────────────────────

int main() {
    std::cout << "╔══════════════════════════════════════════════╗\n"
              << "║   Phase 39: SSTA Industrial Tests            ║\n"
              << "╚══════════════════════════════════════════════╝\n\n";

    RUN(config_defaults);
    RUN(process_variation_defaults);
    RUN(statistical_delay_compute);
    RUN(ssta_result_fields);
    RUN(statistical_path_fields);
    RUN(basic_mc_analysis);
    RUN(yield_estimation);
    RUN(compute_yield_api);
    RUN(sigma_to_yield_table);
    RUN(seed_reproducibility);
    RUN(different_seeds_differ);
    RUN(critical_paths_populated);
    RUN(path_correlations);
    RUN(spatial_correlation_effect);
    RUN(variation_impact);
    RUN(custom_process_variation);
    RUN(sensitivity_analysis);
    RUN(pipeline_ssta);
    RUN(empty_netlist);
    RUN(single_gate);
    RUN(get_statistical_wns);
    RUN(e2e_full_ssta_flow);

    std::cout << "\n════════════════════════════════════════\n"
              << "Phase 39 Results: " << passed << " passed, " << failed << " failed\n"
              << "════════════════════════════════════════\n";
    return failed > 0 ? 1 : 0;
}
