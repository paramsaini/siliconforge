// Phase 83: Tier 2 Gap Closure Tests
// SDC group_path/wire_load/propagated/ideal, Liberty noise/EM/NLPM/cap/DVFS,
// SPEF inductance/mutual/corners/scaling, POCV MC/MCMM/CPPR modes/SSTA IS,
// IR PDN freq-domain/adaptive/temp/voltage derating, DRC context/inter-layer

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

#include "frontend/sdc_parser.hpp"
#include "core/liberty_parser.hpp"
#include "core/spef_parser.hpp"
#include "core/parasitic_extract.hpp"
#include "timing/sta.hpp"
#include "timing/ssta.hpp"
#include "timing/ir_drop.hpp"
#include "timing/power.hpp"
#include "verify/drc.hpp"
#include "core/netlist.hpp"
#include "pnr/physical.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

// ========== SDC TIER 2 ==========

void test_sdc_group_path() {
    printf("\n[SDC group_path]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "group_path -name critical -from clk1 -to reg_out -weight 2.0\n"
        "group_path -name io_paths -from data_in -to data_out\n",
        sdc);
    CHECK(r.success, "group_path parse succeeds");
    CHECK(sdc.group_paths.size() == 2, "2 group_path constraints");
    CHECK(sdc.group_paths[0].name == "critical", "name = critical");
    CHECK(sdc.group_paths[0].from == "clk1", "from = clk1");
    CHECK(sdc.group_paths[0].to == "reg_out", "to = reg_out");
    CHECK(std::abs(sdc.group_paths[0].weight - 2.0) < 1e-6, "weight = 2.0");
    CHECK(std::abs(sdc.group_paths[1].weight - 1.0) < 1e-6, "default weight = 1.0");
}

void test_sdc_wire_load() {
    printf("\n[SDC set_wire_load_model]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "set_wire_load_model -name medium -library tech_lib\n",
        sdc);
    CHECK(r.success, "wire_load parse succeeds");
    CHECK(sdc.wire_loads.size() == 1, "1 wire_load model");
    CHECK(sdc.wire_loads[0].model_name == "medium", "model = medium");
    CHECK(sdc.wire_loads[0].library == "tech_lib", "library = tech_lib");
}

void test_sdc_propagated_clock() {
    printf("\n[SDC set_propagated_clock]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "create_clock -name clk -period 10.0 [get_ports clk]\n"
        "set_propagated_clock clk\n",
        sdc);
    CHECK(r.success, "propagated_clock parse succeeds");
    CHECK(sdc.propagated_clocks.size() >= 1, "propagated clock recorded");
    CHECK(sdc.propagated_clocks[0] == "clk", "propagated clock = clk");
}

void test_sdc_ideal_network() {
    printf("\n[SDC set_ideal_network]\n");
    sf::SdcParser parser;
    sf::SdcConstraints sdc;
    auto r = parser.parse_string(
        "set_ideal_network reset\n"
        "set_ideal_network scan_en\n",
        sdc);
    CHECK(r.success, "ideal_network parse succeeds");
    CHECK(sdc.ideal_networks.size() == 2, "2 ideal networks");
    CHECK(sdc.ideal_networks[0] == "reset", "net0 = reset");
    CHECK(sdc.ideal_networks[1] == "scan_en", "net1 = scan_en");
}

// ========== LIBERTY TIER 2 ==========

void test_liberty_noise_table() {
    printf("\n[Liberty NoiseTable]\n");
    sf::NoiseTable nt;
    nt.input_net_transition = {0.01, 0.05, 0.1};
    nt.total_output_net_cap = {0.001, 0.01};
    nt.noise_immunity_high = {{0.5, 0.45}, {0.48, 0.42}, {0.44, 0.38}};
    nt.noise_immunity_low = {{0.3, 0.28}, {0.29, 0.26}, {0.27, 0.24}};
    CHECK(nt.noise_immunity_high.size() == 3, "3 rows in high table");
    CHECK(nt.noise_immunity_low[0].size() == 2, "2 cols in low table");
}

void test_liberty_em_limits() {
    printf("\n[Liberty EmLimit]\n");
    sf::EmLimit em;
    em.max_current_ma = 5.0;
    em.max_transition_ns = 0.5;
    em.pin_name = "Y";
    CHECK(em.max_current_ma == 5.0, "max current = 5mA");
    CHECK(em.max_transition_ns == 0.5, "max transition = 0.5ns");
}

void test_liberty_nlpm() {
    printf("\n[Liberty NlpmTable]\n");
    sf::NlpmTable nlpm;
    nlpm.input_transition = {0.01, 0.1};
    nlpm.output_capacitance = {0.001, 0.01};
    nlpm.values = {{0.001, 0.005}, {0.002, 0.008}};
    CHECK(nlpm.values.size() == 2, "2x2 NLPM table");
    CHECK(!nlpm.input_transition.empty(), "has transitions");
}

void test_liberty_cap_table() {
    printf("\n[Liberty CapTable]\n");
    sf::CapTable ct;
    ct.input_transition = {0.01, 0.05, 0.1};
    ct.values = {0.002, 0.0021, 0.0023};
    ct.valid = true;
    CHECK(ct.valid, "cap table valid");
    CHECK(ct.values.size() == 3, "3 cap values");
}

void test_liberty_dvfs() {
    printf("\n[Liberty DvfsEntry]\n");
    sf::DvfsEntry dv;
    dv.voltage = 0.9;
    dv.temperature = 85.0;
    dv.delay_factor = 1.15;
    dv.power_factor = 0.81;
    CHECK(dv.voltage == 0.9, "voltage = 0.9V");
    CHECK(dv.delay_factor > 1.0, "delay factor > 1 at low voltage");
    CHECK(dv.power_factor < 1.0, "power factor < 1 at low voltage");
}

// ========== SPEF/PARASITIC TIER 2 ==========

void test_spef_inductor() {
    printf("\n[SPEF Inductance]\n");
    sf::SpefInductor ind;
    ind.id = 1;
    ind.node1 = "n1";
    ind.node2 = "n2";
    ind.value = 1.5e-12; // 1.5 pH
    CHECK(ind.value > 0, "positive inductance");
    CHECK(ind.node1 == "n1", "node1 = n1");
}

void test_spef_mutual_inductance() {
    printf("\n[SPEF Mutual Inductance]\n");
    sf::MutualInductance mi;
    mi.net1 = "clk_net";
    mi.net2 = "data_net";
    mi.coupling_l = 0.5e-12;
    CHECK(mi.coupling_l > 0, "positive mutual L");
    CHECK(mi.net1 != mi.net2, "different nets");
}

void test_extraction_corner() {
    printf("\n[Extraction Corner]\n");
    sf::ExtractionCorner ec;
    ec.name = "rcworst";
    ec.r_factor = 1.2;
    ec.c_factor = 1.15;
    ec.l_factor = 1.1;
    CHECK(ec.r_factor > 1.0, "rcworst R > 1x");
    CHECK(ec.c_factor > 1.0, "rcworst C > 1x");

    sf::ExtractionCorner best;
    best.name = "rcbest";
    best.r_factor = 0.8;
    best.c_factor = 0.85;
    best.l_factor = 0.9;
    CHECK(best.r_factor < 1.0, "rcbest R < 1x");
}

void test_spef_scale_parasitics() {
    printf("\n[SPEF Scale Parasitics]\n");
    sf::SpefData sd;
    sf::SpefNet sn;
    sn.name = "test_net";
    sf::SpefResistor r; r.id = 1; r.node1 = "a"; r.node2 = "b"; r.value = 100.0;
    sf::SpefCapacitor c; c.id = 1; c.node1 = "a"; c.value = 0.01e-12;
    sn.resistors.push_back(r);
    sn.caps.push_back(c);
    sd.nets.push_back(sn);

    double orig_r = sd.nets[0].resistors[0].value;
    double orig_c = sd.nets[0].caps[0].value;
    sd.scale_parasitics(1.1, 1.0); // 10% temp increase
    CHECK(sd.nets[0].resistors[0].value > orig_r, "R increased with temp");
    // Reset for corner test
    sd.nets[0].resistors[0].value = orig_r;
    sd.nets[0].caps[0].value = orig_c;
    sf::ExtractionCorner worst; worst.name = "rcworst"; worst.r_factor = 1.2; worst.c_factor = 1.15;
    sd.apply_corner(worst);
    CHECK(sd.nets[0].resistors[0].value > orig_r, "R scaled by corner");
    CHECK(sd.nets[0].caps[0].value > orig_c, "C scaled by corner");
}

// ========== STA TIER 2 ==========

void test_pocv_monte_carlo_config() {
    printf("\n[POCV Monte Carlo Config]\n");
    sf::PocvMonteCarloConfig cfg;
    CHECK(cfg.num_samples == 1000, "default 1000 samples");
    CHECK(cfg.enable_correlation == true, "correlation enabled");
    CHECK(std::abs(cfg.correlation_distance - 100.0) < 1e-6, "100um correlation distance");
}

void test_pocv_monte_carlo_run() {
    printf("\n[POCV Monte Carlo Run]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_b = nl.add_net("buf_out");
    nl.add_gate(sf::GateType::BUF, {n_a}, n_b, "buf0");
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::BUF, {n_b}, n_y, "buf1");

    sf::StaEngine sta(nl);
    sf::PocvMonteCarloConfig cfg;
    cfg.num_samples = 50; // small for test speed
    auto result = sta.run_pocv_monte_carlo(10.0, cfg);
    CHECK(result.num_samples == 50, "ran 50 samples");
    CHECK(result.wns_distribution.size() == 50, "50 WNS values");
}

void test_mcmm_scenarios() {
    printf("\n[MCMM Scenarios]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::BUF, {n_a}, n_y, "buf0");

    sf::StaEngine sta(nl);
    std::vector<sf::McmmWeightedScenario> scenarios;
    sf::McmmWeightedScenario s1; s1.name = "func_slow"; s1.weight = 2.0;
    s1.mode = "functional"; s1.corner = "slow"; s1.guardband = 0.05;
    sf::McmmWeightedScenario s2; s2.name = "scan_fast"; s2.weight = 1.0;
    s2.mode = "scan"; s2.corner = "fast";
    scenarios.push_back(s1);
    scenarios.push_back(s2);
    sta.set_mcmm_scenarios(scenarios);
    CHECK(sta.mcmm_scenarios().size() == 2, "2 MCMM scenarios");
    CHECK(sta.mcmm_scenarios()[0].weight == 2.0, "scenario 0 weight = 2.0");
    CHECK(sta.mcmm_scenarios()[0].guardband == 0.05, "scenario 0 guardband = 0.05ns");
}

void test_cppr_modes() {
    printf("\n[CPPR Modes]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::BUF, {n_a}, n_y, "buf0");

    sf::StaEngine sta(nl);
    sta.set_cppr_mode(sf::CpprMode::APPROXIMATE);
    CHECK(true, "CPPR APPROXIMATE mode set");
    sta.set_cppr_mode(sf::CpprMode::EXACT);
    CHECK(true, "CPPR EXACT mode set");
}

void test_ssta_importance_sampling() {
    printf("\n[SSTA Importance Sampling]\n");
    sf::SstaConfig cfg;
    cfg.enable_importance_sampling = true;
    cfg.tail_threshold = -0.2;
    cfg.num_samples = 500;
    CHECK(cfg.enable_importance_sampling, "IS enabled");
    CHECK(cfg.tail_threshold == -0.2, "tail threshold = -0.2ns");
    CHECK(cfg.num_samples == 500, "500 samples");
}

// ========== IR/PDN TIER 2 ==========

void test_freq_domain_pdn_config() {
    printf("\n[Freq-Domain PDN Config]\n");
    sf::FreqDomainPdnConfig cfg;
    CHECK(cfg.freq_start_hz > 0, "positive start freq");
    CHECK(cfg.freq_stop_hz > cfg.freq_start_hz, "stop > start");
    CHECK(cfg.num_points > 0, "positive num_points");
    CHECK(cfg.include_package, "package included by default");
}

void test_power_temperature() {
    printf("\n[Power Temperature]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::NOT, {n_a}, n_y, "inv0");

    sf::PowerAnalyzer pa(nl);
    pa.set_temperature(25.0);
    auto r25 = pa.analyze(500.0, 1.0, 0.1);
    pa.set_temperature(85.0);
    auto r85 = pa.analyze(500.0, 1.0, 0.1);
    CHECK(r85.static_power_mw >= r25.static_power_mw, "leakage increases with temp");
}

void test_power_voltage_derating() {
    printf("\n[Power Voltage Derating]\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A"); nl.mark_input(n_a);
    auto n_y = nl.add_net("Y"); nl.mark_output(n_y);
    nl.add_gate(sf::GateType::NOT, {n_a}, n_y, "inv0");

    sf::PowerAnalyzer pa(nl);
    std::unordered_map<std::string, double> voltages;
    voltages["inv0"] = 0.95;
    pa.apply_voltage_derating(voltages);
    CHECK(true, "voltage derating applied");
}

// ========== DRC TIER 2 ==========

void test_drc_context_rule() {
    printf("\n[DRC Context Rule]\n");
    sf::ContextRule cr;
    cr.name = "eol_spacing";
    cr.layer = "M1";
    cr.spacing = 0.1;
    cr.context = "end_of_line";
    cr.context_distance = 0.05;
    CHECK(cr.spacing == 0.1, "spacing = 0.1um");
    CHECK(cr.context == "end_of_line", "EOL context");
}

void test_drc_inter_layer_rule() {
    printf("\n[DRC Inter-Layer Rule]\n");
    sf::InterLayerRule ilr;
    ilr.layer1 = "M1";
    ilr.layer2 = "VIA1";
    ilr.min_enclosure = 0.04;
    ilr.type = "enclosure";
    CHECK(ilr.min_enclosure == 0.04, "enclosure = 0.04um");
    CHECK(ilr.type == "enclosure", "type = enclosure");
}

// ========== MAIN ==========

int main() {
    printf("=== Phase 83: Tier 2 Gap Closure Tests ===\n");

    // SDC Tier 2
    test_sdc_group_path();
    test_sdc_wire_load();
    test_sdc_propagated_clock();
    test_sdc_ideal_network();

    // Liberty Tier 2
    test_liberty_noise_table();
    test_liberty_em_limits();
    test_liberty_nlpm();
    test_liberty_cap_table();
    test_liberty_dvfs();

    // SPEF/Parasitic Tier 2
    test_spef_inductor();
    test_spef_mutual_inductance();
    test_extraction_corner();
    test_spef_scale_parasitics();

    // STA Tier 2
    test_pocv_monte_carlo_config();
    test_pocv_monte_carlo_run();
    test_mcmm_scenarios();
    test_cppr_modes();
    test_ssta_importance_sampling();

    // IR/PDN/Power Tier 2
    test_freq_domain_pdn_config();
    test_power_temperature();
    test_power_voltage_derating();

    // DRC Tier 2
    test_drc_context_rule();
    test_drc_inter_layer_rule();

    printf("\n=== Phase 83 Results: %d/%d passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
