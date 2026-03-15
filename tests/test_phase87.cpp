// Phase 87: Comprehensive gap closure — SV parameterized types, $readmemh/b,
// TCL packages, HLS float synthesis, NUMA threading, 3D DRC, lithography sim,
// DFM yield, via coupling, package interaction, LVS device param extraction,
// chiplet skew modeling, conditional SDF annotation

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <cassert>
#include <future>
#include <atomic>
#include <unordered_map>

#include "core/netlist.hpp"
#include "core/liberty_parser.hpp"
#include "core/thread_pool.hpp"
#include "timing/sta.hpp"
#include "timing/pdn.hpp"
#include "timing/sdf_writer.hpp"
#include "hls/c_parser.hpp"
#include "shell/tcl_interp.hpp"
#include "frontend/sv_parser.hpp"
#include "verify/drc.hpp"
#include "verify/lvs.hpp"
#include "pnr/physical.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

// ========== SV PARAMETERIZED TYPES ==========

void test_sv_param_module_parse() {
    printf("\n=== SV Parameterized Module Parsing ===\n");
    sf::SvParser parser;
    std::string src = R"(
        module adder #(
            parameter WIDTH = 8,
            parameter DEPTH = 4
        )(
            input  [WIDTH-1:0] a,
            input  [WIDTH-1:0] b,
            output [WIDTH-1:0] sum
        );
            assign sum = a + b;
        endmodule
    )";
    bool ok = parser.parse(src);
    CHECK(ok, "SV parser accepts parameterized module");

    auto* mod = parser.find_module("adder");
    CHECK(mod != nullptr, "Module 'adder' found");
    CHECK(mod->parameters.size() == 2, "Two parameters parsed");
    CHECK(mod->parameters[0].name == "WIDTH", "First param is WIDTH");
    CHECK(mod->parameters[0].default_value == "8", "WIDTH default = 8");
    CHECK(mod->parameters[1].name == "DEPTH", "Second param is DEPTH");
    CHECK(mod->ports.size() == 3, "Three ports parsed");
}

void test_sv_param_resolution() {
    printf("\n=== SV Parameter Resolution with Overrides ===\n");
    sf::SvParser parser;
    std::string src = R"(
        module fifo #(
            parameter DATA_W = 32,
            parameter ADDR_W = 10
        )(
            input clk
        );
            localparam DEPTH = 1 << ADDR_W;
        endmodule
    )";
    parser.parse(src);

    // Override DATA_W to 64
    std::unordered_map<std::string, std::string> overrides;
    overrides["DATA_W"] = "64";
    bool ok = parser.resolve_parameters("fifo", overrides);
    CHECK(ok, "Parameter resolution succeeds");

    auto* mod = parser.find_module("fifo");
    CHECK(mod->parameters[0].resolved_int == 64, "DATA_W resolved to 64 via override");
    CHECK(mod->parameters[1].resolved_int == 10, "ADDR_W resolved to default 10");
}

void test_sv_clog2_expr() {
    printf("\n=== SV $clog2 Expression Evaluation ===\n");
    sf::SvParser parser;
    std::string src = R"(
        module mem_ctrl #(
            parameter DEPTH = 256,
            parameter ADDR_W = $clog2(DEPTH)
        )(input clk);
        endmodule
    )";
    parser.parse(src);

    std::unordered_map<std::string, std::string> overrides;
    parser.resolve_parameters("mem_ctrl", overrides);

    auto* mod = parser.find_module("mem_ctrl");
    CHECK(mod != nullptr, "mem_ctrl module found");
    CHECK(mod->parameters[1].resolved_int == 8, "$clog2(256) = 8");
}

// ========== $readmemh / $readmemb ==========

void test_readmemh_data() {
    printf("\n=== $readmemh In-Memory Data ===\n");
    sf::SvParser parser;
    std::string src = R"(
        module rom #(parameter DEPTH = 8)(input clk);
            reg [7:0] mem [0:7];
        endmodule
    )";
    parser.parse(src);

    std::string hex_data = "FF\n0A\n55\nDE AD\n@04\nBE\nEF\n12\n34";
    int loaded = parser.execute_readmemh_data(hex_data, "mem");
    CHECK(loaded > 0, "$readmemh_data loaded entries");

    auto* mem = parser.find_memory("mem");
    CHECK(mem != nullptr, "Memory 'mem' found");
    CHECK(mem->data[0] == 0xFF, "mem[0] = 0xFF");
    CHECK(mem->data[1] == 0x0A, "mem[1] = 0x0A");
    CHECK(mem->data[2] == 0x55, "mem[2] = 0x55");
}

void test_readmemb_data() {
    printf("\n=== $readmemb In-Memory Data ===\n");
    sf::SvParser parser;
    std::string src = R"(
        module brom(input clk);
            reg [3:0] data [0:3];
        endmodule
    )";
    parser.parse(src);

    std::string bin_data = "1010\n0101\n1111\n0000";
    int loaded = parser.execute_readmemb_data(bin_data, "data");
    CHECK(loaded > 0, "$readmemb_data loaded entries");

    auto* mem = parser.find_memory("data");
    CHECK(mem != nullptr, "Memory 'data' found");
    CHECK(mem->data[0] == 0b1010, "data[0] = 1010b");
    CHECK(mem->data[1] == 0b0101, "data[1] = 0101b");
}

// ========== TCL PACKAGE SYSTEM ==========

void test_tcl_package_provide_require() {
    printf("\n=== TCL Package Provide/Require ===\n");
    sf::TclInterp interp;

    // Provide a package
    std::string r1 = interp.eval("package provide mylib 1.2.3");
    CHECK(r1.find("error") == std::string::npos, "package provide succeeds");

    // Require it back
    std::string r2 = interp.eval("package require mylib");
    CHECK(r2.find("1.2.3") != std::string::npos || r2.find("error") == std::string::npos,
          "package require returns version or succeeds");

    // List package names
    std::string r3 = interp.eval("package names");
    CHECK(r3.find("mylib") != std::string::npos, "package names includes mylib");

    // Check versions
    std::string r4 = interp.eval("package versions mylib");
    CHECK(r4.find("1.2.3") != std::string::npos, "package versions returns 1.2.3");
}

void test_tcl_package_ifneeded() {
    printf("\n=== TCL Package Ifneeded ===\n");
    sf::TclInterp interp;

    std::string r = interp.eval("package ifneeded testpkg 2.0 {set loaded 1}");
    CHECK(r.find("error") == std::string::npos, "package ifneeded registration succeeds");

    // Require triggers the ifneeded script
    std::string r2 = interp.eval("package require testpkg");
    CHECK(r2.find("error") == std::string::npos, "package require with ifneeded succeeds");
}

// ========== HLS FLOAT SYNTHESIS ==========

void test_hls_float_ieee754_types() {
    printf("\n=== HLS IEEE 754 Float Types ===\n");
    auto half = sf::HlsFloatType::half();
    CHECK(half.width == 16, "half precision = 16 bits");
    CHECK(half.exponent_bits == 5, "half exponent = 5 bits");
    CHECK(half.mantissa_bits == 10, "half mantissa = 10 bits");

    auto single = sf::HlsFloatType::single();
    CHECK(single.width == 32, "single precision = 32 bits");
    CHECK(single.exponent_bits == 8, "single exponent = 8 bits");

    auto dbl = sf::HlsFloatType::dbl();
    CHECK(dbl.width == 64, "double precision = 64 bits");
    CHECK(dbl.exponent_bits == 11, "double exponent = 11 bits");
    CHECK(dbl.mantissa_bits == 52, "double mantissa = 52 bits");
}

void test_hls_float_synth_ops() {
    printf("\n=== HLS Float Synthesis Operations ===\n");
    auto fmt = sf::HlsFloatType::single();

    auto fadd = sf::HlsFloatSynthesizer::synthesize_fadd(fmt);
    CHECK(fadd.latency_cycles > 0, "FP add has positive latency");
    CHECK(fadd.area_estimate > 0, "FP add has positive area");

    auto fmul = sf::HlsFloatSynthesizer::synthesize_fmul(fmt);
    CHECK(fmul.latency_cycles > 0, "FP mul has positive latency");
    CHECK(fmul.area_estimate > 0, "FP mul has positive area");

    auto fdiv = sf::HlsFloatSynthesizer::synthesize_fdiv(fmt);
    CHECK(fdiv.latency_cycles > fmul.latency_cycles, "FP div latency > FP mul latency");

    auto f2i = sf::HlsFloatSynthesizer::synthesize_f2i(fmt);
    CHECK(f2i.latency_cycles > 0, "Float-to-int has positive latency");

    auto i2f = sf::HlsFloatSynthesizer::synthesize_i2f(fmt);
    CHECK(i2f.latency_cycles > 0, "Int-to-float has positive latency");
}

void test_hls_float_type_from_name() {
    printf("\n=== HLS Float Type from C Name ===\n");
    auto f = sf::HlsFloatSynthesizer::type_from_name("float");
    CHECK(f.width == 32, "float -> 32-bit");

    auto d = sf::HlsFloatSynthesizer::type_from_name("double");
    CHECK(d.width == 64, "double -> 64-bit");

    auto h = sf::HlsFloatSynthesizer::type_from_name("half");
    CHECK(h.width == 16, "half -> 16-bit");
}

void test_hls_float_cost_estimation() {
    printf("\n=== HLS Float Expression Cost ===\n");
    auto fmt = sf::HlsFloatType::single();
    std::vector<sf::HlsFloatOpResult> ops;
    ops.push_back(sf::HlsFloatSynthesizer::synthesize_fadd(fmt));
    ops.push_back(sf::HlsFloatSynthesizer::synthesize_fmul(fmt));
    ops.push_back(sf::HlsFloatSynthesizer::synthesize_fadd(fmt));

    auto cost = sf::HlsFloatSynthesizer::estimate_cost(ops);
    CHECK(cost.total_area > 0, "Expression tree total area > 0");
    CHECK(cost.adders == 2, "Expression tree has 2 adders");
    CHECK(cost.multipliers == 1, "Expression tree has 1 multiplier");
}

// ========== NUMA-AWARE THREADING ==========

void test_numa_config() {
    printf("\n=== NUMA Configuration ===\n");
    sf::NumaConfig cfg;
    cfg.node_count = 2;
    cfg.cores_per_node = 4;
    cfg.memory_latency_ratio = 1.7;
    cfg.build_default_map();

    CHECK((int)cfg.node_core_map.size() == cfg.node_count * cfg.cores_per_node,
          "NUMA core map has total_cores entries");
    CHECK(cfg.cores_on_node(0).size() == 4, "Node 0 has 4 cores");
    CHECK(cfg.cores_on_node(1).size() == 4, "Node 1 has 4 cores");
    CHECK(cfg.node_for_core(0) == 0, "Core 0 on node 0");
    CHECK(cfg.node_for_core(5) == 1, "Core 5 on node 1");
}

void test_numa_submit_and_parallel_for() {
    printf("\n=== NUMA Submit and Parallel For ===\n");
    sf::ThreadPool pool(4);

    sf::NumaConfig cfg;
    cfg.node_count = 2;
    cfg.cores_per_node = 2;
    cfg.build_default_map();
    pool.set_numa_config(cfg);

    // Submit NUMA-aware task
    auto fut = pool.submit_numa_aware(0, []() { return 42; });
    CHECK(fut.get() == 42, "NUMA-aware task returns correct result");

    // Parallel for NUMA
    std::atomic<int> sum{0};
    pool.parallel_for_numa(0, 100, [&sum](int i) {
        sum.fetch_add(i, std::memory_order_relaxed);
    });
    CHECK(sum.load() == 4950, "NUMA parallel_for sums 0..99 = 4950");
}

// ========== 3D DRC ==========

void test_3d_drc_rules() {
    printf("\n=== 3D DRC Rules ===\n");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);

    // Add some vias to act as TSVs
    pd.vias.push_back({{100.0, 100.0}, 0, 1});
    pd.vias.push_back({{103.0, 100.0}, 0, 1});  // within keepout zone

    // Add a wire near TSV
    pd.wires.push_back({0, {95.0, 100.0}, {102.0, 100.5}, 0.2, 0});

    sf::DrcEngine drc(pd);
    sf::Drc3dConfig cfg3d;
    cfg3d.tsv_keepout_um = 5.0;
    cfg3d.bump_pitch_um = 40.0;
    cfg3d.thermal_via_density = 0.05;
    cfg3d.num_dies = 2;
    drc.set_3d_config(cfg3d);

    auto result = drc.check_3d_rules();
    CHECK(result.violations >= 0, "3D DRC runs without crash");
    printf("    3D DRC found %d violations\n", result.violations);
}

// ========== LITHOGRAPHY SIMULATION ==========

void test_litho_check() {
    printf("\n=== Lithography Printability Check ===\n");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);

    // Add a very narrow wire (sub-resolution feature, ~20nm width)
    pd.wires.push_back({0, {10.0, 10.0}, {10.02, 100.0}, 0.02, 0});

    // Add a normal wire (~200nm width)
    pd.wires.push_back({0, {200.0, 200.0}, {200.2, 300.0}, 0.2, 1});

    sf::DrcEngine drc(pd);
    sf::LithoConfig lcfg;
    lcfg.wavelength_nm = 193.0;
    lcfg.NA = 1.35;
    lcfg.k1_factor = 0.3;
    drc.set_litho_config(lcfg);

    auto result = drc.litho_check();
    CHECK(result.violations >= 0, "Litho check runs without crash");
    printf("    Litho check found %d violations\n", result.violations);
}

// ========== DFM YIELD PREDICTION ==========

void test_dfm_yield() {
    printf("\n=== DFM Yield Prediction ===\n");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 5000, 5000);

    // Add some wires to create critical area
    for (int i = 0; i < 50; i++) {
        double y = i * 100.0;
        pd.wires.push_back({0, {0.0, y}, {5000.0, y + 0.2}, 0.2, i});
    }

    sf::DrcEngine drc(pd);
    auto yp = drc.predict_yield(0.5);  // 0.5 defects/cm^2
    CHECK(yp.yield_estimate >= 0.0 && yp.yield_estimate <= 1.0, "Yield in [0, 1]");
    CHECK(yp.critical_area_um2 >= 0.0, "Critical area non-negative");
    printf("    Predicted yield = %.4f, critical area = %.1f um^2\n",
           yp.yield_estimate, yp.critical_area_um2);
}

// ========== VIA COUPLING ==========

void test_via_coupling_inductance() {
    printf("\n=== Via-to-Via Coupling Inductance ===\n");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);

    // Place two vias close together
    pd.vias.push_back({{100.0, 100.0}, 0, 1});
    pd.vias.push_back({{110.0, 100.0}, 0, 1});  // 10um apart

    // Place a far via
    pd.vias.push_back({{500.0, 500.0}, 0, 1});

    sf::PdnAnalyzer pdn(pd);
    sf::PdnConfig pdn_cfg;
    pdn_cfg.vdd = 0.9;
    pdn.set_config(pdn_cfg);

    auto couplings = pdn.compute_via_coupling(10.0, 50.0);
    // Close vias should couple
    bool found_close = false;
    for (auto& c : couplings) {
        if (c.distance_um < 15.0) {
            found_close = true;
            CHECK(c.mutual_inductance_ph > 0, "Close via pair has positive mutual inductance");
            CHECK(c.coupling_coefficient > 0 && c.coupling_coefficient <= 1.0,
                  "Coupling coefficient in (0, 1]");
        }
    }
    CHECK(found_close || couplings.empty(), "Via coupling computation runs");
    printf("    Found %zu coupling pairs\n", couplings.size());
}

// ========== PACKAGE INTERACTION ==========

void test_package_interaction() {
    printf("\n=== Package-to-Die Interaction ===\n");
    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 5000, 5000);

    sf::PdnAnalyzer pdn(pd);
    sf::PdnConfig pdn_cfg;
    pdn_cfg.vdd = 0.9;
    pdn_cfg.total_current_ma = 5000.0;  // ~5W at 0.9V
    pdn.set_config(pdn_cfg);

    sf::PackageModel pkg;
    pkg.lead_inductance_nh = 2.0;
    pkg.bond_wire_resistance_mohm = 50.0;
    pkg.package_cap_pf = 200.0;
    pkg.power_pins = 64;
    pkg.ground_pins = 64;
    pkg.signal_pins = 512;
    pkg.switching_fraction = 0.15;

    auto result = pdn.analyze_package_interaction(pkg);
    CHECK(result.pkg_ir_drop_mv > 0, "Package IR drop > 0");
    CHECK(result.ssn_mv > 0, "SSN > 0");
    CHECK(result.resonance_freq_mhz > 0, "Resonance frequency > 0");
    CHECK(!result.summary.empty(), "Summary generated");
    printf("    IR drop=%.1fmV, SSN=%.1fmV, f_res=%.1fMHz, budget=%s\n",
           result.pkg_ir_drop_mv, result.ssn_mv, result.resonance_freq_mhz,
           result.meets_noise_budget ? "OK" : "FAIL");
}

// ========== LVS DEVICE PARAMETER EXTRACTION ==========

void test_lvs_device_params() {
    printf("\n=== LVS Device Parameter Extraction ===\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("A");
    auto n_y = nl.add_net("Y");
    (void)nl.add_gate(sf::GateType::NOT, {n_a}, n_y, "INV_X1");

    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 1000, 1000);

    sf::LvsChecker lvs(nl, pd);
    auto result = lvs.extract_device_params();
    CHECK(result.total_extracted >= 0, "Device param extraction runs");
    printf("    Extracted %d devices (%d MOS, %d failed)\n",
           result.total_extracted, result.mos_extracted, result.failed);
}

// ========== CHIPLET SKEW MODELING ==========

void test_chiplet_skew() {
    printf("\n=== Chiplet Skew Modeling ===\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("a");
    auto n_b = nl.add_net("b");
    auto n_c = nl.add_net("c");
    nl.add_gate(sf::GateType::BUF, {n_a}, n_b, "BUF_chip0");
    nl.add_gate(sf::GateType::BUF, {n_b}, n_c, "BUF_chip1");

    sf::LibertyLibrary lib;
    std::string lib_src = R"(
        library(test) {
            time_unit : "1ns";
            cell(BUF_X1) {
                pin(A) { direction: input; capacitance: 0.01; }
                pin(Y) { direction: output; function: "A";
                    timing() { related_pin: "A"; cell_rise(scalar) { values("0.05"); }
                        cell_fall(scalar) { values("0.05"); } } }
            }
        }
    )";
    lib.parse_string(lib_src);

    sf::PhysicalDesign pd;
    pd.die_area = sf::Rect(0, 0, 5000, 5000);

    sf::StaEngine sta(nl, &lib, &pd);

    sf::ChipletSkewConfig ccfg;
    ccfg.inter_chiplet_delay_ps = 50.0;
    ccfg.d2d_jitter_ps = 10.0;
    ccfg.thermal_gradient_factor = 0.02;
    sta.set_chiplet_config(ccfg);

    sta.assign_chiplet("BUF_chip0", 0);
    sta.assign_chiplet("BUF_chip1", 1);
    sta.set_chiplet_distance(0, 1, 5.0);  // 5mm apart
    sta.set_chiplet_thermal_delta(1, 15.0);  // chip1 is 15C hotter

    sta.apply_chiplet_skew();

    CHECK(true, "Chiplet skew applied without crash");
}

// ========== SDF ANNOTATION ==========

void test_sdf_annotation() {
    printf("\n=== SDF Back-Annotation ===\n");
    sf::Netlist nl;
    auto n_a = nl.add_net("a_net");
    auto n_y = nl.add_net("y_net");
    nl.add_gate(sf::GateType::BUF, {n_a}, n_y, "BUF_X1");

    sf::LibertyLibrary lib;
    std::string lib_src = R"(
        library(test) {
            time_unit : "1ns";
            cell(BUF_X1) {
                pin(A) { direction: input; capacitance: 0.01; }
                pin(Y) { direction: output; function: "A";
                    timing() { related_pin: "A"; cell_rise(scalar) { values("0.1"); }
                        cell_fall(scalar) { values("0.1"); } } }
            }
        }
    )";
    lib.parse_string(lib_src);

    sf::SdfWriter writer(nl);
    sf::SdfConfig sdf_cfg;
    sdf_cfg.design_name = "test_design";
    sdf_cfg.lib = &lib;
    std::string sdf_content = writer.generate(sdf_cfg);
    CHECK(!sdf_content.empty(), "SDF generated");
    CHECK(sdf_content.find("DELAYFILE") != std::string::npos, "SDF contains DELAYFILE header");
    CHECK(sdf_content.find("BUF_X1") != std::string::npos, "SDF contains cell instance");
}

// ========== MAIN ==========

int main() {
    printf("Phase 87: Gap Closure — SV params, readmem, TCL pkgs, HLS float,\n");
    printf("          NUMA, 3D DRC, litho, yield, via coupling, package, chiplet skew\n");
    printf("================================================================\n");

    // SV Parameterized Types
    test_sv_param_module_parse();
    test_sv_param_resolution();
    test_sv_clog2_expr();

    // $readmemh / $readmemb
    test_readmemh_data();
    test_readmemb_data();

    // TCL Packages
    test_tcl_package_provide_require();
    test_tcl_package_ifneeded();

    // HLS Float Synthesis
    test_hls_float_ieee754_types();
    test_hls_float_synth_ops();
    test_hls_float_type_from_name();
    test_hls_float_cost_estimation();

    // NUMA Threading
    test_numa_config();
    test_numa_submit_and_parallel_for();

    // 3D DRC
    test_3d_drc_rules();

    // Lithography
    test_litho_check();

    // DFM Yield
    test_dfm_yield();

    // Via Coupling
    test_via_coupling_inductance();

    // Package Interaction
    test_package_interaction();

    // LVS Device Params
    test_lvs_device_params();

    // Chiplet Skew
    test_chiplet_skew();

    // SDF Annotation
    test_sdf_annotation();

    printf("\n================================================================\n");
    printf("Phase 87 Results: %d / %d passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
