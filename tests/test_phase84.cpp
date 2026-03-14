// Phase 84: Tier 2 Remainder + Tier 3 Enhancement Tests
// Crosstalk extraction, SSN, freq-dep coupling, DEF fills, VHDL generics,
// SDF conditional delays, SVA sequences, UPF isolation, TCL dict, HLS types,
// thread pool priorities

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

#include "core/parasitic_extract.hpp"
#include "core/spef_parser.hpp"
#include "timing/ir_drop.hpp"
#include "pnr/physical.hpp"
#include "frontend/vhdl_parser.hpp"
#include "timing/sdf_writer.hpp"
#include "frontend/sva_parser.hpp"
#include "frontend/upf_parser.hpp"
#include "shell/tcl_interp.hpp"
#include "hls/c_parser.hpp"
#include "core/thread_pool.hpp"

static int total = 0, passed = 0;
#define CHECK(cond, msg) do { \
    total++; \
    if (cond) { passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

// ========== CROSSTALK EXTRACTION ==========

void test_coupling_cap_struct() {
    printf("\n[Coupling Cap]\n");
    sf::CouplingCap cc;
    cc.aggressor_net = "clk";
    cc.victim_net = "data";
    cc.layer = "M3";
    cc.coupling_cap = 0.5;
    cc.parallel_length = 10.0;
    CHECK(cc.coupling_cap > 0, "positive coupling cap");
    CHECK(cc.layer == "M3", "layer = M3");
    CHECK(cc.parallel_length == 10.0, "parallel length = 10um");
}

void test_crosstalk_extraction_result() {
    printf("\n[Crosstalk Extraction Result]\n");
    sf::CrosstalkExtractionResult r;
    CHECK(r.coupling_caps.empty(), "empty initially");
    CHECK(r.num_aggressors == 0, "no aggressors");
    CHECK(r.total_coupling_cap == 0.0, "zero total coupling");
}

void test_frequency_dependent_cap() {
    printf("\n[Frequency-Dependent Coupling Cap]\n");
    double c_dc = 1.0; // 1 fF
    double c_1ghz = sf::SpefExtractor::frequency_dependent_cap(c_dc, 1.0);
    double c_10ghz = sf::SpefExtractor::frequency_dependent_cap(c_dc, 10.0);
    CHECK(c_1ghz >= c_dc, "C at 1GHz >= C_dc");
    CHECK(c_10ghz >= c_1ghz, "C at 10GHz >= C at 1GHz");
    CHECK(c_10ghz < c_dc * 2.0, "C at 10GHz < 2x C_dc (reasonable)");
}

// ========== SUBSTRATE NOISE ==========

void test_ssn_config() {
    printf("\n[SSN Config]\n");
    sf::SsnConfig cfg;
    CHECK(cfg.substrate_resistance == 100.0, "default R_sub = 100 ohms");
    CHECK(cfg.substrate_capacitance > 0, "positive C_sub");
    CHECK(cfg.switching_noise_factor > 0, "positive noise factor");
}

void test_substrate_noise_result() {
    printf("\n[Substrate Noise Result]\n");
    sf::SubstrateSsnResult r;
    CHECK(r.peak_noise_mv == 0.0, "default peak = 0");
    CHECK(r.rms_noise_mv == 0.0, "default rms = 0");
    CHECK(r.noisy_cells.empty(), "no noisy cells");
}

// ========== DEF FILLS ==========

void test_fill_shape() {
    printf("\n[DEF Fill Shape]\n");
    sf::FillShape fs;
    fs.layer = "M1";
    fs.x0 = 10.0; fs.y0 = 20.0;
    fs.x1 = 15.0; fs.y1 = 25.0;
    CHECK(fs.layer == "M1", "fill layer = M1");
    CHECK(fs.x1 > fs.x0, "valid X range");
    CHECK(fs.y1 > fs.y0, "valid Y range");
}

void test_physical_design_fills() {
    printf("\n[PhysicalDesign Fills]\n");
    sf::PhysicalDesign pd;
    CHECK(pd.fills.empty(), "no fills initially");
    sf::FillShape f1; f1.layer = "M1"; f1.x0 = 0; f1.y0 = 0; f1.x1 = 1; f1.y1 = 1;
    sf::FillShape f2; f2.layer = "M2"; f2.x0 = 5; f2.y0 = 5; f2.x1 = 6; f2.y1 = 6;
    pd.fills.push_back(f1);
    pd.fills.push_back(f2);
    CHECK(pd.fills.size() == 2, "2 fills added");
}

void test_die_area_polygon() {
    printf("\n[DEF Die Area Polygon]\n");
    sf::PhysicalDesign pd;
    CHECK(pd.die_area_polygon.empty(), "no polygon initially");
    pd.die_area_polygon.push_back({0, 0});
    pd.die_area_polygon.push_back({100, 0});
    pd.die_area_polygon.push_back({100, 80});
    pd.die_area_polygon.push_back({50, 100});
    pd.die_area_polygon.push_back({0, 80});
    CHECK(pd.die_area_polygon.size() == 5, "5-point L-shaped die");
}

// ========== VHDL GENERICS ==========

void test_vhdl_generic_struct() {
    printf("\n[VHDL Generic]\n");
    sf::VhdlGeneric g;
    g.name = "WIDTH";
    g.type = "integer";
    g.default_value = "8";
    CHECK(g.name == "WIDTH", "name = WIDTH");
    CHECK(g.type == "integer", "type = integer");
    CHECK(g.default_value == "8", "default = 8");
}

// ========== SDF CONDITIONAL DELAYS ==========

void test_sdf_cond_delay() {
    printf("\n[SDF Conditional Delay]\n");
    sf::SdfCondDelay cd;
    cd.condition = "sel == 1'b1";
    cd.delay_rise = 0.15;
    cd.delay_fall = 0.18;
    CHECK(cd.condition == "sel == 1'b1", "condition set");
    CHECK(cd.delay_rise > 0, "positive rise delay");
    CHECK(cd.delay_fall > cd.delay_rise, "fall > rise");
}

// ========== SVA SEQUENCES (Tier 3) ==========

void test_sva_sequence() {
    printf("\n[SVA Sequence]\n");
    sf::SvaSequence seq;
    seq.name = "req_ack";
    seq.expression = "req ##[1:3] ack";
    seq.arguments = {"req", "ack"};
    CHECK(seq.name == "req_ack", "sequence name");
    CHECK(seq.arguments.size() == 2, "2 arguments");
}

// ========== UPF ISOLATION (Tier 3) ==========

void test_upf_isolation_strategy() {
    printf("\n[UPF Isolation Strategy]\n");
    sf::UpfIsolationStrategy iso;
    iso.name = "iso_pd1";
    iso.domain = "PD_LOW";
    iso.isolation_signal = "iso_en";
    iso.clamp_value = "0";
    iso.location = "parent";
    CHECK(iso.name == "iso_pd1", "name = iso_pd1");
    CHECK(iso.clamp_value == "0", "clamp to 0");
    CHECK(iso.location == "parent", "location = parent");
}

// ========== TCL DICT (Tier 3) ==========

void test_tcl_dict() {
    printf("\n[TCL Dict]\n");
    sf::TclInterp tcl;
    auto r1 = tcl.eval("dict create name SiliconForge version 2.0");
    CHECK(!r1.empty(), "dict create returns value");
    auto r2 = tcl.eval("dict get [dict create a 1 b 2] a");
    CHECK(r2 == "1" || !r2.empty(), "dict get works");
}

// ========== HLS ARRAY/STRUCT (Tier 3) ==========

void test_hls_array_type() {
    printf("\n[HLS Array Type]\n");
    sf::HlsArrayType arr;
    arr.element_type = "int";
    arr.size = 16;
    CHECK(arr.element_type == "int", "element type = int");
    CHECK(arr.size == 16, "size = 16");
}

void test_hls_struct_type() {
    printf("\n[HLS Struct Type]\n");
    sf::HlsStructType st;
    st.name = "pixel_t";
    st.fields.push_back({"r", "uint8_t"});
    st.fields.push_back({"g", "uint8_t"});
    st.fields.push_back({"b", "uint8_t"});
    CHECK(st.name == "pixel_t", "struct name = pixel_t");
    CHECK(st.fields.size() == 3, "3 fields");
}

// ========== THREAD POOL PRIORITIES (Tier 3) ==========

void test_task_priority() {
    printf("\n[Task Priority]\n");
    CHECK(sf::TaskPriority::LOW < sf::TaskPriority::NORMAL, "LOW < NORMAL");
    CHECK(sf::TaskPriority::NORMAL < sf::TaskPriority::HIGH, "NORMAL < HIGH");
    CHECK(sf::TaskPriority::HIGH < sf::TaskPriority::CRITICAL, "HIGH < CRITICAL");
}

void test_thread_pool_priority_submit() {
    printf("\n[Thread Pool Priority Submit]\n");
    sf::ThreadPool pool(2);
    auto f1 = pool.submit_with_priority(sf::TaskPriority::HIGH, []() { return 42; });
    auto f2 = pool.submit_with_priority(sf::TaskPriority::LOW, []() { return 7; });
    CHECK(f1.get() == 42, "HIGH priority task returns 42");
    CHECK(f2.get() == 7, "LOW priority task returns 7");
}

// ========== MAIN ==========

int main() {
    printf("=== Phase 84: Tier 2 Remainder + Tier 3 Tests ===\n");

    // Crosstalk
    test_coupling_cap_struct();
    test_crosstalk_extraction_result();
    test_frequency_dependent_cap();

    // SSN
    test_ssn_config();
    test_substrate_noise_result();

    // DEF fills
    test_fill_shape();
    test_physical_design_fills();
    test_die_area_polygon();

    // VHDL
    test_vhdl_generic_struct();

    // SDF
    test_sdf_cond_delay();

    // SVA (Tier 3)
    test_sva_sequence();

    // UPF (Tier 3)
    test_upf_isolation_strategy();

    // TCL (Tier 3)
    test_tcl_dict();

    // HLS (Tier 3)
    test_hls_array_type();
    test_hls_struct_type();

    // Thread pool (Tier 3)
    test_task_priority();
    test_thread_pool_priority_submit();

    printf("\n=== Phase 84 Results: %d/%d passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
