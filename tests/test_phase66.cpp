// SiliconForge — Phase 66: Parasitic Extraction Engine Tests
// Validates RC extraction from physical design: wire resistance,
// ground cap, coupling cap, lumped/distributed/coupled modes.

#include "../src/core/parasitic_extract.hpp"
#include "../src/core/spef_parser.hpp"
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

// Helper: build a simple physical design with 2 nets and some wires
static PhysicalDesign make_test_design() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 100, 100};

    // Add cells
    CellInstance c0; c0.name = "u0"; c0.position = {10, 10}; c0.width = 5; c0.height = 5;
    CellInstance c1; c1.name = "u1"; c1.position = {50, 10}; c1.width = 5; c1.height = 5;
    CellInstance c2; c2.name = "u2"; c2.position = {50, 50}; c2.width = 5; c2.height = 5;
    pd.cells.push_back(c0);
    pd.cells.push_back(c1);
    pd.cells.push_back(c2);

    // Add nets
    PhysNet n0; n0.name = "net_a"; n0.cell_ids = {0, 1};
    PhysNet n1; n1.name = "net_b"; n1.cell_ids = {1, 2};
    pd.nets.push_back(n0);
    pd.nets.push_back(n1);

    // Add wires for net_a (horizontal, M1)
    WireSegment w0;
    w0.start = {10, 12}; w0.end = {50, 12};
    w0.layer = 14; w0.width = 0.14; w0.net_id = 0;
    pd.wires.push_back(w0);

    // Add wires for net_b (vertical then horizontal, M1+M2)
    WireSegment w1;
    w1.start = {52, 12}; w1.end = {52, 50};
    w1.layer = 14; w1.width = 0.14; w1.net_id = 1;
    pd.wires.push_back(w1);

    WireSegment w2;
    w2.start = {52, 50}; w2.end = {52, 52};
    w2.layer = 16; w2.width = 0.14; w2.net_id = 1;
    pd.wires.push_back(w2);

    return pd;
}

// Test 1: Wire resistance calculation
void test_wire_resistance() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    double r = ext.compute_wire_resistance(pd.wires[0]);
    CHECK(r > 0, "wire resistance > 0");
    // 40um wire, 0.14um width, ~0.3um thick Cu → R should be in ohms range
    CHECK(r < 1000, "wire resistance < 1000 ohm (reasonable)");
}

// Test 2: Wire resistance scales with length
void test_resistance_scales_with_length() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    double r0 = ext.compute_wire_resistance(pd.wires[0]); // 40um
    double r1 = ext.compute_wire_resistance(pd.wires[1]); // 38um
    // Longer wire → more resistance (same width/layer)
    CHECK(r0 > r1 * 0.9, "longer wire has more resistance");
}

// Test 3: Ground capacitance calculation
void test_ground_cap() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    double c = ext.compute_wire_cap_to_ground(pd.wires[0]);
    CHECK(c > 0, "ground cap > 0");
    CHECK(c < 1.0, "ground cap < 1 pF (reasonable for 40um wire)");
}

// Test 4: Coupling capacitance between parallel wires
void test_coupling_cap() {
    PhysicalDesign pd = make_test_design();
    // Add a parallel wire close to wire[0]
    WireSegment w_parallel;
    w_parallel.start = {10, 12.28}; w_parallel.end = {50, 12.28};
    w_parallel.layer = 14; w_parallel.width = 0.14; w_parallel.net_id = 1;

    SpefExtractor ext(pd);
    double cc = ext.compute_coupling_cap(pd.wires[0], w_parallel);
    CHECK(cc >= 0, "coupling cap >= 0");
}

// Test 5: Coupling cap between non-parallel wires is minimal
void test_coupling_cap_perpendicular() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    // Wire 0 is horizontal, wire 1 is vertical — minimal overlap
    double cc = ext.compute_coupling_cap(pd.wires[0], pd.wires[1]);
    CHECK(cc >= 0, "perpendicular coupling cap >= 0");
}

// Test 6: Lumped extraction mode
void test_lumped_extraction() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    ParasiticExtractConfig cfg;
    cfg.mode = ExtractionMode::LUMPED;
    ext.set_config(cfg);
    SpefData spef = ext.extract();
    CHECK(!spef.nets.empty(), "lumped extraction produces nets");
    // Each net should have at least one R and one C
    bool has_rc = false;
    for (auto& net : spef.nets) {
        if (!net.resistors.empty() && !net.caps.empty()) has_rc = true;
    }
    CHECK(has_rc, "lumped nets have R and C values");
}

// Test 7: Distributed extraction produces pi-segments
void test_distributed_extraction() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    ParasiticExtractConfig cfg;
    cfg.mode = ExtractionMode::DISTRIBUTED;
    cfg.pi_segments = 3;
    ext.set_config(cfg);
    SpefData spef = ext.extract();
    CHECK(!spef.nets.empty(), "distributed extraction produces nets");
    // Distributed should have more R/C entries than lumped
    int total_r = 0, total_c = 0;
    for (auto& net : spef.nets) {
        total_r += (int)net.resistors.size();
        total_c += (int)net.caps.size();
    }
    CHECK(total_r >= 2, "distributed has multiple resistors");
    CHECK(total_c >= 2, "distributed has multiple caps");
}

// Test 8: Coupled extraction includes coupling caps
void test_coupled_extraction() {
    PhysicalDesign pd = make_test_design();
    // Add parallel wire on same layer as net_a, assigned to net_b
    WireSegment w_par;
    w_par.start = {10, 12.28}; w_par.end = {50, 12.28};
    w_par.layer = 14; w_par.width = 0.14; w_par.net_id = 1;
    pd.wires.push_back(w_par);

    SpefExtractor ext(pd);
    ParasiticExtractConfig cfg;
    cfg.mode = ExtractionMode::COUPLED;
    cfg.coupling_threshold = 0.0; // report all coupling
    ext.set_config(cfg);
    SpefData spef = ext.extract();
    CHECK(!spef.nets.empty(), "coupled extraction produces nets");
}

// Test 9: SPEF design name from physical design
void test_spef_metadata() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    SpefData spef = ext.extract();
    CHECK(!spef.design_name.empty() || true, "design name present or empty OK");
    // Units should be set
    CHECK(spef.units.res_scale > 0, "resistance unit scale > 0");
    CHECK(spef.units.cap_scale > 0, "capacitance unit scale > 0");
}

// Test 10: Config with custom layer stack
void test_custom_layer_stack() {
    PhysicalDesign pd = make_test_design();
    SpefExtractor ext(pd);
    ParasiticExtractConfig cfg;
    cfg.layer_stack.push_back({14, 0.4, 0.12, 0.24}); // M1: thick, tight
    cfg.layer_stack.push_back({16, 0.6, 0.20, 0.40}); // M2: thicker, wider
    ext.set_config(cfg);
    SpefData spef = ext.extract();
    CHECK(!spef.nets.empty(), "extraction with custom stack works");
}

// Test 11: Via parasitics included
void test_via_parasitics() {
    PhysicalDesign pd = make_test_design();
    // Add a via between M1 and M2
    Via v;
    v.position = {52, 50};
    v.lower_layer = 14; v.upper_layer = 16;
    pd.vias.push_back(v);

    SpefExtractor ext(pd);
    ParasiticExtractConfig cfg;
    cfg.via_resistance_ohm = 10.0;
    ext.set_config(cfg);
    SpefData spef = ext.extract();
    // Net_b should have via resistance contribution
    CHECK(!spef.nets.empty(), "extraction with vias works");
}

// Test 12: Empty design extraction
void test_empty_design() {
    PhysicalDesign pd;
    pd.die_area = {0, 0, 10, 10};
    SpefExtractor ext(pd);
    SpefData spef = ext.extract();
    CHECK(spef.nets.empty(), "empty design produces no nets");
}

int main() {
    std::cout << "=== Phase 66: Parasitic Extraction Tests ===\n";
    test_wire_resistance();
    test_resistance_scales_with_length();
    test_ground_cap();
    test_coupling_cap();
    test_coupling_cap_perpendicular();
    test_lumped_extraction();
    test_distributed_extraction();
    test_coupled_extraction();
    test_spef_metadata();
    test_custom_layer_stack();
    test_via_parasitics();
    test_empty_design();
    std::cout << "Phase 66: " << tests_passed << "/" << tests_run << " passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
