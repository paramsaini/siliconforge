// SiliconForge — Phase 63: SPEF/DSPF Parasitic Parser Tests
// Validates IEEE 1481 SPEF parsing: header, name map, net parasitics,
// lumped/distributed RC, coupling caps, coordinate extraction.

#include "../src/core/spef_parser.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;
#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [" << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

static constexpr double EPS = 1e-12;

// Minimal SPEF content for testing
static const char* BASIC_SPEF = R"(
*SPEF "IEEE 1481-2009"
*DESIGN "test_chip"
*T_UNIT 1 NS
*C_UNIT 1 PF
*R_UNIT 1 OHM
*L_UNIT 1 NH

*NAME_MAP
*1 clk
*2 data_in
*3 data_out

*D_NET *1 0.025
*CONN
*P *1:I I *C 0.0 0.0
*I inst0:A I *C 10.5 20.3
*CAP
1 *1:I 0.010
2 inst0:A 0.015
*RES
1 *1:I inst0:A 5.5
*END

*D_NET *2 0.044
*CONN
*P *2:I I *C 0.0 5.0
*I inst1:B I *C 30.0 40.0
*I inst2:C I *C 50.0 60.0
*CAP
1 *2:I 0.012
2 inst1:B 0.018
3 inst2:C 0.014
*RES
1 *2:I inst1:B 3.2
2 inst1:B inst2:C 4.1
*END

*D_NET *3 0.031
*CONN
*P *3:O O *C 100.0 200.0
*I inst3:Q O *C 80.0 90.0
*CAP
1 *3:O 0.016
2 inst3:Q 0.015
*RES
1 inst3:Q *3:O 6.8
*END
)";

static const char* COUPLING_CAP_SPEF = R"(
*SPEF "IEEE 1481-2009"
*DESIGN "coupling_test"
*T_UNIT 1 PS
*C_UNIT 1 FF
*R_UNIT 1 KOHM
*L_UNIT 1 NH

*D_NET net_a 25.0
*CONN
*I u0:Z O
*I u1:A I
*CAP
1 u0:Z 10.0
2 u1:A 8.0
3 u0:Z net_b:u2:A 7.0
*RES
1 u0:Z u1:A 1.5
*END
)";

// Test 1: Parse basic SPEF header
void test_header_parse() {
    SpefParser parser;
    bool ok = parser.parse_string(BASIC_SPEF);
    CHECK(ok, "parse_string returns true");
    CHECK(parser.data().design_name == "test_chip", "design name parsed");
    CHECK(std::abs(parser.data().units.time_scale - 1e-9) < EPS, "time unit 1 NS");
    CHECK(std::abs(parser.data().units.cap_scale - 1e-12) < EPS, "cap unit 1 PF");
    CHECK(std::abs(parser.data().units.res_scale - 1.0) < EPS, "res unit 1 OHM");
}

// Test 2: Name map resolution
void test_name_map() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    auto& d = parser.data();
    // Net names should be resolved from *1/*2/*3 to clk/data_in/data_out
    CHECK(d.nets.size() == 3, "3 nets parsed");
    CHECK(d.nets[0].name == "clk", "net *1 resolved to clk");
    CHECK(d.nets[1].name == "data_in", "net *2 resolved to data_in");
    CHECK(d.nets[2].name == "data_out", "net *3 resolved to data_out");
}

// Test 3: Net parasitics — total cap
void test_net_total_cap() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    auto& d = parser.data();
    CHECK(std::abs(d.nets[0].total_cap - 0.025) < 1e-6, "clk total_cap=0.025");
    CHECK(std::abs(d.nets[1].total_cap - 0.044) < 1e-6, "data_in total_cap=0.044");
    CHECK(std::abs(d.nets[2].total_cap - 0.031) < 1e-6, "data_out total_cap=0.031");
}

// Test 4: Pin parasitics and coordinates
void test_pin_parasitics() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    auto& net0 = parser.data().nets[0]; // clk
    CHECK(net0.pins.size() == 2, "clk has 2 pins");
    // Second pin should be inst0:A at (10.5, 20.3)
    bool found = false;
    for (auto& p : net0.pins) {
        if (p.pin_name.find("inst0") != std::string::npos) {
            CHECK(std::abs(p.x - 10.5) < 1e-3, "inst0:A x=10.5");
            CHECK(std::abs(p.y - 20.3) < 1e-3, "inst0:A y=20.3");
            found = true;
        }
    }
    CHECK(found, "inst0:A pin found in clk net");
}

// Test 5: Resistor network
void test_resistors() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    auto& net1 = parser.data().nets[1]; // data_in
    CHECK(net1.resistors.size() == 2, "data_in has 2 resistors");
    CHECK(std::abs(net1.resistors[0].value - 3.2) < 1e-6, "R1=3.2 ohm");
    CHECK(std::abs(net1.resistors[1].value - 4.1) < 1e-6, "R2=4.1 ohm");
}

// Test 6: Capacitor values
void test_capacitors() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    auto& net0 = parser.data().nets[0];
    CHECK(net0.caps.size() == 2, "clk has 2 caps");
    CHECK(std::abs(net0.caps[0].value - 0.010) < 1e-6, "C1=0.010 pF");
    CHECK(std::abs(net0.caps[1].value - 0.015) < 1e-6, "C2=0.015 pF");
}

// Test 7: Multi-fanout net (data_in has 3 pins)
void test_multi_fanout() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    auto& net1 = parser.data().nets[1]; // data_in
    CHECK(net1.pins.size() == 3, "data_in has 3 pins (1 port + 2 instances)");
    CHECK(net1.caps.size() == 3, "data_in has 3 cap entries");
}

// Test 8: Net map lookup
void test_net_map_lookup() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    auto& d = parser.data();
    auto it = d.net_map.find("data_out");
    CHECK(it != d.net_map.end(), "data_out found in net_map");
    if (it != d.net_map.end()) {
        CHECK(d.nets[it->second].name == "data_out", "net_map index correct");
    }
}

// Test 9: Coupling cap (inter-net)
void test_coupling_cap() {
    SpefParser parser;
    bool ok = parser.parse_string(COUPLING_CAP_SPEF);
    CHECK(ok, "coupling SPEF parsed");
    auto& d = parser.data();
    CHECK(d.nets.size() >= 1, "at least 1 net");
    auto& net = d.nets[0];
    // Should have coupling cap entry (node2 is non-empty)
    bool has_coupling = false;
    for (auto& c : net.caps) {
        if (!c.node2.empty()) {
            has_coupling = true;
            CHECK(std::abs(c.value - 7.0) < 1e-6, "coupling cap=7.0");
        }
    }
    CHECK(has_coupling, "coupling cap detected");
}

// Test 10: Unit scaling (PS/FF/KOHM)
void test_unit_scaling() {
    SpefParser parser;
    parser.parse_string(COUPLING_CAP_SPEF);
    auto& u = parser.data().units;
    CHECK(std::abs(u.time_scale - 1e-12) < 1e-20, "1 PS = 1e-12");
    CHECK(std::abs(u.cap_scale - 1e-15) < 1e-20, "1 FF = 1e-15");
    CHECK(std::abs(u.res_scale - 1e3) < 1e-6, "1 KOHM = 1e3");
}

// Test 11: Empty SPEF (just header)
void test_empty_nets() {
    const char* empty_spef = R"(
*SPEF "IEEE 1481-2009"
*DESIGN "empty_chip"
*T_UNIT 1 NS
*C_UNIT 1 PF
*R_UNIT 1 OHM
*L_UNIT 1 NH
)";
    SpefParser parser;
    bool ok = parser.parse_string(empty_spef);
    CHECK(ok, "empty SPEF parses");
    CHECK(parser.data().design_name == "empty_chip", "design name from empty SPEF");
    CHECK(parser.data().nets.empty(), "no nets in empty SPEF");
}

// Test 12: Print stats doesn't crash
void test_print_stats() {
    SpefParser parser;
    parser.parse_string(BASIC_SPEF);
    // Just ensure it doesn't crash
    std::ostringstream devnull;
    auto* old = std::cout.rdbuf(devnull.rdbuf());
    parser.print_stats();
    std::cout.rdbuf(old);
    CHECK(true, "print_stats doesn't crash");
}

int main() {
    std::cout << "=== Phase 63: SPEF/DSPF Parser Tests ===\n";
    test_header_parse();
    test_name_map();
    test_net_total_cap();
    test_pin_parasitics();
    test_resistors();
    test_capacitors();
    test_multi_fanout();
    test_net_map_lookup();
    test_coupling_cap();
    test_unit_scaling();
    test_empty_nets();
    test_print_stats();
    std::cout << "Phase 63: " << tests_passed << "/" << tests_run << " passed\n";
    return (tests_passed == tests_run) ? 0 : 1;
}
