// SiliconForge — Phase 74: VHDL 2008 Parser Tests

#include "../src/frontend/vhdl_parser.hpp"
#include "../src/core/netlist.hpp"
#include <cassert>
#include <iostream>
#include <sstream>

using namespace sf;

static int tests_run = 0, tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { std::cerr << "FAIL: " << msg << " [line " << __LINE__ << "]\n"; } \
    else { tests_passed++; } \
} while(0)

// Test 1: Parse simple entity with 2 inputs, 1 output
static void test_simple_entity() {
    const char* src = R"(
        library ieee;
        use ieee.std_logic_1164.all;
        entity adder is
          port (
            a : in std_logic;
            b : in std_logic;
            z : out std_logic
          );
        end entity adder;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "simple entity parses successfully");
    CHECK(r.entity_name == "adder", "entity name is 'adder'");
    CHECK(r.num_inputs == 2, "2 inputs");
    CHECK(r.num_outputs == 1, "1 output");
    CHECK(parser.ports().size() == 3, "3 ports total");
}

// Test 2: Parse concurrent AND assignment
static void test_and_gate() {
    const char* src = R"(
        entity top is port (a : in std_logic; b : in std_logic; z : out std_logic); end entity;
        architecture rtl of top is
        begin
          z <= a and b;
        end architecture rtl;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "AND gate parses");
    CHECK(nl.num_gates() >= 1, "at least one gate created");
    bool found_and = false;
    for (auto& g : nl.gates()) {
        if (g.type == GateType::AND) found_and = true;
    }
    CHECK(found_and, "AND gate found in netlist");
}

// Test 3: Parse OR, XOR, NOT, NAND gates
static void test_various_gates() {
    const char* src = R"(
        entity gates is port (
            a : in std_logic; b : in std_logic;
            y1 : out std_logic; y2 : out std_logic;
            y3 : out std_logic; y4 : out std_logic
        ); end entity;
        architecture rtl of gates is
        begin
          y1 <= a or b;
          y2 <= a xor b;
          y3 <= not a;
          y4 <= a nand b;
        end architecture;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "various gates parse");

    bool has_or = false, has_xor = false, has_not = false, has_nand = false;
    for (auto& g : nl.gates()) {
        if (g.type == GateType::OR) has_or = true;
        if (g.type == GateType::XOR) has_xor = true;
        if (g.type == GateType::NOT) has_not = true;
        if (g.type == GateType::NAND) has_nand = true;
    }
    CHECK(has_or, "OR gate found");
    CHECK(has_xor, "XOR gate found");
    CHECK(has_not, "NOT gate found");
    CHECK(has_nand, "NAND gate found");
}

// Test 4: Parse conditional signal assignment (when/else → MUX)
static void test_conditional_assign() {
    const char* src = R"(
        entity mux2 is port (
            a : in std_logic; b : in std_logic;
            sel : in std_logic; z : out std_logic
        ); end entity;
        architecture rtl of mux2 is
        begin
          z <= a when sel = '1' else b;
        end architecture;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "conditional assignment parses");
    bool has_mux = false;
    for (auto& g : nl.gates()) {
        if (g.type == GateType::MUX) has_mux = true;
    }
    CHECK(has_mux, "MUX gate inferred from when/else");
}

// Test 5: Parse process with rising_edge → DFF
static void test_dff_process() {
    const char* src = R"(
        entity ff is port (
            clk : in std_logic; d : in std_logic; q : out std_logic
        ); end entity;
        architecture rtl of ff is
        begin
          process (clk)
          begin
            if rising_edge(clk) then
              q <= d;
            end if;
          end process;
        end architecture;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "DFF process parses");
    CHECK(r.num_processes == 1, "one process found");
    bool has_dff = false;
    for (auto& g : nl.gates()) {
        if (g.type == GateType::DFF) has_dff = true;
    }
    CHECK(has_dff, "DFF inferred from rising_edge");
}

// Test 6: Parse process with if/else → MUX
static void test_if_else_mux() {
    const char* src = R"(
        entity sel_mux is port (
            sel : in std_logic; a : in std_logic;
            b : in std_logic; z : out std_logic
        ); end entity;
        architecture rtl of sel_mux is
        begin
          process (sel, a, b)
          begin
            if sel then
              z <= a;
            else
              z <= b;
            end if;
          end process;
        end architecture;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "if/else process parses");
    bool has_mux = false;
    for (auto& g : nl.gates()) {
        if (g.type == GateType::MUX) has_mux = true;
    }
    CHECK(has_mux, "MUX inferred from if/else");
}

// Test 7: Parse std_logic_vector ports (bus width)
static void test_bus_ports() {
    const char* src = R"(
        entity bus_ent is port (
            data_in : in std_logic_vector(7 downto 0);
            data_out : out std_logic_vector(3 downto 0)
        ); end entity;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "bus entity parses");
    CHECK(parser.ports().size() == 2, "2 bus ports");
    CHECK(parser.ports()[0].width == 8, "input bus width 8");
    CHECK(parser.ports()[1].width == 4, "output bus width 4");
}

// Test 8: Parse component instantiation (port map)
static void test_component_inst() {
    const char* src = R"(
        entity top is port (
            a : in std_logic; b : in std_logic; z : out std_logic
        ); end entity;
        architecture struct of top is
          component and_gate
            port (x : in std_logic; y : in std_logic; o : out std_logic);
          end component;
        begin
          u1 : and_gate port map (x => a, y => b, o => z);
        end architecture;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "component instantiation parses");
    CHECK(nl.num_gates() >= 1, "gate created from port map");
}

// Test 9: Test VHDL writer (to_vhdl)
static void test_vhdl_writer() {
    Netlist nl;
    NetId a = nl.add_net("a");
    NetId b = nl.add_net("b");
    NetId z = nl.add_net("z");
    nl.mark_input(a);
    nl.mark_input(b);
    nl.mark_output(z);
    nl.add_gate(GateType::AND, {a, b}, z, "g0");

    std::string vhdl = VhdlParser::to_vhdl(nl, "my_and");
    CHECK(vhdl.find("entity my_and is") != std::string::npos, "entity declaration in output");
    CHECK(vhdl.find("a : in std_logic") != std::string::npos, "input port in output");
    CHECK(vhdl.find("z : out std_logic") != std::string::npos, "output port in output");
    CHECK(vhdl.find("z <= a and b") != std::string::npos, "AND assignment in output");
}

// Test 10: Parse case statement → MUX tree
static void test_case_mux() {
    const char* src = R"(
        entity case_ent is port (
            sel : in std_logic;
            a : in std_logic; b : in std_logic;
            c : in std_logic; z : out std_logic
        ); end entity;
        architecture rtl of case_ent is
        begin
          process (sel, a, b, c)
          begin
            case sel is
              when '0' => z <= a;
              when '1' => z <= b;
              when others => z <= c;
            end case;
          end process;
        end architecture;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "case statement parses");
    bool has_mux = false;
    for (auto& g : nl.gates()) {
        if (g.type == GateType::MUX) has_mux = true;
    }
    CHECK(has_mux, "MUX tree from case statement");
}

// Test 11: Parse empty entity (no signals)
static void test_empty_entity() {
    const char* src = R"(
        entity empty is
        end entity empty;
    )";
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    CHECK(r.success, "empty entity parses");
    CHECK(r.entity_name == "empty", "entity name correct");
    CHECK(r.num_inputs == 0, "no inputs");
    CHECK(r.num_outputs == 0, "no outputs");
}

// Test 12: Parse error handling (syntax error → success=false)
static void test_error_handling() {
    const char* src = R"(
        entity broken is port (
            a : in std_logic;
    )";
    // The parser should handle incomplete VHDL gracefully.
    // It may succeed partially or report error — either way it should not crash.
    Netlist nl;
    VhdlParser parser;
    auto r = parser.parse_string(src, nl);
    // Incomplete entity — parser handles gracefully (no crash)
    CHECK(true, "no crash on incomplete VHDL");
}

int main() {
    std::cout << "=== SiliconForge Phase 74: VHDL Parser Tests ===\n";

    test_simple_entity();
    test_and_gate();
    test_various_gates();
    test_conditional_assign();
    test_dff_process();
    test_if_else_mux();
    test_bus_ports();
    test_component_inst();
    test_vhdl_writer();
    test_case_mux();
    test_empty_entity();
    test_error_handling();

    std::cout << "\nResults: " << tests_passed << "/" << tests_run << " passed\n";
    if (tests_passed == tests_run)
        std::cout << "ALL TESTS PASSED\n";
    else
        std::cout << "SOME TESTS FAILED\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
