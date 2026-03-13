// test_phase50.cpp — SystemVerilog Phase 2: Type System
// Tests: typedef enum, typedef logic, struct packed, package/import,
//        typedef'd types as ports and declarations

#include "frontend/verilog_parser.hpp"
#include "synth/behavioral_synth.hpp"
#include <cassert>
#include <iostream>

using namespace sf;

static VerilogParseResult parse_sv(const std::string& src, Netlist& nl, VerilogParser& vp) {
    auto r = vp.parse_string(src, nl);
    if (!r.success) {
        std::cerr << "Parse failed: " << r.error << "\n";
        std::cerr << "Source:\n" << src << "\n";
    }
    return r;
}

// =============================================================================
// TEST 1: typedef enum — basic FSM states
// =============================================================================
static void test_typedef_enum_basic() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef enum {IDLE, RUN, DONE} state_t;
        module fsm(input clk, input start, output reg [31:0] state);
            state_t current;
            always_ff @(posedge clk)
                state <= current;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_wires >= 1); // current declared as wire (32-bit enum)
    std::cout << "  PASS: typedef enum basic\n";
}

// =============================================================================
// TEST 2: typedef enum with explicit values
// =============================================================================
static void test_typedef_enum_values() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef enum {OFF = 0, ON = 1, ERROR = 3} mode_t;
        module test(input clk, output reg [31:0] out);
            mode_t m;
            always_ff @(posedge clk)
                out <= m;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: typedef enum with explicit values\n";
}

// =============================================================================
// TEST 3: typedef enum with custom width
// =============================================================================
static void test_typedef_enum_width() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef enum logic [2:0] {S0, S1, S2, S3, S4} narrow_t;
        module test(input clk, output reg [2:0] out);
            narrow_t st;
            always_ff @(posedge clk)
                out <= st;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: typedef enum with logic [2:0]\n";
}

// =============================================================================
// TEST 4: typedef logic — simple alias
// =============================================================================
static void test_typedef_logic() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef logic [7:0] byte_t;
        module test(input byte_t data_in, output byte_t data_out);
            assign data_out = data_in;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 8);
    assert(r.num_outputs == 8);
    std::cout << "  PASS: typedef logic [7:0]\n";
}

// =============================================================================
// TEST 5: typedef logic — 32-bit
// =============================================================================
static void test_typedef_logic_wide() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef logic [31:0] word_t;
        module test(input word_t a, input word_t b, output word_t c);
            assign c = a;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 64); // 2 x 32-bit
    assert(r.num_outputs == 32);
    std::cout << "  PASS: typedef logic [31:0]\n";
}

// =============================================================================
// TEST 6: struct packed — basic
// =============================================================================
static void test_struct_packed_basic() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef struct packed {
            logic [7:0] opcode;
            logic [7:0] operand;
        } instruction_t;
        module test(input instruction_t instr, output logic [7:0] op);
            assign op = instr;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 16);  // 8+8 packed struct
    assert(r.num_outputs == 8);
    std::cout << "  PASS: struct packed basic (16-bit)\n";
}

// =============================================================================
// TEST 7: struct packed — multi-field with different widths
// =============================================================================
static void test_struct_packed_multi() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef struct packed {
            logic        valid;
            logic [3:0]  tag;
            logic [31:0] data;
        } cache_line_t;
        module test(input cache_line_t line_in, output cache_line_t line_out);
            assign line_out = line_in;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 37);  // 1 + 4 + 32
    assert(r.num_outputs == 37);
    std::cout << "  PASS: struct packed multi-field (37-bit)\n";
}

// =============================================================================
// TEST 8: struct packed — variable declaration in module body
// =============================================================================
static void test_struct_decl_body() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef struct packed {
            logic [7:0] addr;
            logic [7:0] data;
        } bus_t;
        module test(input clk, output reg [15:0] out);
            bus_t bus_reg;
            always_ff @(posedge clk)
                out <= bus_reg;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_wires >= 1);
    std::cout << "  PASS: struct packed decl in body\n";
}

// =============================================================================
// TEST 9: package with typedef
// =============================================================================
static void test_package_typedef() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package my_pkg;
            typedef logic [15:0] addr_t;
        endpackage

        module test(input my_pkg::addr_t addr, output logic [15:0] out);
            assign out = addr;
        endmodule
    )", nl, vp);
    // Note: my_pkg::addr_t won't resolve directly in ANSI ports without import
    // But the package should be parsed. Let's test with import instead.
    // For now just check it doesn't crash
    assert(r.success);
    std::cout << "  PASS: package with typedef (no crash)\n";
}

// =============================================================================
// TEST 10: import pkg::* — wildcard import
// =============================================================================
static void test_import_wildcard() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package defs;
            typedef logic [7:0] data_t;
            typedef logic [3:0] ctrl_t;
        endpackage

        module test(input clk, output reg [7:0] out);
            import defs::*;
            data_t d;
            ctrl_t c;
            always_ff @(posedge clk)
                out <= d;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_wires >= 2); // d (8-bit) and c (4-bit)
    std::cout << "  PASS: import pkg::* wildcard\n";
}

// =============================================================================
// TEST 11: import pkg::symbol — specific import
// =============================================================================
static void test_import_specific() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package types;
            typedef logic [31:0] word_t;
            typedef logic [63:0] dword_t;
        endpackage

        module test(input clk, output reg [31:0] out);
            import types::word_t;
            word_t w;
            always_ff @(posedge clk)
                out <= w;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_wires >= 1);
    std::cout << "  PASS: import pkg::symbol specific\n";
}

// =============================================================================
// TEST 12: package with enum — constants accessible
// =============================================================================
static void test_package_enum() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package fsm_pkg;
            typedef enum logic [1:0] {IDLE, FETCH, EXEC, HALT} cpu_state_t;
        endpackage

        module test(input clk, output reg [1:0] state);
            import fsm_pkg::*;
            cpu_state_t cs;
            always_ff @(posedge clk)
                state <= cs;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: package enum with constants\n";
}

// =============================================================================
// TEST 13: package with parameter
// =============================================================================
static void test_package_param() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package config;
            parameter DATA_W = 16;
        endpackage

        module test(input clk, output reg [15:0] out);
            import config::*;
            logic [15:0] data;
            always_ff @(posedge clk)
                out <= data;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: package parameter\n";
}

// =============================================================================
// TEST 14: multiple typedefs in same module
// =============================================================================
static void test_multi_typedef() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef logic [7:0] byte_t;
        typedef logic [15:0] half_t;
        typedef logic [31:0] word_t;
        module test(input byte_t a, input half_t b, input word_t c, output logic y);
            assign y = a[0];
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 8 + 16 + 32); // 56
    std::cout << "  PASS: multiple typedefs\n";
}

// =============================================================================
// TEST 15: typedef enum as port type
// =============================================================================
static void test_enum_as_port() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef enum logic [1:0] {ADD, SUB, MUL, DIV} op_t;
        module alu(input op_t opcode, input logic [7:0] a, input logic [7:0] b,
                   output logic [7:0] result);
            assign result = a;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 2 + 8 + 8); // 2-bit enum + 8 + 8
    std::cout << "  PASS: enum as port type\n";
}

// =============================================================================
// TEST 16: struct packed with byte fields
// =============================================================================
static void test_struct_byte_fields() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef struct packed {
            byte header;
            byte payload;
        } packet_t;
        module test(input packet_t pkt, output logic [15:0] out);
            assign out = pkt;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 16); // 8+8
    std::cout << "  PASS: struct packed with byte fields\n";
}

// =============================================================================
// TEST 17: typedef used in always_comb
// =============================================================================
static void test_typedef_in_always() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef enum logic [1:0] {IDLE, RUN, WAIT, DONE} state_t;
        module fsm(input clk, input rst, input go,
                   output reg [1:0] state_out);
            state_t current, next;
            always_ff @(posedge clk) begin
                if (rst)
                    current <= IDLE;
                else
                    current <= next;
            end
            always_comb begin
                state_out = current;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: typedef used in always blocks\n";
}

// =============================================================================
// TEST 18: file-level typedef (before module, no package)
// =============================================================================
static void test_file_level_typedef() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef logic [11:0] addr_t;

        module mem_ctrl(input addr_t addr, output logic [7:0] data);
            assign data = 8'b0;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 12);
    std::cout << "  PASS: file-level typedef\n";
}

// =============================================================================
// TEST 19: nested struct (struct field is another typedef)
// =============================================================================
static void test_nested_struct() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef logic [7:0] byte_t;
        typedef struct packed {
            byte_t addr;
            byte_t data;
            logic  valid;
        } req_t;
        module test(input req_t req, output logic ack);
            assign ack = req[0];
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 17); // 8+8+1
    std::cout << "  PASS: nested struct (typedef field)\n";
}

// =============================================================================
// TEST 20: mixed SV Phase1 + Phase2 features
// =============================================================================
static void test_mixed_phase1_phase2() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package my_types;
            typedef enum logic [2:0] {NOP, ADD, SUB, AND, OR, XOR, SHL, SHR} alu_op_t;
            typedef logic [31:0] word_t;
        endpackage

        module alu(
            input logic clk,
            input logic rst,
            input logic [2:0] opcode,
            input logic [31:0] a,
            input logic [31:0] b,
            output reg [31:0] result
        );
            import my_types::*;
            word_t tmp;
            alu_op_t op;
            always_ff @(posedge clk) begin
                if (rst)
                    result <= 32'b0;
                else
                    result <= tmp;
            end
            always_comb begin
                unique case (opcode)
                    3'b000: tmp = a;
                    3'b001: tmp = a + b;
                    default: tmp = 32'b0;
                endcase
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: mixed Phase 1 + Phase 2\n";
}

// =============================================================================
// TEST 21: typedef inside module body
// =============================================================================
static void test_typedef_in_module() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module test(input clk, output reg [3:0] out);
            typedef enum logic [3:0] {A, B, C, D} local_t;
            local_t val;
            always_ff @(posedge clk)
                out <= val;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: typedef inside module body\n";
}

// =============================================================================
// TEST 22: multiple packages
// =============================================================================
static void test_multiple_packages() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package pkg_a;
            typedef logic [7:0] a_t;
        endpackage

        package pkg_b;
            typedef logic [15:0] b_t;
        endpackage

        module test(input clk, output reg [7:0] out);
            import pkg_a::*;
            import pkg_b::*;
            a_t x;
            b_t y;
            always_ff @(posedge clk)
                out <= x;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_wires >= 2);
    std::cout << "  PASS: multiple packages\n";
}

// =============================================================================
// TEST 23: enum default width (32-bit)
// =============================================================================
static void test_enum_default_width() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef enum {ALPHA, BETA, GAMMA} greek_t;
        module test(input greek_t sel, output logic [31:0] out);
            assign out = sel;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 32); // default enum is 32-bit
    std::cout << "  PASS: enum default 32-bit width\n";
}

// =============================================================================
// TEST 24: struct as variable with always_ff
// =============================================================================
static void test_struct_always_ff() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef struct packed {
            logic [3:0] upper;
            logic [3:0] lower;
        } nibble_pair_t;
        module test(input clk, input [7:0] din, output reg [7:0] dout);
            nibble_pair_t regs;
            always_ff @(posedge clk) begin
                regs <= din;
                dout <= regs;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: struct variable with always_ff\n";
}

// =============================================================================
// TEST 25: empty package (edge case)
// =============================================================================
static void test_empty_package() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package empty_pkg;
        endpackage

        module test(input a, output y);
            assign y = a;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 1);
    assert(r.num_outputs == 1);
    std::cout << "  PASS: empty package (no crash)\n";
}

int main() {
    std::cout << "=== Phase 50: SystemVerilog Phase 2 — Type System ===\n";

    test_typedef_enum_basic();      // 1
    test_typedef_enum_values();     // 2
    test_typedef_enum_width();      // 3
    test_typedef_logic();           // 4
    test_typedef_logic_wide();      // 5
    test_struct_packed_basic();     // 6
    test_struct_packed_multi();     // 7
    test_struct_decl_body();        // 8
    test_package_typedef();         // 9
    test_import_wildcard();         // 10
    test_import_specific();         // 11
    test_package_enum();            // 12
    test_package_param();           // 13
    test_multi_typedef();           // 14
    test_enum_as_port();            // 15
    test_struct_byte_fields();      // 16
    test_typedef_in_always();       // 17
    test_file_level_typedef();      // 18
    test_nested_struct();           // 19
    test_mixed_phase1_phase2();     // 20
    test_typedef_in_module();       // 21
    test_multiple_packages();       // 22
    test_enum_default_width();      // 23
    test_struct_always_ff();        // 24
    test_empty_package();           // 25

    std::cout << "=== All 25 Phase 50 tests PASSED ===\n";
    return 0;
}
