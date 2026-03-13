// test_phase49.cpp — SystemVerilog Phase 1: Core Keywords
// Tests: always_ff, always_comb, always_latch, logic/bit types,
//        byte/shortint/int/longint, unique/priority case/if,
//        mixed SV + Verilog-2001

#include "frontend/verilog_parser.hpp"
#include "synth/behavioral_synth.hpp"
#include <cassert>
#include <iostream>
#include <sstream>

using namespace sf;

// Helper: parse SV source, check success, return result
static VerilogParseResult parse_sv(const std::string& src, Netlist& nl, VerilogParser& vp) {
    auto r = vp.parse_string(src, nl);
    if (!r.success) {
        std::cerr << "Parse failed: " << r.error << "\n";
        std::cerr << "Source:\n" << src << "\n";
    }
    return r;
}

// =============================================================================
// TEST 1: always_ff @(posedge clk) — basic flip-flop
// =============================================================================
static void test_always_ff_posedge() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module ff_test(input clk, input d, output reg q);
            always_ff @(posedge clk)
                q <= d;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    // AST root should have an ALWAYS_POS_CLK node
    bool found = false;
    for (auto& c : vp.ast.root->children) {
        if (c->type == AstNodeType::ALWAYS_POS_CLK) {
            found = true;
            assert(c->value == "clk");
        }
    }
    assert(found);
    std::cout << "  PASS: always_ff @(posedge clk)\n";
}

// =============================================================================
// TEST 2: always_ff @(negedge clk) — negative edge flip-flop
// =============================================================================
static void test_always_ff_negedge() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module ff_neg(input clk, input d, output reg q);
            always_ff @(negedge clk)
                q <= d;
        endmodule
    )", nl, vp);
    assert(r.success);
    bool found = false;
    for (auto& c : vp.ast.root->children) {
        if (c->type == AstNodeType::ALWAYS_NEG_CLK) {
            found = true;
            assert(c->value == "clk");
        }
    }
    assert(found);
    std::cout << "  PASS: always_ff @(negedge clk)\n";
}

// =============================================================================
// TEST 3: always_ff with async reset
// =============================================================================
static void test_always_ff_async_reset() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module ff_rst(input clk, input rst_n, input d, output reg q);
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    q <= 1'b0;
                else
                    q <= d;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: always_ff with async reset\n";
}

// =============================================================================
// TEST 4: always_comb — pure combinational
// =============================================================================
static void test_always_comb() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module comb_test(input a, input b, output reg y);
            always_comb
                y = a & b;
        endmodule
    )", nl, vp);
    assert(r.success);
    bool found = false;
    for (auto& c : vp.ast.root->children) {
        if (c->type == AstNodeType::ALWAYS_COMB && c->value == "*") {
            found = true;
        }
    }
    assert(found);
    std::cout << "  PASS: always_comb\n";
}

// =============================================================================
// TEST 5: always_comb with begin-end block
// =============================================================================
static void test_always_comb_block() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module comb_blk(input [1:0] sel, input a, input b, input c, output reg y);
            always_comb begin
                case (sel)
                    2'b00: y = a;
                    2'b01: y = b;
                    default: y = c;
                endcase
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: always_comb with begin-end\n";
}

// =============================================================================
// TEST 6: always_latch
// =============================================================================
static void test_always_latch() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module latch_test(input en, input d, output reg q);
            always_latch
                if (en) q <= d;
        endmodule
    )", nl, vp);
    assert(r.success);
    bool found = false;
    for (auto& c : vp.ast.root->children) {
        if (c->type == AstNodeType::ALWAYS_COMB && c->value == "*_latch") {
            found = true;
        }
    }
    assert(found);
    std::cout << "  PASS: always_latch\n";
}

// =============================================================================
// TEST 7: logic type — scalar port
// =============================================================================
static void test_logic_scalar() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module logic_test(input logic a, input logic b, output logic y);
            assign y = a ^ b;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 2);
    assert(r.num_outputs == 1);
    std::cout << "  PASS: logic scalar ports\n";
}

// =============================================================================
// TEST 8: logic type — bus port
// =============================================================================
static void test_logic_bus() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module logic_bus(input logic [7:0] data_in, output logic [7:0] data_out);
            assign data_out = data_in;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 8);
    assert(r.num_outputs == 8);
    std::cout << "  PASS: logic bus ports [7:0]\n";
}

// =============================================================================
// TEST 9: bit type — scalar
// =============================================================================
static void test_bit_scalar() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module bit_test(input bit a, output bit y);
            assign y = a;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 1);
    assert(r.num_outputs == 1);
    std::cout << "  PASS: bit scalar port\n";
}

// =============================================================================
// TEST 10: bit type — bus
// =============================================================================
static void test_bit_bus() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module bit_bus(input bit [3:0] nibble, output bit [3:0] out);
            assign out = nibble;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 4);
    assert(r.num_outputs == 4);
    std::cout << "  PASS: bit bus ports [3:0]\n";
}

// =============================================================================
// TEST 11: byte type — implicit [7:0]
// =============================================================================
static void test_byte_port() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module byte_test(input byte data, output byte result);
            assign result = data;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 8);
    assert(r.num_outputs == 8);
    std::cout << "  PASS: byte port (implicit [7:0])\n";
}

// =============================================================================
// TEST 12: shortint type — implicit [15:0]
// =============================================================================
static void test_shortint_port() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module shortint_test(input shortint val, output shortint out);
            assign out = val;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 16);
    assert(r.num_outputs == 16);
    std::cout << "  PASS: shortint port (implicit [15:0])\n";
}

// =============================================================================
// TEST 13: int type — implicit [31:0]
// =============================================================================
static void test_int_port() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module int_test(input int val, output int out);
            assign out = val;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 32);
    assert(r.num_outputs == 32);
    std::cout << "  PASS: int port (implicit [31:0])\n";
}

// =============================================================================
// TEST 14: longint type — implicit [63:0]
// =============================================================================
static void test_longint_port() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module longint_test(input longint val, output longint out);
            assign out = val;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 64);
    assert(r.num_outputs == 64);
    std::cout << "  PASS: longint port (implicit [63:0])\n";
}

// =============================================================================
// TEST 15: logic wire declaration in module body
// =============================================================================
static void test_logic_wire_decl() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module logic_wire(input a, input b, output y);
            logic tmp;
            assign tmp = a & b;
            assign y = tmp;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_wires >= 1);
    std::cout << "  PASS: logic wire declaration\n";
}

// =============================================================================
// TEST 16: logic bus declaration in module body
// =============================================================================
static void test_logic_bus_decl() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module logic_bus_decl(input [3:0] a, output [3:0] y);
            logic [3:0] tmp;
            assign tmp = a;
            assign y = tmp;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: logic bus declaration [3:0]\n";
}

// =============================================================================
// TEST 17: unique case
// =============================================================================
static void test_unique_case() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module ucase(input [1:0] sel, input a, input b, input c, input d, output reg y);
            always_comb begin
                unique case (sel)
                    2'b00: y = a;
                    2'b01: y = b;
                    2'b10: y = c;
                    2'b11: y = d;
                endcase
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: unique case\n";
}

// =============================================================================
// TEST 18: priority case
// =============================================================================
static void test_priority_case() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module pcase(input [1:0] sel, input a, input b, output reg y);
            always_comb begin
                priority case (sel)
                    2'b00: y = a;
                    2'b01: y = b;
                    default: y = 1'b0;
                endcase
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: priority case\n";
}

// =============================================================================
// TEST 19: unique if
// =============================================================================
static void test_unique_if() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module uif(input a, input b, input sel, output reg y);
            always_comb begin
                unique if (sel)
                    y = a;
                else
                    y = b;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: unique if\n";
}

// =============================================================================
// TEST 20: priority if
// =============================================================================
static void test_priority_if() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module pif(input [1:0] sel, input a, input b, input c, output reg y);
            always_comb begin
                priority if (sel == 2'b00)
                    y = a;
                else if (sel == 2'b01)
                    y = b;
                else
                    y = c;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: priority if\n";
}

// =============================================================================
// TEST 21: Mixed SV + Verilog-2001 in same module
// =============================================================================
static void test_mixed_sv_verilog() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module mixed(input clk, input rst, input [7:0] a, input [7:0] b,
                     output reg [7:0] sum, output reg [7:0] latched);
            wire [7:0] comb_sum;
            assign comb_sum = a + b;

            always_ff @(posedge clk) begin
                if (rst)
                    sum <= 8'b0;
                else
                    sum <= comb_sum;
            end

            always @(*) begin
                latched = a;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    // Should have both SV and V2001 always blocks
    int pos_clk = 0, comb = 0;
    for (auto& c : vp.ast.root->children) {
        if (c->type == AstNodeType::ALWAYS_POS_CLK) pos_clk++;
        if (c->type == AstNodeType::ALWAYS_COMB) comb++;
    }
    assert(pos_clk >= 1);
    assert(comb >= 1);
    std::cout << "  PASS: mixed SV + Verilog-2001\n";
}

// =============================================================================
// TEST 22: always_ff with begin-end and multiple assignments
// =============================================================================
static void test_always_ff_multi_assign() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module ff_multi(input clk, input [7:0] d1, input [7:0] d2,
                        output reg [7:0] q1, output reg [7:0] q2);
            always_ff @(posedge clk) begin
                q1 <= d1;
                q2 <= d2;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: always_ff multi-assignment\n";
}

// =============================================================================
// TEST 23: byte/shortint/longint in module body declarations
// =============================================================================
static void test_sv_type_body_decls() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module type_decls(input [7:0] a, output [7:0] y);
            byte b_var;
            shortint s_var;
            longint l_var;
            assign y = a;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: byte/shortint/longint body decls\n";
}

// =============================================================================
// TEST 24: logic memory array declaration
// =============================================================================
static void test_logic_memory_array() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module mem_test(input clk, input [7:0] wdata, input [3:0] addr,
                        input we, output reg [7:0] rdata);
            logic [7:0] mem [0:15];
            always_ff @(posedge clk) begin
                if (we)
                    mem[addr] <= wdata;
                rdata <= mem[addr];
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: logic memory array\n";
}

// =============================================================================
// TEST 25: All SV types as ports in single module
// =============================================================================
static void test_all_sv_types() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        module all_types(
            input logic a,
            input bit b,
            input byte c,
            input shortint d,
            input int e,
            input longint f,
            output logic y
        );
            assign y = a;
        endmodule
    )", nl, vp);
    assert(r.success);
    // a=1, b=1, c=8, d=16, e=32, f=64 = 122 input bits
    assert(r.num_inputs == 122);
    assert(r.num_outputs == 1);
    std::cout << "  PASS: all SV types as ports\n";
}

int main() {
    std::cout << "=== Phase 49: SystemVerilog Phase 1 — Core Keywords ===\n";

    test_always_ff_posedge();       // 1
    test_always_ff_negedge();       // 2
    test_always_ff_async_reset();   // 3
    test_always_comb();             // 4
    test_always_comb_block();       // 5
    test_always_latch();            // 6
    test_logic_scalar();            // 7
    test_logic_bus();               // 8
    test_bit_scalar();              // 9
    test_bit_bus();                 // 10
    test_byte_port();               // 11
    test_shortint_port();           // 12
    test_int_port();                // 13
    test_longint_port();            // 14
    test_logic_wire_decl();         // 15
    test_logic_bus_decl();          // 16
    test_unique_case();             // 17
    test_priority_case();           // 18
    test_unique_if();               // 19
    test_priority_if();             // 20
    test_mixed_sv_verilog();        // 21
    test_always_ff_multi_assign();  // 22
    test_sv_type_body_decls();      // 23
    test_logic_memory_array();      // 24
    test_all_sv_types();            // 25

    std::cout << "=== All 25 Phase 49 tests PASSED ===\n";
    return 0;
}
