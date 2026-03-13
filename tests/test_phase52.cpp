// SV Phase 4 Tests — Advanced RTL: compound assign, ++/--, inside, .* port
#include "frontend/verilog_parser.hpp"
#include "synth/behavioral_synth.hpp"
#include <cassert>
#include <iostream>
#include <string>

using namespace sf;

static int tests_passed = 0;
static int tests_total = 0;

static VerilogParseResult parse_sv(const std::string& src, Netlist& nl, VerilogParser& vp) {
    auto r = vp.parse_string(src, nl);
    if (!r.success) {
        std::cerr << "Parse failed: " << r.error << "\n";
        std::cerr << "Source:\n" << src << "\n";
    }
    return r;
}

#define TEST(name) { \
    tests_total++; \
    std::cout << "  Test " << tests_total << ": " << name << " ... "; \
    try {

#define PASS \
        tests_passed++; \
        std::cout << "PASS" << std::endl; \
    } catch (const std::exception& e) { \
        std::cout << "FAIL (" << e.what() << ")" << std::endl; \
    } catch (...) { \
        std::cout << "FAIL (unknown)" << std::endl; \
    } \
}

int main() {
    std::cout << "=== SV Phase 4: Advanced RTL Operators ===" << std::endl;

    // ===== Compound Assignments =====

    TEST("Plus-assign (+=)")
    {
        std::string rtl =
            "module top(input clk, input [7:0] a, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q += a;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "top");
    }
    PASS

    TEST("Minus-assign (-=)")
    {
        std::string rtl =
            "module sub(input clk, input [7:0] b, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q -= b;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "sub");
    }
    PASS

    TEST("Star-assign (*=)")
    {
        std::string rtl =
            "module mul(input clk, input [7:0] c, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q *= c;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "mul");
    }
    PASS

    TEST("And-assign (&=)")
    {
        std::string rtl =
            "module andm(input clk, input [7:0] d, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q &= d;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "andm");
    }
    PASS

    TEST("Or-assign (|=)")
    {
        std::string rtl =
            "module orm(input clk, input [7:0] e, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q |= e;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "orm");
    }
    PASS

    TEST("Xor-assign (^=)")
    {
        std::string rtl =
            "module xorm(input clk, input [7:0] f, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q ^= f;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "xorm");
    }
    PASS

    // ===== Increment / Decrement =====

    TEST("Postfix increment (x++)")
    {
        std::string rtl =
            "module inc1(input clk, output reg [7:0] cnt);\n"
            "always @(posedge clk) begin\n"
            "  cnt++;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "inc1");
    }
    PASS

    TEST("Postfix decrement (x--)")
    {
        std::string rtl =
            "module dec1(input clk, output reg [7:0] cnt);\n"
            "always @(posedge clk) begin\n"
            "  cnt--;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "dec1");
    }
    PASS

    TEST("Prefix increment (++x)")
    {
        std::string rtl =
            "module inc2(input clk, output reg [7:0] cnt);\n"
            "always @(posedge clk) begin\n"
            "  ++cnt;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "inc2");
    }
    PASS

    TEST("Prefix decrement (--x)")
    {
        std::string rtl =
            "module dec2(input clk, output reg [7:0] cnt);\n"
            "always @(posedge clk) begin\n"
            "  --cnt;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "dec2");
    }
    PASS

    // ===== For-loop with ++ =====

    TEST("For loop with i++")
    {
        std::string rtl =
            "module forinc(input [3:0] a, output [3:0] y);\n"
            "genvar i;\n"
            "generate\n"
            "  for (i = 0; i < 4; i++) begin\n"
            "    assign y[i] = a[i];\n"
            "  end\n"
            "endgenerate\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "forinc");
    }
    PASS

    TEST("For loop with classic i=i+1")
    {
        std::string rtl =
            "module forclass(input [3:0] a, output [3:0] y);\n"
            "genvar i;\n"
            "generate\n"
            "  for (i = 0; i < 4; i = i + 1) begin\n"
            "    assign y[i] = a[i];\n"
            "  end\n"
            "endgenerate\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "forclass");
    }
    PASS

    TEST("For loop with i += 2")
    {
        std::string rtl =
            "module forstep(input [7:0] a, output [7:0] y);\n"
            "genvar i;\n"
            "generate\n"
            "  for (i = 0; i < 8; i += 2) begin\n"
            "    assign y[i] = a[i];\n"
            "  end\n"
            "endgenerate\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "forstep");
    }
    PASS

    // ===== Inside Operator =====

    TEST("Inside with single values")
    {
        std::string rtl =
            "module ins1(input [2:0] sel, output reg [7:0] q);\n"
            "always @(*) begin\n"
            "  if (sel inside {3'd1, 3'd3, 3'd5})\n"
            "    q = 8'hFF;\n"
            "  else\n"
            "    q = 8'h00;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "ins1");
    }
    PASS

    TEST("Inside with range [lo:hi]")
    {
        std::string rtl =
            "module ins2(input [3:0] val, output reg hit);\n"
            "always @(*) begin\n"
            "  if (val inside {[4'd2:4'd5]})\n"
            "    hit = 1'b1;\n"
            "  else\n"
            "    hit = 1'b0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "ins2");
    }
    PASS

    TEST("Inside mixed values and ranges")
    {
        std::string rtl =
            "module ins3(input [3:0] x, output reg match);\n"
            "always @(*) begin\n"
            "  if (x inside {4'd0, [4'd3:4'd7], 4'd15})\n"
            "    match = 1'b1;\n"
            "  else\n"
            "    match = 1'b0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "ins3");
    }
    PASS

    // ===== .* Implicit Port Connection =====

    TEST(".* port auto-connect basic")
    {
        std::string rtl =
            "module child(input a, input b, output y);\n"
            "  assign y = a & b;\n"
            "endmodule\n"
            "\n"
            "module top_dot(input a, input b, output y);\n"
            "  child u1(.*);\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "top_dot");
    }
    PASS

    TEST(".* with explicit override")
    {
        std::string rtl =
            "module child2(input a, input b, output y);\n"
            "  assign y = a ^ b;\n"
            "endmodule\n"
            "\n"
            "module top2(input a, input b, input c, output y);\n"
            "  child2 u1(.b(c), .*);\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "top2");
    }
    PASS

    // ===== $clog2 (verify still working) =====

    TEST("$clog2 in parameter")
    {
        std::string rtl =
            "module clog2_test #(parameter DEPTH = 256)\n"
            "(input [$clog2(DEPTH)-1:0] addr, output [7:0] data);\n"
            "  assign data = {addr, addr};\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "clog2_test");
    }
    PASS

    // ===== Compound assign in always_comb =====

    TEST("+= in always_comb")
    {
        std::string rtl =
            "module combo_add(input [7:0] a, input [7:0] b, output reg [7:0] sum);\n"
            "always_comb begin\n"
            "  sum = a;\n"
            "  sum += b;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "combo_add");
    }
    PASS

    // ===== Mixed: compound assign + if =====

    TEST("Compound assign inside if-else")
    {
        std::string rtl =
            "module condadd(input clk, input sel, input [7:0] a, output reg [7:0] acc);\n"
            "always @(posedge clk) begin\n"
            "  if (sel)\n"
            "    acc += a;\n"
            "  else\n"
            "    acc -= a;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "condadd");
    }
    PASS

    // ===== Multiple compound in sequence =====

    TEST("Multiple compound assigns in sequence")
    {
        std::string rtl =
            "module multi_compound(input clk, input [7:0] a, input [7:0] b, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q += a;\n"
            "  q ^= b;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "multi_compound");
    }
    PASS

    // ===== Inside in always_ff =====

    TEST("Inside operator in always_ff context")
    {
        std::string rtl =
            "module ins_ff(input clk, input [3:0] state, output reg active);\n"
            "always_ff @(posedge clk) begin\n"
            "  if (state inside {4'd1, 4'd2, 4'd4, 4'd8})\n"
            "    active <= 1'b1;\n"
            "  else\n"
            "    active <= 1'b0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "ins_ff");
    }
    PASS

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " PASSED ===" << std::endl;
    return (tests_passed == tests_total) ? 0 : 1;
}
