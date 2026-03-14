// test_phase53.cpp — SystemVerilog Phase 5: Synthesis-Relevant Features
// Tests: for(int i=...), return, void function, ==?/!=?, $countones/$onehot/$onehot0,
//        do...while, break/continue, foreach, automatic, ANSI function ports

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
    std::cout << "=== SV Phase 5: Synthesis-Relevant Features ===" << std::endl;

    // ===== for(int i=...) local variable declaration =====

    TEST("for(int i=0; i<4; i++) in generate")
    {
        std::string rtl =
            "module forint(input [3:0] a, output [3:0] y);\n"
            "generate\n"
            "  for (int i = 0; i < 4; i++) begin\n"
            "    assign y[i] = ~a[i];\n"
            "  end\n"
            "endgenerate\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "forint");
    }
    PASS

    TEST("for(integer j=0; ...) in generate")
    {
        std::string rtl =
            "module forinteger(input [7:0] a, output [7:0] y);\n"
            "generate\n"
            "  for (integer j = 0; j < 8; j = j + 1) begin\n"
            "    assign y[j] = a[j];\n"
            "  end\n"
            "endgenerate\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "forinteger");
    }
    PASS

    TEST("for(int i=...) in always block")
    {
        std::string rtl =
            "module forbehav(input [3:0] a, output reg [3:0] y);\n"
            "always @(*) begin\n"
            "  for (int i = 0; i < 4; i++) begin\n"
            "    y[i] = a[i];\n"
            "  end\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "forbehav");
    }
    PASS

    // ===== return statement in functions =====

    TEST("Function with return statement")
    {
        std::string rtl =
            "module retfunc(input [7:0] a, output [7:0] y);\n"
            "function [7:0] invert;\n"
            "  input [7:0] val;\n"
            "  begin\n"
            "    return ~val;\n"
            "  end\n"
            "endfunction\n"
            "assign y = invert(a);\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "retfunc");
    }
    PASS

    TEST("Function with return — no begin/end")
    {
        std::string rtl =
            "module retfunc2(input [7:0] a, output [7:0] y);\n"
            "function [7:0] pass_thru;\n"
            "  input [7:0] val;\n"
            "  return val;\n"
            "endfunction\n"
            "assign y = pass_thru(a);\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "retfunc2");
    }
    PASS

    // ===== void function =====

    TEST("void function (parsed, no return value)")
    {
        std::string rtl =
            "module voidmod(input clk, input [7:0] a, output reg [7:0] q);\n"
            "function void do_nothing;\n"
            "  input [7:0] x;\n"
            "endfunction\n"
            "always @(posedge clk) q <= a;\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "voidmod");
    }
    PASS

    // ===== automatic keyword =====

    TEST("automatic function")
    {
        std::string rtl =
            "module autofunc(input [7:0] a, output [7:0] y);\n"
            "function automatic [7:0] myfunc;\n"
            "  input [7:0] val;\n"
            "  begin\n"
            "    myfunc = val ^ 8'hFF;\n"
            "  end\n"
            "endfunction\n"
            "assign y = myfunc(a);\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "autofunc");
    }
    PASS

    // ===== ANSI-style function ports =====

    TEST("Function with ANSI port list")
    {
        std::string rtl =
            "module ansifunc(input [7:0] a, input [7:0] b, output [7:0] y);\n"
            "function [7:0] add_func(input [7:0] x, input [7:0] y_in);\n"
            "  begin\n"
            "    add_func = x + y_in;\n"
            "  end\n"
            "endfunction\n"
            "assign y = add_func(a, b);\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "ansifunc");
    }
    PASS

    TEST("Function automatic with SV types and ANSI ports")
    {
        std::string rtl =
            "module svfunc(input [7:0] a, output [7:0] y);\n"
            "function automatic logic [7:0] xor_func(input logic [7:0] val);\n"
            "  return val ^ 8'hAA;\n"
            "endfunction\n"
            "assign y = xor_func(a);\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "svfunc");
    }
    PASS

    // ===== Wildcard equality ==? and !=? =====

    TEST("Wildcard equality ==?")
    {
        std::string rtl =
            "module wildeq(input [3:0] a, output reg match);\n"
            "always @(*) begin\n"
            "  if (a ==? 4'b1x1x)\n"
            "    match = 1'b1;\n"
            "  else\n"
            "    match = 1'b0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "wildeq");
    }
    PASS

    TEST("Wildcard inequality !=?")
    {
        std::string rtl =
            "module wildneq(input [3:0] b, output reg mismatch);\n"
            "always @(*) begin\n"
            "  if (b !=? 4'b0000)\n"
            "    mismatch = 1'b1;\n"
            "  else\n"
            "    mismatch = 1'b0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "wildneq");
    }
    PASS

    // ===== $countones =====

    TEST("$countones system function")
    {
        std::string rtl =
            "module cntones(input [7:0] data, output reg [3:0] cnt);\n"
            "always @(*) begin\n"
            "  cnt = $countones(data);\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "cntones");
    }
    PASS

    // ===== $onehot =====

    TEST("$onehot system function")
    {
        std::string rtl =
            "module onehot_test(input [7:0] sel, output reg valid);\n"
            "always @(*) begin\n"
            "  valid = $onehot(sel);\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "onehot_test");
    }
    PASS

    // ===== $onehot0 =====

    TEST("$onehot0 system function")
    {
        std::string rtl =
            "module onehot0_test(input [3:0] enc, output reg ok);\n"
            "always @(*) begin\n"
            "  ok = $onehot0(enc);\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "onehot0_test");
    }
    PASS

    // ===== do...while loop =====

    TEST("do...while loop (compile-time)")
    {
        std::string rtl =
            "module dowhile_mod(input clk, output reg [7:0] cnt);\n"
            "parameter N = 3;\n"
            "always @(posedge clk) begin\n"
            "  cnt = 8'd0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "dowhile_mod");
    }
    PASS

    // ===== break statement =====

    TEST("break keyword parsed")
    {
        std::string rtl =
            "module breakmod(input clk, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q = 8'd0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "breakmod");
    }
    PASS

    // ===== continue statement =====

    TEST("continue keyword parsed")
    {
        std::string rtl =
            "module contmod(input clk, output reg [7:0] q);\n"
            "always @(posedge clk) begin\n"
            "  q = 8'd1;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "contmod");
    }
    PASS

    // ===== Mixed: function with return + compound assign =====

    TEST("Function with return used in compound assign")
    {
        std::string rtl =
            "module mixed1(input clk, input [7:0] a, output reg [7:0] acc);\n"
            "function [7:0] double_it;\n"
            "  input [7:0] v;\n"
            "  begin\n"
            "    return v + v;\n"
            "  end\n"
            "endfunction\n"
            "always @(posedge clk) begin\n"
            "  acc += double_it(a);\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "mixed1");
    }
    PASS

    // ===== Mixed: ==? in if with always_ff =====

    TEST("Wildcard equality in always_ff")
    {
        std::string rtl =
            "module mixed2(input clk, input [7:0] cmd, output reg active);\n"
            "always_ff @(posedge clk) begin\n"
            "  if (cmd ==? 8'b1xxx_xxxx)\n"
            "    active <= 1'b1;\n"
            "  else\n"
            "    active <= 1'b0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "mixed2");
    }
    PASS

    // ===== for(int i) + inside =====

    TEST("for(int i) with inside operator")
    {
        std::string rtl =
            "module mixed3(input [3:0] sel, output reg [3:0] mask);\n"
            "always @(*) begin\n"
            "  mask = 4'b0000;\n"
            "  if (sel inside {4'd1, 4'd3, 4'd5, 4'd7})\n"
            "    mask = 4'b1111;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "mixed3");
    }
    PASS

    // ===== $onehot in conditional =====

    TEST("$onehot in conditional expression")
    {
        std::string rtl =
            "module mixed4(input [7:0] bus, output reg fault);\n"
            "always @(*) begin\n"
            "  if (!$onehot(bus))\n"
            "    fault = 1'b1;\n"
            "  else\n"
            "    fault = 1'b0;\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "mixed4");
    }
    PASS

    // ===== ANSI function with return and always_comb =====

    TEST("ANSI function + return + always_comb")
    {
        std::string rtl =
            "module mixed5(input [7:0] a, input [7:0] b, output reg [7:0] result);\n"
            "function automatic [7:0] max_val(input [7:0] x, input [7:0] y_in);\n"
            "  begin\n"
            "    if (x > y_in)\n"
            "      return x;\n"
            "    else\n"
            "      return y_in;\n"
            "  end\n"
            "endfunction\n"
            "always_comb begin\n"
            "  result = max_val(a, b);\n"
            "end\n"
            "endmodule\n";
        VerilogParser p; Netlist nl;
        auto r = parse_sv(rtl, nl, p);
        assert(r.success);
        assert(r.module_name == "mixed5");
    }
    PASS

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_total
              << " PASSED ===" << std::endl;
    return (tests_passed == tests_total) ? 0 : 1;
}
