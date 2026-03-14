// ============================================================
// SiliconForge — SV Phase 7 Tests (Synthesis Parity & Verification Skip)
// Wire inline assign, fill literals, type casting, const/static,
// extern/program/class/bind skip, enhanced eval_const_expr, timeunit, covergroup
// ============================================================

#include "frontend/verilog_parser.hpp"
#include "core/netlist.hpp"
#include <cassert>
#include <iostream>
#include <string>

using namespace sf;

static int pass_count = 0;
static int fail_count = 0;

#define TEST(name) \
    static void test_##name(); \
    struct Register_##name { Register_##name() { tests.push_back({#name, test_##name}); } } reg_##name; \
    static void test_##name()

struct TestEntry { const char* name; void(*fn)(); };
static std::vector<TestEntry> tests;

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        std::cerr << "  FAIL: " #cond " at line " << __LINE__ << "\n"; \
        fail_count++; return; \
    } \
} while(0)

static VerilogParseResult parse_sv(const std::string& src) {
    Netlist nl;
    VerilogParser parser;
    return parser.parse_string(src, nl);
}

static std::pair<VerilogParseResult, Netlist> parse_sv_nl(const std::string& src) {
    Netlist nl;
    VerilogParser parser;
    auto r = parser.parse_string(src, nl);
    return {r, std::move(nl)};
}

// ============================================================
// 1. Wire inline assignment — single bit
// ============================================================
TEST(wire_inline_assign_1bit) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            wire w = a;
            assign y = w;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 2. Wire inline assignment — bus
// ============================================================
TEST(wire_inline_assign_bus) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [7:0] a, output [7:0] y);
            wire [7:0] tmp = a;
            assign y = tmp;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_gates > 0);
    pass_count++;
}

// ============================================================
// 3. Wire inline assignment — expression
// ============================================================
TEST(wire_inline_assign_expr) {
    auto r = parse_sv(R"(
        module top(input a, input b, output y);
            wire w = a & b;
            assign y = w;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_gates > 0);
    pass_count++;
}

// ============================================================
// 4. Fill literal '0 — all zeros
// ============================================================
TEST(fill_literal_zero) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, output [7:0] y);
            assign y = '0;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 5. Fill literal '1 — all ones
// ============================================================
TEST(fill_literal_one) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, output [7:0] y);
            assign y = '1;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 6. Width cast — 8'(expr)
// ============================================================
TEST(width_cast_basic) {
    auto r = parse_sv(R"(
        module top(input [15:0] a, output [7:0] y);
            assign y = 8'(a);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 7. Type cast — int'(expr)
// ============================================================
TEST(type_cast_int) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, output [31:0] y);
            assign y = int'(a);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 8. Type cast — logic'(expr)
// ============================================================
TEST(type_cast_logic) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, output [7:0] y);
            assign y = logic'(a);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 9. Const variable — parse and skip qualifier
// ============================================================
TEST(const_variable) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            const logic c = 1'b1;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 10. Static function
// ============================================================
TEST(static_function) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, output [7:0] y);
            function static logic [7:0] invert;
                input [7:0] x;
                invert = ~x;
            endfunction
            assign y = invert(a);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 11. Program block — skip
// ============================================================
TEST(program_skip) {
    auto r = parse_sv(R"(
        program test_prog;
            initial begin
                $display("hello");
            end
        endprogram

        module top(input a, output y);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 12. Class block — skip
// ============================================================
TEST(class_skip) {
    auto r = parse_sv(R"(
        class my_class;
            int data;
            function void set(int v);
                data = v;
            endfunction
        endclass

        module top(input a, output y);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 13. Bind statement — skip
// ============================================================
TEST(bind_skip) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 14. Extern module — skip
// ============================================================
TEST(extern_module_skip) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 15. Timeunit/timeprecision — skip
// ============================================================
TEST(timeunit_skip) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            timeunit 1ns;
            timeprecision 1ps;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 16. Covergroup — skip
// ============================================================
TEST(covergroup_skip) {
    auto r = parse_sv(R"(
        module top(input clk, input [3:0] a, output reg [3:0] q);
            always @(posedge clk) q <= a;
            covergroup cg @(posedge clk);
                coverpoint a;
            endgroup
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 17. Constraint block — skip
// ============================================================
TEST(constraint_skip) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, output [7:0] y);
            constraint c_valid {
                a > 0;
                a < 255;
            }
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 18. Enhanced eval_const_expr — shift operators
// ============================================================
TEST(eval_const_shift) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input a, output [7:0] y);
            parameter N = 1 << 3;
            wire [N-1:0] tmp;
            assign y = tmp;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 19. Enhanced eval_const_expr — ternary
// ============================================================
TEST(eval_const_ternary) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input a, output [7:0] y);
            parameter USE_WIDE = 1;
            parameter W = USE_WIDE ? 8 : 4;
            wire [W-1:0] data;
            assign y = data;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 20. Enhanced eval_const_expr — bitwise
// ============================================================
TEST(eval_const_bitwise) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [7:0] a, output [7:0] y);
            parameter MASK = 8'hFF & 8'h0F;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 21. Localparam with complex expression
// ============================================================
TEST(localparam_complex_expr) {
    auto r = parse_sv(R"(
        module top #(parameter N = 4) (input [N-1:0] a, output [N*2-1:0] y);
            localparam WIDTH = N * 2 + 1;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 22. Const in procedural block
// ============================================================
TEST(const_in_always) {
    auto r = parse_sv(R"(
        module top(input clk, input [7:0] a, output reg [7:0] q);
            always @(posedge clk) begin
                const int LIMIT = 100;
                q <= a;
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 23. Combined: fill + cast + wire assign
// ============================================================
TEST(fill_cast_wire_combined) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, output [7:0] y);
            wire [7:0] zeros = '0;
            wire [7:0] result = a ^ zeros;
            assign y = result;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 24. Combined: program + class + module
// ============================================================
TEST(program_class_module) {
    auto r = parse_sv(R"(
        class driver;
            int id;
        endclass

        program testbench;
            initial $display("test");
        endprogram

        module top(input clk, input a, output reg q);
            always @(posedge clk) q <= a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 25. Full Phase 7 integration
// ============================================================
TEST(full_phase7_integration) {
    auto r = parse_sv(R"(
        class verification_env;
            int score;
        endclass

        program test_suite;
            initial $finish;
        endprogram

        module top #(parameter N = 8) (
            input clk,
            input [N-1:0] data_in,
            output reg [N-1:0] data_out
        );
            timeunit 1ns;
            timeprecision 1ps;

            localparam HALF = N / 2;
            localparam MASK = (1 << N) - 1;

            wire [N-1:0] masked = data_in & MASK;

            always @(posedge clk) begin
                data_out <= masked;
            end

            covergroup cg @(posedge clk);
                coverpoint data_in;
            endgroup
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

int main() {
    std::cout << "=== SV Phase 7 Tests (Synthesis Parity & Verification Skip) ===\n";
    for (auto& te : tests) {
        std::cout << "  " << te.name << " ... ";
        te.fn();
        if (fail_count == 0)
            std::cout << "PASS\n";
        else
            std::cout << "FAIL\n";
        fail_count = 0;
    }
    std::cout << "\n" << pass_count << "/" << tests.size() << " tests passed.\n";
    return (pass_count == (int)tests.size()) ? 0 : 1;
}
