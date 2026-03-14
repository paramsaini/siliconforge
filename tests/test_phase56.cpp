// ============================================================
// SiliconForge — SV Phase 8 Tests (Final Synthesis Parity)
// Assign delay, multi-value case, DPI-C, let, fork/join,
// checker/config, string/chandle, unique0, string literals
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
// Test 1: assign with # delay — assign #5 y = a;
// ============================================================
TEST(assign_delay_simple) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            assign #5 y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_gates > 0);
    pass_count++;
}

// ============================================================
// Test 2: assign with parenthesized delay — assign #(3.5) y = a;
// ============================================================
TEST(assign_delay_paren) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            assign #(10) y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_gates > 0);
    pass_count++;
}

// ============================================================
// Test 3: multi-value case item — 2'b00, 2'b01: y = 1;
// ============================================================
TEST(multi_value_case) {
    auto r = parse_sv(R"(
        module top(input [1:0] sel, output reg y);
            always @(*) begin
                case (sel)
                    2'b00, 2'b01: y = 1'b1;
                    default: y = 1'b0;
                endcase
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 4: multi-value case with 3 items
// ============================================================
TEST(multi_value_case_three) {
    auto r = parse_sv(R"(
        module top(input [2:0] sel, output reg y);
            always @(*) begin
                case (sel)
                    3'd0, 3'd1, 3'd2: y = 1'b1;
                    3'd3: y = 1'b0;
                    default: y = 1'bx;
                endcase
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 5: import "DPI-C" function — skip
// ============================================================
TEST(dpi_import_function) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            import "DPI-C" function int c_model(input int x);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 6: import "DPI-C" context task — skip
// ============================================================
TEST(dpi_import_task) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            import "DPI-C" context task c_task(input int x);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 7: export "DPI-C" — skip
// ============================================================
TEST(dpi_export) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            export "DPI-C" function my_func;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 8: let declaration — skip
// ============================================================
TEST(let_declaration) {
    auto r = parse_sv(R"(
        module top(input a, input b, output y);
            let max(x, y) = x > y ? x : y;
            assign y = a & b;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 9: fork...join — skip (not synthesizable)
// ============================================================
TEST(fork_join_skip) {
    auto r = parse_sv(R"(
        module top(input clk, input a, output reg y);
            always @(posedge clk) begin
                fork
                    begin
                        y <= a;
                    end
                join
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 10: fork...join_any — skip
// ============================================================
TEST(fork_join_any_skip) {
    auto r = parse_sv(R"(
        module top(input clk, input a, output reg y);
            always @(posedge clk) begin
                fork
                    y <= a;
                join_any
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 11: fork...join_none — skip
// ============================================================
TEST(fork_join_none_skip) {
    auto r = parse_sv(R"(
        module top(input clk, input a, output reg y);
            always @(posedge clk) begin
                fork
                    y <= a;
                join_none
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 12: checker...endchecker — skip
// ============================================================
TEST(checker_skip) {
    auto r = parse_sv(R"(
        checker my_check(input clk, input a);
            assert property (@(posedge clk) a);
        endchecker

        module top(input a, output y);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 13: config...endconfig — skip
// ============================================================
TEST(config_skip) {
    auto r = parse_sv(R"(
        config my_config;
            design top;
            default liblist work;
        endconfig

        module top(input a, output y);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 14: string type variable — skip
// ============================================================
TEST(string_type_skip) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            string my_str;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 15: chandle type variable — skip
// ============================================================
TEST(chandle_type_skip) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            chandle my_handle;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 16: unique0 case — variant of unique
// ============================================================
TEST(unique0_case) {
    auto r = parse_sv(R"(
        module top(input [1:0] sel, output reg y);
            always @(*) begin
                unique0 case (sel)
                    2'b00: y = 1'b0;
                    2'b01: y = 1'b1;
                endcase
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 17: assign delay + bus — assign #3 out = a & b;
// ============================================================
TEST(assign_delay_bus) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, input [7:0] b, output [7:0] y);
            assign #3 y = a & b;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_gates > 0);
    pass_count++;
}

// ============================================================
// Test 18: string literal in $display — should be skipped
// ============================================================
TEST(string_literal_display) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            assign y = a;
            initial $display("Hello World %d", a);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 19: fork with nested begin/end blocks
// ============================================================
TEST(fork_nested_begin_end) {
    auto r = parse_sv(R"(
        module top(input clk, input a, input b, output reg y);
            always @(posedge clk) begin
                fork
                    begin
                        y <= a;
                    end
                    begin
                        y <= b;
                    end
                join
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 20: multiple features combined
// ============================================================
TEST(combined_phase8) {
    auto r = parse_sv(R"(
        module top(input [7:0] a, input [7:0] b, input [1:0] sel, output [7:0] y);
            import "DPI-C" function int c_add(input int x, input int y);
            let my_max(x, y) = x > y ? x : y;
            string debug_msg;
            assign #2 y = (sel == 2'b00) ? a : b;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_gates > 0);
    pass_count++;
}

// ============================================================
// Test 21: checker and config before module
// ============================================================
TEST(checker_config_before_module) {
    auto r = parse_sv(R"(
        checker bus_check(input clk, input [7:0] data);
            assert property (@(posedge clk) data !== 8'hFF);
        endchecker

        config design_config;
            design top;
        endconfig

        module top(input a, output y);
            assign y = ~a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_gates > 0);
    pass_count++;
}

// ============================================================
// Test 22: DPI-C with pure keyword
// ============================================================
TEST(dpi_pure_function) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            import "DPI-C" pure function int my_func(input int x);
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 23: multi-value casex
// ============================================================
TEST(multi_value_casex) {
    auto r = parse_sv(R"(
        module top(input [3:0] opcode, output reg [1:0] ctrl);
            always @(*) begin
                casex (opcode)
                    4'b000x, 4'b001x: ctrl = 2'b00;
                    4'b01xx: ctrl = 2'b01;
                    default: ctrl = 2'b11;
                endcase
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 24: let declaration with no parameters
// ============================================================
TEST(let_no_params) {
    auto r = parse_sv(R"(
        module top(input a, output y);
            let MAGIC = 42;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Test 25: unique0 in module body level
// ============================================================
TEST(unique0_module_body) {
    auto r = parse_sv(R"(
        module top(input [1:0] sel, input a, input b, output reg y);
            always @(*) begin
                unique0 case (sel)
                    2'd0: y = a;
                    2'd1: y = b;
                    2'd2: y = a & b;
                    default: y = 1'b0;
                endcase
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// Main
// ============================================================
int main() {
    std::cout << "=== SV Phase 8: Final Synthesis Parity ===\n";
    for (auto& t : tests) {
        std::cout << "  " << t.name << "... ";
        t.fn();
        if (fail_count == 0) std::cout << "PASS\n";
        else { std::cout << "FAIL\n"; fail_count = 0; }
    }
    std::cout << "Phase 8: " << pass_count << "/" << tests.size() << " passed\n";
    return (pass_count == (int)tests.size()) ? 0 : 1;
}
