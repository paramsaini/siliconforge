// ============================================================
// SiliconForge — SV Phase 6 Tests (Advanced Types & Verification Parse/Skip)
// Streaming operators, union packed, multi-dim packed arrays,
// parameter type, SVA assert/assume/cover, clocking, final, property, sequence
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

// Helper: parse and check success
static VerilogParseResult parse_sv(const std::string& src) {
    Netlist nl;
    VerilogParser parser;
    auto r = parser.parse_string(src, nl);
    return r;
}

static std::pair<VerilogParseResult, Netlist> parse_sv_nl(const std::string& src) {
    Netlist nl;
    VerilogParser parser;
    auto r = parser.parse_string(src, nl);
    return {r, std::move(nl)};
}

// ============================================================
// 1. Streaming operator {<<{expr}} — bit reverse
// ============================================================
TEST(streaming_bit_reverse) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [7:0] a, output [7:0] y);
            assign y = {<<{a}};
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_inputs == 8);
    ASSERT_TRUE(r.num_outputs == 8);
    pass_count++;
}

// ============================================================
// 2. Streaming operator {>>{expr}} — pass through
// ============================================================
TEST(streaming_passthrough) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [7:0] a, output [7:0] y);
            assign y = {>>{a}};
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 3. Streaming with slice width {<<8{expr}}
// ============================================================
TEST(streaming_byte_reverse) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [31:0] a, output [31:0] y);
            assign y = {<<8{a}};
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 4. Union packed typedef
// ============================================================
TEST(union_packed_basic) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [31:0] in_data, output [31:0] out_data);
            typedef union packed {
                logic [31:0] word;
                logic [3:0][7:0] bytes;
            } data_u;
            assign out_data = in_data;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 5. Union packed — width = max of fields
// ============================================================
TEST(union_packed_width_max) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [15:0] a, output [15:0] y);
            typedef union packed {
                logic [15:0] half;
                logic [7:0] byte_val;
            } mixed_u;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 6. Multi-dimensional packed array — basic
// ============================================================
TEST(multidim_packed_basic) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [3:0][7:0] data, output [31:0] out);
            assign out = data;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_inputs == 32); // 4*8 = 32
    pass_count++;
}

// ============================================================
// 7. Multi-dimensional packed — output
// ============================================================
TEST(multidim_packed_output) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [31:0] in, output [1:0][15:0] out);
            assign out = in;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_outputs == 32); // 2*16 = 32
    pass_count++;
}

// ============================================================
// 8. Multi-dimensional packed — wire
// ============================================================
TEST(multidim_packed_wire) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [31:0] in, output [31:0] out);
            wire [3:0][7:0] tmp;
            assign tmp = in;
            assign out = tmp;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 9. Parameter type
// ============================================================
TEST(parameter_type_basic) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [7:0] a, output [7:0] y);
            parameter type T = logic;
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 10. Parameter type with range
// ============================================================
TEST(parameter_type_range) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [15:0] a, output [15:0] y);
            parameter type DATA_T = logic [15:0];
            assign y = a;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 11. SVA immediate assert — skip
// ============================================================
TEST(sva_immediate_assert) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input [7:0] a, output reg [7:0] q);
            always @(posedge clk) begin
                q <= a;
                assert(a != 8'hFF);
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 12. SVA assume — skip
// ============================================================
TEST(sva_assume) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input [7:0] a, output reg [7:0] q);
            always @(posedge clk) begin
                assume(a > 0);
                q <= a;
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 13. SVA cover — skip
// ============================================================
TEST(sva_cover) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input a, output reg q);
            always @(posedge clk) begin
                cover(a == 1);
                q <= a;
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 14. SVA concurrent assert property — module level skip
// ============================================================
TEST(sva_concurrent_assert) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input a, output reg q);
            always @(posedge clk) q <= a;
            assert property (@(posedge clk) a);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 15. SVA assume property — module level skip
// ============================================================
TEST(sva_assume_property) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input [3:0] a, output reg [3:0] q);
            always @(posedge clk) q <= a;
            assume property (@(posedge clk) a != 0);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 16. SVA cover property — module level skip
// ============================================================
TEST(sva_cover_property) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input a, input b, output reg q);
            always @(posedge clk) q <= a & b;
            cover property (@(posedge clk) a && b);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 17. Property block — parse and skip
// ============================================================
TEST(property_block_skip) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input a, input b, output reg q);
            always @(posedge clk) q <= a;
            property p_valid;
                @(posedge clk) a |-> b;
            endproperty
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 18. Sequence block — parse and skip
// ============================================================
TEST(sequence_block_skip) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input a, input b, output reg q);
            always @(posedge clk) q <= a & b;
            sequence s_handshake;
                a ##1 b;
            endsequence
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 19. Clocking block — parse and skip
// ============================================================
TEST(clocking_block_skip) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input clk, input a, output reg q);
            always @(posedge clk) q <= a;
            clocking cb @(posedge clk);
                input a;
                output q;
            endclocking
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 20. Final block — parse and skip
// ============================================================
TEST(final_block_skip) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input a, output y);
            assign y = a;
            final begin
                $display("Simulation done");
            end
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 21. Final single statement — skip
// ============================================================
TEST(final_single_stmt_skip) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input a, output y);
            assign y = a;
            final $display("done");
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 22. Combined: struct + union in same module
// ============================================================
TEST(struct_union_combined) {
    auto [r, nl] = parse_sv_nl(R"(
        module top(input [31:0] in, output [31:0] out);
            typedef struct packed {
                logic [15:0] hi;
                logic [15:0] lo;
            } split_t;
            typedef union packed {
                logic [31:0] word;
                split_t halves;
            } data_u;
            assign out = in;
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 23. Combined: SVA + always in same module
// ============================================================
TEST(sva_with_always) {
    auto r = parse_sv(R"(
        module top(input clk, input rst, input [7:0] a, output reg [7:0] q);
            always @(posedge clk) begin
                if (rst) q <= 0;
                else q <= a;
            end
            assert property (@(posedge clk) disable iff (rst) a != 0);
            cover property (@(posedge clk) q == 8'hFF);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 24. Property + assert property in same module
// ============================================================
TEST(property_then_assert) {
    auto r = parse_sv(R"(
        module top(input clk, input a, input b, output reg q);
            always @(posedge clk) q <= a ^ b;
            property p_xor;
                @(posedge clk) a |-> ##1 b;
            endproperty
            assert property (p_xor);
        endmodule
    )");
    ASSERT_TRUE(r.success);
    pass_count++;
}

// ============================================================
// 25. Full featured: multi-dim + union + SVA + clocking + final
// ============================================================
TEST(full_featured_module) {
    Netlist nl;
    VerilogParser parser;
    auto r = parser.parse_string(R"(
        module top(input clk, input [1:0][7:0] data_in, output reg [15:0] data_out);
            typedef union packed {
                logic [15:0] word;
                logic [1:0][7:0] bytes;
            } packet_u;
            parameter type T = logic [15:0];
            always @(posedge clk) begin
                data_out <= data_in;
                assert(data_in != 0);
            end
            property p_nonzero;
                @(posedge clk) data_in != 0;
            endproperty
            sequence s_toggle;
                data_out ##1 ~data_out;
            endsequence
            clocking cb @(posedge clk);
                input data_in;
                output data_out;
            endclocking
            final begin
                $display("test done");
            end
        endmodule
    )", nl);
    ASSERT_TRUE(r.success);
    ASSERT_TRUE(r.num_inputs == 17); // clk(1) + data_in(2*8=16)
    pass_count++;
}

int main() {
    std::cout << "=== SV Phase 6 Tests (Advanced Types & Verification) ===\n";
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
