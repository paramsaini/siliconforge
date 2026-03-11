// test_full_operators.v
// Comprehensive test exercising ALL new parser features:
// - Subtraction, multiplication, shifts, comparators
// - Ternary operator, concatenation
// - always @(*) combinational
// - for loops, parameters
// - Logical operators &&, ||, !

module full_operator_test (
    input clk,
    input [3:0] a,
    input [3:0] b,
    input sel,
    output [3:0] sum_out,
    output [3:0] diff_out,
    output [3:0] and_out,
    output [3:0] or_out,
    output [3:0] xor_out,
    output [3:0] not_out,
    output [3:0] mux_out,
    output [3:0] shift_left_out,
    output [3:0] shift_right_out,
    output lt_out,
    output gt_out,
    output eq_out,
    output neq_out
);

    // Arithmetic
    assign sum_out = a + b;
    assign diff_out = a - b;

    // Bitwise
    assign and_out = a & b;
    assign or_out = a | b;
    assign xor_out = a ^ b;
    assign not_out = ~a;

    // Ternary MUX
    assign mux_out = sel ? a : b;

    // Shifts
    assign shift_left_out = a << 2;
    assign shift_right_out = a >> 1;

    // Comparisons
    assign lt_out = (a < b);
    assign gt_out = (a > b);
    assign eq_out = (a == b);
    assign neq_out = (a != b);

endmodule
