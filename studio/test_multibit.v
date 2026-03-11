// test_multibit.v — Multi-bit bus test for Stage 1 validation
// Tests: bus declarations, bitwise ops, ripple-carry adder, case statement

module multibit_alu (
    input clk,
    input [3:0] a,
    input [3:0] b,
    input [1:0] sel,
    output [3:0] result
);
    wire [3:0] and_out;
    wire [3:0] or_out;
    wire [3:0] sum_out;

    // Bitwise AND
    assign and_out = a & b;

    // Bitwise OR
    assign or_out = a | b;

    // 4-bit ripple-carry adder
    assign sum_out = a + b;

    // Case-based MUX: select which result to register
    always @(posedge clk) begin
        case (sel)
            2'b00: result <= and_out;
            2'b01: result <= or_out;
            2'b10: result <= sum_out;
            default: result <= a;
        endcase
    end

endmodule
