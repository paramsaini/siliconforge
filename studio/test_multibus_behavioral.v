// test_multibus_behavioral.v
// Tests multi-bit bus operations inside always @(posedge clk) blocks
// This is the critical gap that was fixed — buses in behavioral synthesis

module multibus_behavioral (
    input clk,
    input [7:0] a,
    input [7:0] b,
    input sel,
    output [7:0] sum_out,
    output [7:0] diff_out,
    output [7:0] mux_out,
    output overflow
);
    reg [7:0] sum_out;
    reg [7:0] diff_out;
    reg [7:0] mux_out;
    reg overflow;

    always @(posedge clk) begin
        // Multi-bit addition in always block
        sum_out <= a + b;

        // Multi-bit subtraction in always block
        diff_out <= a - b;

        // Multi-bit MUX via ternary in always block
        mux_out <= sel ? a : b;

        // Multi-bit comparison returning single bit
        overflow <= (a + b) > 8'd255 ? 1'b1 : 1'b0;
    end

endmodule
