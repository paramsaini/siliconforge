// Resource Sharing Test: Mutually Exclusive Adders
// Naive synthesis creates Two adders.
// Optimized CDFG synthesis creates One adder with multiplexed inputs.
module resource_fsm(
    input wire clk,
    input wire sel,
    input wire [7:0] a,
    input wire [7:0] b,
    input wire [7:0] c,
    input wire [7:0] d,
    output reg [7:0] out
);

    always @(posedge clk) begin
        if (sel) begin
            out <= a + b;
        end else begin
            out <= c + d;
        end
    end

endmodule
