`timescale 1ns / 1ps

module test_v100_final (
    input wire clk,
    input wire rst,
    input wire [7:0] a,
    input wire [7:0] b,
    input wire signed [7:0] sa,
    input wire signed [7:0] sb,
    input wire [1:0] sel,
    input wire en,
    output reg [7:0] result,
    output reg [7:0] alu_out,
    output wire [7:0] tri_out,
    output reg [7:0] xnor_out,
    output reg signed_lt_out
);

    // Parameters with ** power operator
    parameter WIDTH = 8;
    parameter DEPTH = 2**4;   // 16
    localparam ADDR_W = $clog2(DEPTH);  // 4
    localparam HALF = WIDTH / 2;

    // === and !== case equality (maps to ==/!= for synth)
    wire eq_exact = (a === b) ? 1'b1 : 1'b0;
    wire neq_exact = (a !== b) ? 1'b1 : 1'b0;

    // ~^ XNOR operator
    assign xnor_out = a ~^ b;

    // Signed comparison — auto-propagated from signed declarations
    wire auto_signed_lt = (sa < sb) ? 1'b1 : 1'b0;

    // Tri-state buffer
    assign tri_out = en ? a : 8'bz;

    // Task definition
    task clamp_add;
        input [7:0] x;
        input [7:0] y;
        output [7:0] z;
        z = x + y;
    endtask

    // Function with automatic
    function automatic [7:0] saturate;
        input [7:0] val;
        input [7:0] maxval;
        saturate = (val > maxval) ? maxval : val;
    endfunction

    // ALU with casex don't-care + indexed part-select
    always @(posedge clk) begin
        if (rst) begin
            result <= 8'h00;
            alu_out <= 8'h00;
            signed_lt_out <= 1'b0;
        end else begin
            // casex with don't-care
            casex (sel)
                2'b00: result <= a + b;
                2'b01: result <= a - b;
                2'b1x: result <= a & b;
                default: result <= 8'h00;
            endcase

            // Indexed part-select on RHS
            alu_out[3:0] <= a[0 +: 4];
            alu_out[7:4] <= b[4 +: 4];

            // Signed comparison output
            signed_lt_out <= auto_signed_lt;
        end
    end

endmodule
