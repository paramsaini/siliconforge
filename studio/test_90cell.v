// SiliconForge Test — targets ~100-120 post-synthesis cells
`timescale 1ns / 1ps

module test_90cell(
    input wire clk,
    input wire rst,
    input wire [7:0] a, b, c, d,
    input wire [1:0] sel,
    input wire en,
    output wire [7:0] w0, w1, w2, w3, w4,
    output wire [7:0] w5, w6, w7, w8,
    output reg [7:0] r0, r1
);

    // 9 independent XOR-heavy assign cones (AIG-resistant)
    assign w0 = a ^ b ^ c;
    assign w1 = a ^ {b[6:0], b[7]} ^ d;
    assign w2 = (a & b) | (b & c) | (a & c);
    assign w3 = {c[0], c[7:1]} ^ d ^ {a[3:0], a[7:4]};
    assign w4 = (a ^ d) & (b ^ c);
    assign w5 = (a | b) ^ (c & d);
    assign w6 = en ? (a ^ b) : (c ^ d);
    assign w7 = a ^ b ^ {c[6:0], c[7]} ^ {d[0], d[7:1]};
    assign w8 = (a & ~b) ^ (c & ~d);

    // Registered section (FFs for CTS/STA)
    always @(posedge clk) begin
        if (rst) begin
            r0 <= 8'h00;
            r1 <= 8'h00;
        end else begin
            r0 <= w0 ^ w2;
            r1 <= w1 ^ w3;
        end
    end

endmodule
