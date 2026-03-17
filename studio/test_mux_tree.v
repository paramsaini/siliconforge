// test_mux_tree.v — 16:1 MUX tree built from 2:1 MUX primitives
// Expected: ~200 gates (15 × 2:1 MUXes × 8-bit width)
// Tests: deep module hierarchy (4 levels), generate for replication

module mux2to1 #(parameter W = 8) (
    input  [W-1:0] a,
    input  [W-1:0] b,
    input          sel,
    output [W-1:0] y
);
    assign y = sel ? b : a;
endmodule

module mux4to1 #(parameter W = 8) (
    input  [W-1:0] in0, in1, in2, in3,
    input  [1:0]   sel,
    output [W-1:0] y
);
    wire [W-1:0] m0, m1;

    mux2to1 #(W) u0 (.a(in0), .b(in1), .sel(sel[0]), .y(m0));
    mux2to1 #(W) u1 (.a(in2), .b(in3), .sel(sel[0]), .y(m1));
    mux2to1 #(W) u2 (.a(m0),  .b(m1),  .sel(sel[1]), .y(y));
endmodule

module mux16to1 #(parameter W = 8) (
    input  [W-1:0] d0, d1, d2, d3,
    input  [W-1:0] d4, d5, d6, d7,
    input  [W-1:0] d8, d9, d10, d11,
    input  [W-1:0] d12, d13, d14, d15,
    input  [3:0]   sel,
    output [W-1:0] y
);
    wire [W-1:0] q0, q1, q2, q3;

    mux4to1 #(W) m0 (.in0(d0),  .in1(d1),  .in2(d2),  .in3(d3),  .sel(sel[1:0]), .y(q0));
    mux4to1 #(W) m1 (.in0(d4),  .in1(d5),  .in2(d6),  .in3(d7),  .sel(sel[1:0]), .y(q1));
    mux4to1 #(W) m2 (.in0(d8),  .in1(d9),  .in2(d10), .in3(d11), .sel(sel[1:0]), .y(q2));
    mux4to1 #(W) m3 (.in0(d12), .in1(d13), .in2(d14), .in3(d15), .sel(sel[1:0]), .y(q3));

    mux4to1 #(W) mf (.in0(q0),  .in1(q1),  .in2(q2),  .in3(q3),  .sel(sel[3:2]), .y(y));
endmodule
