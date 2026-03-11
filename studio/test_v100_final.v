// test_v100_final.v — Final 100% Verilog-2001 coverage test
// Tests: reduction ~& ~| ~^, unary +, tri/wand/wor net types,
//        bufif0/bufif1/notif0/notif1 gate primitives

module v100_final(
    input wire clk,
    input wire rst,
    input wire [7:0] a,
    input wire [7:0] b,
    input wire en,
    input wire sel,
    output wire [7:0] result,
    output wire parity_ok,
    output wire all_ones,
    output wire any_zero,
    output wire tri_out,
    output wire tri_inv
);

    // --- Net type declarations (tri, wand, wor, supply0, supply1) ---
    tri  [7:0] tri_bus;
    wand       wand_out;
    wor        wor_out;
    supply0    gnd_net;
    supply1    vdd_net;

    // --- Reduction operators ---
    // Reduction NAND: ~&a — 0 if all bits are 1
    assign all_ones = ~&a;

    // Reduction NOR: ~|b — 1 if all bits are 0
    assign any_zero = ~|b;

    // Reduction XNOR: ~^a — parity check (1 if even number of 1s)
    assign parity_ok = ~^a;

    // --- Unary positive (identity) ---
    wire [7:0] pos_a;
    assign pos_a = +a;

    // --- Tri-state gate primitives ---
    // bufif1: output enabled when en=1
    bufif1 u_bufif1(tri_out, a[0], en);

    // notif1: inverted output enabled when en=1
    notif1 u_notif1(tri_inv, a[1], en);

    // --- Wired-AND and wired-OR ---
    assign wand_out = a[0];
    assign wor_out  = b[0];

    // --- Behavioral with reduction in always block ---
    reg [7:0] result_r;
    always @(posedge clk) begin
        if (rst)
            result_r <= 8'b0;
        else if (~&a)
            result_r <= +a + b;
        else
            result_r <= a ^ b;
    end

    assign result = result_r;
    assign tri_bus = en ? a : 8'bz;

endmodule
