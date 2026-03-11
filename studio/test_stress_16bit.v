// test_stress_16bit.v
// Heavy-complexity 16-bit pipelined datapath stress test
// Expected: hundreds of gates from bit-blasted bus operations
//
// Architecture:
//   Layer 1: Four 16-bit adders + four 16-bit bitwise ops
//   Layer 2: Two 16-bit adders combining Layer 1 results
//   Layer 3: One 16-bit adder + XOR combining Layer 2
//   Layer 4: Pipeline DFFs and FSM control
//
// Estimated pre-synth: ~700+ gates

module stress_datapath(
    input clk,
    input [15:0] a,
    input [15:0] b,
    input [15:0] c,
    input [15:0] d,
    input mode,
    output [15:0] sum_ab,
    output [15:0] sum_cd,
    output [15:0] diff_ac,
    output [15:0] diff_bd,
    output [15:0] and_ab,
    output [15:0] or_cd,
    output [15:0] xor_ac,
    output [15:0] xor_bd,
    output [15:0] stage2_sum,
    output [15:0] stage2_xor,
    output [15:0] stage3_out,
    output [15:0] stage3_check,
    output pipeline_valid
);

    // ────────── Layer 1: Parallel arithmetic + logic ──────────

    // Four 16-bit adders (~77 gates each = ~308 gates)
    assign sum_ab = a + b;
    assign sum_cd = c + d;
    assign diff_ac = a + c;
    assign diff_bd = b + d;

    // Four 16-bit bitwise operations (16 gates each = 64 gates)
    assign and_ab = a & b;
    assign or_cd  = c | d;
    assign xor_ac = a ^ c;
    assign xor_bd = b ^ d;

    // ────────── Layer 2: Combine Layer 1 results ──────────

    // Two 16-bit adders combining sums (~154 gates)
    assign stage2_sum = sum_ab + sum_cd;
    assign stage2_xor = xor_ac ^ xor_bd;

    // ────────── Layer 3: Final combine ──────────

    // 16-bit adder + XOR for final result (~93 gates)
    assign stage3_out   = stage2_sum + stage2_xor;
    assign stage3_check = and_ab ^ or_cd;

    // ────────── Layer 4: Pipeline control (behavioral) ──────────

    always @(posedge clk) begin
        if (mode == 1) begin
            pipeline_valid <= 1;
        end else begin
            pipeline_valid <= 0;
        end
    end

endmodule
