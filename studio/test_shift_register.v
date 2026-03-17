// test_shift_register.v — 16-bit barrel shifter + serial shift register
// Expected: ~350 gates (barrel shifter ~256 MUX gates, shift reg ~160 DFFs)
// Tests: generate blocks, parameterized design, serial/parallel conversion

module barrel_shifter #(parameter WIDTH = 16) (
    input  [WIDTH-1:0] data_in,
    input  [3:0]       shift_amt,
    input              shift_dir,   // 0=left, 1=right
    input              arith,       // 0=logical, 1=arithmetic
    output [WIDTH-1:0] data_out
);
    wire [WIDTH-1:0] stage [0:3];
    wire             fill_bit = arith & shift_dir & data_in[WIDTH-1];

    // Stage 0: shift by 0 or 1
    genvar i;
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : s0
            assign stage[0][i] = shift_amt[0] ?
                (shift_dir ? (i == WIDTH-1 ? fill_bit : data_in[i+1]) : (i == 0 ? 1'b0 : data_in[i-1]))
                : data_in[i];
        end
    endgenerate

    // Stage 1: shift by 0 or 2
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : s1
            assign stage[1][i] = shift_amt[1] ?
                (shift_dir ? (i >= WIDTH-2 ? fill_bit : stage[0][i+2]) : (i < 2 ? 1'b0 : stage[0][i-2]))
                : stage[0][i];
        end
    endgenerate

    // Stage 2: shift by 0 or 4
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : s2
            assign stage[2][i] = shift_amt[2] ?
                (shift_dir ? (i >= WIDTH-4 ? fill_bit : stage[1][i+4]) : (i < 4 ? 1'b0 : stage[1][i-4]))
                : stage[1][i];
        end
    endgenerate

    // Stage 3: shift by 0 or 8
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : s3
            assign stage[3][i] = shift_amt[3] ?
                (shift_dir ? (i >= WIDTH-8 ? fill_bit : stage[2][i+8]) : (i < 8 ? 1'b0 : stage[2][i-8]))
                : stage[2][i];
        end
    endgenerate

    assign data_out = stage[3];
endmodule

module shift_register_16 (
    input         clk,
    input         rst_n,
    input         serial_in,
    input         load,
    input  [15:0] parallel_in,
    output        serial_out,
    output [15:0] parallel_out
);
    reg [15:0] sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sr <= 16'h0000;
        else if (load)
            sr <= parallel_in;
        else
            sr <= {sr[14:0], serial_in};
    end

    assign serial_out   = sr[15];
    assign parallel_out = sr;
endmodule
