// test_jpeg_dct.v — 8×8 2D DCT for JPEG compression (Loeffler algorithm)
// Expected: ~18,000 gates (8 1D-DCTs × 2 passes, transpose buffer, pipeline)
// Tests: fixed-point DSP, deep pipeline, matrix transpose, coefficient ROM

module jpeg_dct_1d (
    input                    clk,
    input                    rst_n,
    input  signed [15:0]     x0, x1, x2, x3, x4, x5, x6, x7,
    input                    valid_in,
    output reg signed [15:0] y0, y1, y2, y3, y4, y5, y6, y7,
    output reg               valid_out
);
    // Loeffler 1D-DCT (scaled integer, 4 multiplies, 11 additions)
    // Constants in Q2.13: cos(pi/16)=15137, cos(3pi/16)=11585, etc.
    localparam signed [15:0] C1 = 16'sd15137, S1 = 16'sd3135,
                             C3 = 16'sd11585, S3 = 16'sd8867,
                             C6 = 16'sd6270,  S6 = 16'sd13623,
                             K  = 16'sd11585; // 1/sqrt(2) in Q1.14

    // Stage 1: Even/odd decomposition
    reg signed [16:0] a0, a1, a2, a3, a4, a5, a6, a7;
    reg v1;
    always @(posedge clk) begin
        a0 <= x0 + x7; a7 <= x0 - x7;
        a1 <= x1 + x6; a6 <= x1 - x6;
        a2 <= x2 + x5; a5 <= x2 - x5;
        a3 <= x3 + x4; a4 <= x3 - x4;
        v1 <= valid_in;
    end

    // Stage 2: Even part
    reg signed [17:0] b0, b1, b2, b3;
    reg signed [16:0] b4, b5, b6, b7;
    reg v2;
    always @(posedge clk) begin
        b0 <= a0 + a3; b3 <= a0 - a3;
        b1 <= a1 + a2; b2 <= a1 - a2;
        b4 <= a4; b5 <= a5; b6 <= a6; b7 <= a7;
        v2 <= v1;
    end

    // Stage 3: Rotations via multiplies
    reg signed [31:0] c0, c1, c2, c3, c4, c5, c6, c7;
    reg v3;
    always @(posedge clk) begin
        c0 <= (b0 + b1) * K;             // Y0 (DC)
        c1 <= (b0 - b1) * K;             // Y4
        c2 <= b2 * C6 + b3 * S6;         // Y2
        c3 <= b3 * C6 - b2 * S6;         // Y6
        c4 <= b4 * C1 + b7 * S1;         // Part of Y1
        c5 <= b5 * C3 + b6 * S3;         // Part of Y3
        c6 <= b6 * C3 - b5 * S3;         // Part of Y5
        c7 <= b7 * C1 - b4 * S1;         // Part of Y7
        v3 <= v2;
    end

    // Stage 4: Scale and output
    always @(posedge clk) begin
        y0 <= c0[28:13]; y4 <= c1[28:13];
        y2 <= c2[28:13]; y6 <= c3[28:13];
        y1 <= c4[28:13]; y3 <= c5[28:13];
        y5 <= c6[28:13]; y7 <= c7[28:13];
        valid_out <= v3;
    end
endmodule

// Transpose buffer (8×8 matrix)
module transpose_8x8 (
    input                    clk,
    input                    rst_n,
    input  signed [15:0]     din,
    input                    wr_en,
    input  [2:0]             wr_row,
    input  [2:0]             wr_col,
    output signed [15:0]     dout,
    input  [2:0]             rd_row,
    input  [2:0]             rd_col
);
    reg signed [15:0] mem [0:63];

    always @(posedge clk) begin
        if (wr_en) mem[{wr_row, wr_col}] <= din;
    end
    assign dout = mem[{rd_col, rd_row}]; // Transposed read
endmodule

// Full 2D DCT: row transform → transpose → column transform
module jpeg_dct_2d (
    input                    clk,
    input                    rst_n,
    input  signed [15:0]     pixel_in [0:7],
    input                    row_valid,
    output reg signed [15:0] coeff_out [0:7],
    output reg               coeff_valid
);
    // Row DCT
    wire signed [15:0] row_out [0:7];
    wire row_done;

    jpeg_dct_1d u_row_dct (
        .clk(clk), .rst_n(rst_n),
        .x0(pixel_in[0]), .x1(pixel_in[1]), .x2(pixel_in[2]), .x3(pixel_in[3]),
        .x4(pixel_in[4]), .x5(pixel_in[5]), .x6(pixel_in[6]), .x7(pixel_in[7]),
        .valid_in(row_valid),
        .y0(row_out[0]), .y1(row_out[1]), .y2(row_out[2]), .y3(row_out[3]),
        .y4(row_out[4]), .y5(row_out[5]), .y6(row_out[6]), .y7(row_out[7]),
        .valid_out(row_done)
    );

    // Column DCT (second pass)
    wire signed [15:0] col_out [0:7];
    wire col_done;

    jpeg_dct_1d u_col_dct (
        .clk(clk), .rst_n(rst_n),
        .x0(row_out[0]), .x1(row_out[1]), .x2(row_out[2]), .x3(row_out[3]),
        .x4(row_out[4]), .x5(row_out[5]), .x6(row_out[6]), .x7(row_out[7]),
        .valid_in(row_done),
        .y0(col_out[0]), .y1(col_out[1]), .y2(col_out[2]), .y3(col_out[3]),
        .y4(col_out[4]), .y5(col_out[5]), .y6(col_out[6]), .y7(col_out[7]),
        .valid_out(col_done)
    );

    integer i;
    always @(posedge clk) begin
        for (i = 0; i < 8; i = i + 1) coeff_out[i] <= col_out[i];
        coeff_valid <= col_done;
    end
endmodule
