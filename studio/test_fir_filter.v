// test_fir_filter.v — 32-tap pipelined FIR filter (DSP block)
// Expected: ~12,000 gates (32 multipliers + adder tree + pipeline regs)
// Tests: DSP-style MAC, systolic pipeline, wide multiply-accumulate

module fir_filter #(
    parameter N_TAPS     = 32,
    parameter DATA_WIDTH = 16,
    parameter COEF_WIDTH = 16,
    parameter ACC_WIDTH  = 40
) (
    input                           clk,
    input                           rst_n,
    input  signed [DATA_WIDTH-1:0]  data_in,
    input                           valid_in,
    output reg signed [ACC_WIDTH-1:0] data_out,
    output reg                      valid_out
);
    // Coefficient ROM (hardcoded for symmetric low-pass filter)
    reg signed [COEF_WIDTH-1:0] coeff [0:N_TAPS-1];

    initial begin
        coeff[0]  = 16'sd3;    coeff[1]  = 16'sd8;    coeff[2]  = 16'sd15;   coeff[3]  = 16'sd28;
        coeff[4]  = 16'sd45;   coeff[5]  = 16'sd68;   coeff[6]  = 16'sd95;   coeff[7]  = 16'sd125;
        coeff[8]  = 16'sd155;  coeff[9]  = 16'sd182;  coeff[10] = 16'sd203;  coeff[11] = 16'sd216;
        coeff[12] = 16'sd220;  coeff[13] = 16'sd216;  coeff[14] = 16'sd203;  coeff[15] = 16'sd182;
        coeff[16] = 16'sd155;  coeff[17] = 16'sd125;  coeff[18] = 16'sd95;   coeff[19] = 16'sd68;
        coeff[20] = 16'sd45;   coeff[21] = 16'sd28;   coeff[22] = 16'sd15;   coeff[23] = 16'sd8;
        coeff[24] = 16'sd3;    coeff[25] = 16'sd1;    coeff[26] = 16'sd0;    coeff[27] = -16'sd1;
        coeff[28] = -16'sd2;   coeff[29] = -16'sd2;   coeff[30] = -16'sd1;   coeff[31] = 16'sd0;
    end

    // Delay line (shift register for input samples)
    reg signed [DATA_WIDTH-1:0] delay_line [0:N_TAPS-1];
    reg [N_TAPS-1:0] valid_sr;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N_TAPS; i = i + 1)
                delay_line[i] <= 0;
            valid_sr <= 0;
        end else if (valid_in) begin
            delay_line[0] <= data_in;
            for (i = 1; i < N_TAPS; i = i + 1)
                delay_line[i] <= delay_line[i-1];
            valid_sr <= {valid_sr[N_TAPS-2:0], valid_in};
        end
    end

    // Pipeline stage 1: Multiply (all taps in parallel)
    reg signed [DATA_WIDTH+COEF_WIDTH-1:0] products [0:N_TAPS-1];
    reg v1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N_TAPS; i = i + 1)
                products[i] <= 0;
            v1 <= 0;
        end else begin
            for (i = 0; i < N_TAPS; i = i + 1)
                products[i] <= delay_line[i] * coeff[i];
            v1 <= valid_sr[N_TAPS-1];
        end
    end

    // Pipeline stage 2: Adder tree level 1 (16 pairs)
    reg signed [ACC_WIDTH-1:0] sum_l1 [0:N_TAPS/2-1];
    reg v2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N_TAPS/2; i = i + 1)
                sum_l1[i] <= 0;
            v2 <= 0;
        end else begin
            for (i = 0; i < N_TAPS/2; i = i + 1)
                sum_l1[i] <= products[2*i] + products[2*i+1];
            v2 <= v1;
        end
    end

    // Pipeline stage 3: Adder tree level 2 (8 pairs)
    reg signed [ACC_WIDTH-1:0] sum_l2 [0:N_TAPS/4-1];
    reg v3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N_TAPS/4; i = i + 1)
                sum_l2[i] <= 0;
            v3 <= 0;
        end else begin
            for (i = 0; i < N_TAPS/4; i = i + 1)
                sum_l2[i] <= sum_l1[2*i] + sum_l1[2*i+1];
            v3 <= v2;
        end
    end

    // Pipeline stage 4: Adder tree level 3 (4 pairs)
    reg signed [ACC_WIDTH-1:0] sum_l3 [0:N_TAPS/8-1];
    reg v4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N_TAPS/8; i = i + 1)
                sum_l3[i] <= 0;
            v4 <= 0;
        end else begin
            for (i = 0; i < N_TAPS/8; i = i + 1)
                sum_l3[i] <= sum_l2[2*i] + sum_l2[2*i+1];
            v4 <= v3;
        end
    end

    // Pipeline stage 5: Final accumulation (2 pairs → 1)
    reg signed [ACC_WIDTH-1:0] sum_l4_0, sum_l4_1;
    reg v5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_l4_0 <= 0; sum_l4_1 <= 0; v5 <= 0;
        end else begin
            sum_l4_0 <= sum_l3[0] + sum_l3[1];
            sum_l4_1 <= sum_l3[2] + sum_l3[3];
            v5 <= v4;
        end
    end

    // Pipeline stage 6: Final sum
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out  <= 0;
            valid_out <= 0;
        end else begin
            data_out  <= sum_l4_0 + sum_l4_1;
            valid_out <= v5;
        end
    end
endmodule
