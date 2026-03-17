// test_multiplier_16x16.v — 16×16 pipelined array multiplier
// Expected: ~5,000 gates (partial product generation + carry-save adder tree)
// Tests: heavy arithmetic, multi-stage pipeline, wide datapath

module multiplier_16x16 (
    input             clk,
    input             rst_n,
    input      [15:0] a,
    input      [15:0] b,
    input             valid_in,
    output reg [31:0] product,
    output reg        valid_out
);
    // Pipeline stage 1: Generate partial products (4 groups of 4)
    reg [31:0] pp_group0, pp_group1, pp_group2, pp_group3;
    reg        v1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pp_group0 <= 0; pp_group1 <= 0;
            pp_group2 <= 0; pp_group3 <= 0;
            v1 <= 0;
        end else begin
            v1 <= valid_in;
            pp_group0 <= (b[0] ? {16'b0, a}       : 32'b0) +
                         (b[1] ? {15'b0, a, 1'b0}  : 32'b0) +
                         (b[2] ? {14'b0, a, 2'b0}  : 32'b0) +
                         (b[3] ? {13'b0, a, 3'b0}  : 32'b0);

            pp_group1 <= (b[4] ? {12'b0, a, 4'b0}  : 32'b0) +
                         (b[5] ? {11'b0, a, 5'b0}  : 32'b0) +
                         (b[6] ? {10'b0, a, 6'b0}  : 32'b0) +
                         (b[7] ? {9'b0, a, 7'b0}   : 32'b0);

            pp_group2 <= (b[8]  ? {8'b0, a, 8'b0}  : 32'b0) +
                         (b[9]  ? {7'b0, a, 9'b0}  : 32'b0) +
                         (b[10] ? {6'b0, a, 10'b0} : 32'b0) +
                         (b[11] ? {5'b0, a, 11'b0} : 32'b0);

            pp_group3 <= (b[12] ? {4'b0, a, 12'b0} : 32'b0) +
                         (b[13] ? {3'b0, a, 13'b0} : 32'b0) +
                         (b[14] ? {2'b0, a, 14'b0} : 32'b0) +
                         (b[15] ? {1'b0, a, 15'b0} : 32'b0);
        end
    end

    // Pipeline stage 2: Reduce 4 partial sums to 2
    reg [31:0] sum_lo, sum_hi;
    reg        v2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_lo <= 0; sum_hi <= 0; v2 <= 0;
        end else begin
            sum_lo <= pp_group0 + pp_group1;
            sum_hi <= pp_group2 + pp_group3;
            v2 <= v1;
        end
    end

    // Pipeline stage 3: Final addition
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            product <= 0; valid_out <= 0;
        end else begin
            product   <= sum_lo + sum_hi;
            valid_out <= v2;
        end
    end
endmodule
