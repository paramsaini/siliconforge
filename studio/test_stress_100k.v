// test_stress_100k.v — 100,000+ gate mega stress test
// Expected: ~100,000 gates — maximum OMP thread + memory pressure test
// Architecture: 4×4 processing element mesh with local interconnect

module stress_100k (
    input         clk,
    input         rst_n,
    input  [63:0] data_in,
    input  [3:0]  pe_sel,
    input  [3:0]  op,
    input         valid_in,
    output [63:0] data_out,
    output        valid_out
);
    // ── 16 Processing Elements, each ~6,000 gates ──
    wire [63:0] pe_out [0:15];
    wire [15:0] pe_valid;
    wire [63:0] pe_fwd [0:15];  // Forward connections between PEs

    genvar r, c;
    generate
        for (r = 0; r < 4; r = r + 1) begin : row
            for (c = 0; c < 4; c = c + 1) begin : col
                localparam IDX = r * 4 + c;
                // Each PE gets data_in when selected, otherwise neighbor forward
                wire [63:0] local_in = (pe_sel == IDX) ? data_in :
                                       (c > 0) ? pe_fwd[IDX-1] : 64'd0;
                wire local_valid = (pe_sel == IDX) ? valid_in : pe_valid[IDX > 0 ? IDX-1 : 0];

                processing_element #(.PE_ID(IDX)) u_pe (
                    .clk(clk), .rst_n(rst_n),
                    .din(local_in), .op(op),
                    .valid_in(local_valid),
                    .dout(pe_out[IDX]),
                    .fwd_out(pe_fwd[IDX]),
                    .valid_out(pe_valid[IDX])
                );
            end
        end
    endgenerate

    // Output mux
    assign data_out  = pe_out[pe_sel];
    assign valid_out = pe_valid[pe_sel];
endmodule

// ── Processing Element (~6,000 gates each) ──
module processing_element #(parameter PE_ID = 0) (
    input         clk,
    input         rst_n,
    input  [63:0] din,
    input  [3:0]  op,
    input         valid_in,
    output reg [63:0] dout,
    output reg [63:0] fwd_out,
    output reg        valid_out
);
    // Local register file (8 × 64-bit = 512 bits of state)
    reg [63:0] rf [0:7];
    reg [2:0]  wr_ptr;

    // Pipeline stage 1: Decode + register read
    reg [63:0] s1_a, s1_b, s1_din;
    reg [3:0]  s1_op;
    reg        s1_v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s1_v <= 0; end
        else begin
            s1_din <= din;
            s1_a   <= rf[din[2:0]];
            s1_b   <= rf[din[5:3]];
            s1_op  <= op;
            s1_v   <= valid_in;
        end
    end

    // Pipeline stage 2: Execute (64-bit ALU + multiply)
    reg [63:0] s2_result;
    reg [63:0] s2_mul_hi;
    reg        s2_v;
    wire [63:0] sum64 = s1_a + s1_b;
    wire [63:0] xor64 = s1_a ^ s1_b;
    wire [63:0] and64 = s1_a & s1_b;
    wire [63:0] or64  = s1_a | s1_b;
    wire [31:0] mul_lo_a = s1_a[31:0], mul_lo_b = s1_b[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s2_v <= 0; s2_result <= 0; end
        else begin
            s2_v <= s1_v;
            case (s1_op)
                4'h0: s2_result <= sum64;
                4'h1: s2_result <= s1_a - s1_b;
                4'h2: s2_result <= mul_lo_a * mul_lo_b;
                4'h3: s2_result <= and64;
                4'h4: s2_result <= or64;
                4'h5: s2_result <= xor64;
                4'h6: s2_result <= s1_a << s1_b[5:0];
                4'h7: s2_result <= s1_a >> s1_b[5:0];
                4'h8: s2_result <= ~s1_a;
                4'h9: s2_result <= (s1_a < s1_b) ? s1_a : s1_b;  // MIN
                4'hA: s2_result <= (s1_a > s1_b) ? s1_a : s1_b;  // MAX
                4'hB: s2_result <= {s1_a[31:0], s1_b[63:32]};     // PACK
                4'hC: s2_result <= s1_a + s1_din;                   // ACC
                4'hD: s2_result <= sum64 ^ and64;                   // FUSED
                4'hE: s2_result <= {s1_a[62:0], s1_a[63]};        // ROL
                4'hF: s2_result <= s1_din;                          // LOAD
            endcase
        end
    end

    // Pipeline stage 3: Writeback + forward
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 0; dout <= 0; fwd_out <= 0; wr_ptr <= 0;
        end else begin
            valid_out <= s2_v;
            dout      <= s2_result;
            fwd_out   <= s2_result;
            if (s2_v) begin
                rf[wr_ptr] <= s2_result;
                wr_ptr     <= wr_ptr + 1;
            end
        end
    end

    // Local accumulator chain (adds ~1000 gates per PE)
    reg [63:0] acc_chain [0:7];
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < 8; k = k + 1) acc_chain[k] <= PE_ID + k;
        end else if (s2_v) begin
            acc_chain[0] <= s2_result;
            for (k = 1; k < 8; k = k + 1)
                acc_chain[k] <= acc_chain[k-1] + acc_chain[k] + rf[k];
        end
    end
endmodule
