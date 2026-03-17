// test_gpu_shader.v — 4-lane SIMD shader core (GPU-style compute unit)
// Expected: ~30,000 gates (4 ALU lanes, register file, scheduler, load/store)
// Tests: SIMD parallelism, wide register file, instruction decode, scoreboard

module gpu_shader #(
    parameter LANES    = 4,
    parameter REG_BITS = 5,     // 32 vector registers
    parameter DATA_W   = 32
) (
    input                clk,
    input                rst_n,
    // Instruction fetch
    output [31:0]        pc_out,
    input  [31:0]        instr_in,
    input                instr_valid,
    // Memory interface (per-lane)
    output [LANES*32-1:0] mem_addr_flat,
    output [LANES*32-1:0] mem_wdata_flat,
    output [LANES-1:0]    mem_wr,
    output [LANES-1:0]    mem_rd,
    input  [LANES*32-1:0] mem_rdata_flat,
    input  [LANES-1:0]    mem_ready,
    // Control
    input                 stall_in,
    output                busy
);
    // ── Instruction Format ──
    // [31:28] opcode  [27:23] rd  [22:18] rs1  [17:13] rs2  [12:0] imm13
    wire [3:0]  opcode = instr_in[31:28];
    wire [4:0]  rd     = instr_in[27:23];
    wire [4:0]  rs1    = instr_in[22:18];
    wire [4:0]  rs2    = instr_in[17:13];
    wire [12:0] imm13  = instr_in[12:0];
    wire [31:0] imm_ext = {{19{imm13[12]}}, imm13};

    localparam OP_ADD  = 4'h0, OP_SUB  = 4'h1, OP_MUL  = 4'h2, OP_AND  = 4'h3,
               OP_OR   = 4'h4, OP_XOR  = 4'h5, OP_SHL  = 4'h6, OP_SHR  = 4'h7,
               OP_ADDI = 4'h8, OP_LD   = 4'h9, OP_ST   = 4'hA, OP_MOV  = 4'hB,
               OP_CMP  = 4'hC, OP_MAD  = 4'hD, OP_NOP  = 4'hF;

    // ── Vector Register File (32 regs × 4 lanes × 32 bits) ──
    reg [DATA_W-1:0] vrf [0:31][0:LANES-1];

    // ── Program Counter ──
    reg [31:0] pc;
    assign pc_out = pc;
    assign busy = (state != ST_IDLE);

    // ── Pipeline state ──
    localparam [2:0] ST_IDLE    = 3'd0,
                     ST_DECODE  = 3'd1,
                     ST_EXECUTE = 3'd2,
                     ST_MEMORY  = 3'd3,
                     ST_MWAIT   = 3'd4,
                     ST_WRITEBACK = 3'd5;

    reg [2:0]  state;
    reg [3:0]  ex_op;
    reg [4:0]  ex_rd, ex_rs1, ex_rs2;
    reg [31:0] ex_imm;
    reg [DATA_W-1:0] rs1_data [0:LANES-1];
    reg [DATA_W-1:0] rs2_data [0:LANES-1];
    reg [DATA_W-1:0] result   [0:LANES-1];
    reg [DATA_W-1:0] mem_addr_r [0:LANES-1];
    reg              do_mem_rd, do_mem_wr;

    integer lane, k;

    // Pack/unpack memory flat buses
    genvar g;
    generate
        for (g = 0; g < LANES; g = g + 1) begin : mem_io
            assign mem_addr_flat[g*32 +: 32]  = mem_addr_r[g];
            assign mem_wdata_flat[g*32 +: 32] = rs2_data[g];
            assign mem_wr[g] = (state == ST_MEMORY) & do_mem_wr;
            assign mem_rd[g] = (state == ST_MEMORY) & do_mem_rd;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; pc <= 0;
            ex_op <= OP_NOP; do_mem_rd <= 0; do_mem_wr <= 0;
            for (k = 0; k < 32; k = k + 1)
                for (lane = 0; lane < LANES; lane = lane + 1)
                    vrf[k][lane] <= 0;
        end else if (!stall_in) begin
            case (state)
                ST_IDLE: begin
                    if (instr_valid) begin
                        state <= ST_DECODE;
                        pc <= pc + 4;
                    end
                end

                ST_DECODE: begin
                    ex_op  <= opcode;
                    ex_rd  <= rd;
                    ex_rs1 <= rs1;
                    ex_rs2 <= rs2;
                    ex_imm <= imm_ext;
                    for (lane = 0; lane < LANES; lane = lane + 1) begin
                        rs1_data[lane] <= vrf[rs1][lane];
                        rs2_data[lane] <= vrf[rs2][lane];
                    end
                    do_mem_rd <= (opcode == OP_LD);
                    do_mem_wr <= (opcode == OP_ST);
                    state <= ST_EXECUTE;
                end

                ST_EXECUTE: begin
                    for (lane = 0; lane < LANES; lane = lane + 1) begin
                        case (ex_op)
                            OP_ADD:  result[lane] <= rs1_data[lane] + rs2_data[lane];
                            OP_SUB:  result[lane] <= rs1_data[lane] - rs2_data[lane];
                            OP_MUL:  result[lane] <= rs1_data[lane] * rs2_data[lane];
                            OP_AND:  result[lane] <= rs1_data[lane] & rs2_data[lane];
                            OP_OR:   result[lane] <= rs1_data[lane] | rs2_data[lane];
                            OP_XOR:  result[lane] <= rs1_data[lane] ^ rs2_data[lane];
                            OP_SHL:  result[lane] <= rs1_data[lane] << rs2_data[lane][4:0];
                            OP_SHR:  result[lane] <= rs1_data[lane] >> rs2_data[lane][4:0];
                            OP_ADDI: result[lane] <= rs1_data[lane] + ex_imm;
                            OP_MOV:  result[lane] <= ex_imm;
                            OP_CMP:  result[lane] <= (rs1_data[lane] < rs2_data[lane]) ? 32'd1 : 32'd0;
                            OP_MAD:  result[lane] <= rs1_data[lane] * rs2_data[lane] + vrf[ex_rd][lane]; // multiply-add
                            default: result[lane] <= 0;
                        endcase
                        mem_addr_r[lane] <= rs1_data[lane] + ex_imm;
                    end
                    if (do_mem_rd || do_mem_wr)
                        state <= ST_MEMORY;
                    else
                        state <= ST_WRITEBACK;
                end

                ST_MEMORY: begin
                    state <= ST_MWAIT;
                end

                ST_MWAIT: begin
                    if (&mem_ready) begin
                        if (do_mem_rd) begin
                            for (lane = 0; lane < LANES; lane = lane + 1)
                                result[lane] <= mem_rdata_flat[lane*32 +: 32];
                        end
                        state <= ST_WRITEBACK;
                    end
                end

                ST_WRITEBACK: begin
                    if (ex_op != OP_ST && ex_op != OP_NOP && ex_rd != 0) begin
                        for (lane = 0; lane < LANES; lane = lane + 1)
                            vrf[ex_rd][lane] <= result[lane];
                    end
                    do_mem_rd <= 0;
                    do_mem_wr <= 0;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
