// test_riscv_core.v — 5-stage pipelined RISC-V RV32I core (simplified)
// Expected: ~15,000 gates (register file, ALU, control, forwarding, hazard)
// Tests: deep pipeline, data hazard forwarding, branch prediction, stalling

module riscv_core (
    input         clk,
    input         rst_n,
    // Instruction memory interface
    output [31:0] imem_addr,
    input  [31:0] imem_data,
    // Data memory interface
    output [31:0] dmem_addr,
    output [31:0] dmem_wdata,
    output        dmem_wen,
    output [3:0]  dmem_be,
    input  [31:0] dmem_rdata,
    output        dmem_ren
);
    // ═══════════════════════════════════════════════════════
    // Pipeline registers
    // ═══════════════════════════════════════════════════════

    // IF/ID
    reg [31:0] ifid_pc, ifid_instr;
    reg        ifid_valid;

    // ID/EX
    reg [31:0] idex_pc, idex_rs1_data, idex_rs2_data, idex_imm;
    reg [4:0]  idex_rd, idex_rs1, idex_rs2;
    reg [3:0]  idex_alu_op;
    reg [2:0]  idex_funct3;
    reg        idex_alu_src, idex_mem_read, idex_mem_write;
    reg        idex_reg_write, idex_mem_to_reg, idex_branch, idex_jal, idex_jalr;
    reg        idex_valid;

    // EX/MEM
    reg [31:0] exmem_alu_result, exmem_rs2_data, exmem_pc_branch;
    reg [4:0]  exmem_rd;
    reg [2:0]  exmem_funct3;
    reg        exmem_mem_read, exmem_mem_write, exmem_reg_write, exmem_mem_to_reg;
    reg        exmem_branch_taken;
    reg        exmem_valid;

    // MEM/WB
    reg [31:0] memwb_alu_result, memwb_mem_data;
    reg [4:0]  memwb_rd;
    reg        memwb_reg_write, memwb_mem_to_reg;
    reg        memwb_valid;

    // ═══════════════════════════════════════════════════════
    // Program Counter
    // ═══════════════════════════════════════════════════════
    reg [31:0] pc;
    wire [31:0] pc_next;
    wire        pc_stall;
    wire        flush;

    assign pc_next = flush ? exmem_pc_branch :
                     pc_stall ? pc :
                     pc + 32'd4;

    assign imem_addr = pc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pc <= 32'h0000_0000;
        else        pc <= pc_next;
    end

    // ═══════════════════════════════════════════════════════
    // Register File (32 × 32-bit)
    // ═══════════════════════════════════════════════════════
    reg [31:0] regfile [0:31];
    integer ri;

    wire [4:0]  rf_rs1  = ifid_instr[19:15];
    wire [4:0]  rf_rs2  = ifid_instr[24:20];
    wire [4:0]  rf_rd   = ifid_instr[11:7];
    wire [31:0] rf_rd1  = (rf_rs1 == 5'd0) ? 32'd0 : regfile[rf_rs1];
    wire [31:0] rf_rd2  = (rf_rs2 == 5'd0) ? 32'd0 : regfile[rf_rs2];

    // Writeback
    wire [31:0] wb_data = memwb_mem_to_reg ? memwb_mem_data : memwb_alu_result;
    always @(posedge clk) begin
        if (memwb_valid && memwb_reg_write && memwb_rd != 5'd0)
            regfile[memwb_rd] <= wb_data;
    end

    // ═══════════════════════════════════════════════════════
    // Instruction Decode
    // ═══════════════════════════════════════════════════════
    wire [6:0] opcode  = ifid_instr[6:0];
    wire [2:0] funct3  = ifid_instr[14:12];
    wire [6:0] funct7  = ifid_instr[31:25];

    // Immediate generation
    wire [31:0] imm_i = {{20{ifid_instr[31]}}, ifid_instr[31:20]};
    wire [31:0] imm_s = {{20{ifid_instr[31]}}, ifid_instr[31:25], ifid_instr[11:7]};
    wire [31:0] imm_b = {{19{ifid_instr[31]}}, ifid_instr[31], ifid_instr[7], ifid_instr[30:25], ifid_instr[11:8], 1'b0};
    wire [31:0] imm_u = {ifid_instr[31:12], 12'b0};
    wire [31:0] imm_j = {{11{ifid_instr[31]}}, ifid_instr[31], ifid_instr[19:12], ifid_instr[20], ifid_instr[30:21], 1'b0};

    reg [31:0] imm_dec;
    reg [3:0]  alu_op_dec;
    reg        alu_src_dec, mem_read_dec, mem_write_dec;
    reg        reg_write_dec, mem_to_reg_dec, branch_dec, jal_dec, jalr_dec;

    always @(*) begin
        imm_dec = 32'd0; alu_op_dec = 4'd0; alu_src_dec = 1'b0;
        mem_read_dec = 0; mem_write_dec = 0; reg_write_dec = 0;
        mem_to_reg_dec = 0; branch_dec = 0; jal_dec = 0; jalr_dec = 0;

        case (opcode)
            7'b0110011: begin // R-type
                reg_write_dec = 1; alu_op_dec = {funct7[5], funct3};
            end
            7'b0010011: begin // I-type ALU
                reg_write_dec = 1; alu_src_dec = 1; imm_dec = imm_i;
                alu_op_dec = (funct3 == 3'b101) ? {funct7[5], funct3} : {1'b0, funct3};
            end
            7'b0000011: begin // Load
                reg_write_dec = 1; alu_src_dec = 1; mem_read_dec = 1;
                mem_to_reg_dec = 1; imm_dec = imm_i;
            end
            7'b0100011: begin // Store
                alu_src_dec = 1; mem_write_dec = 1; imm_dec = imm_s;
            end
            7'b1100011: begin // Branch
                branch_dec = 1; imm_dec = imm_b;
            end
            7'b1101111: begin // JAL
                jal_dec = 1; reg_write_dec = 1; imm_dec = imm_j;
            end
            7'b1100111: begin // JALR
                jalr_dec = 1; reg_write_dec = 1; alu_src_dec = 1; imm_dec = imm_i;
            end
            7'b0110111: begin // LUI
                reg_write_dec = 1; imm_dec = imm_u; alu_op_dec = 4'b1111;
            end
            7'b0010111: begin // AUIPC
                reg_write_dec = 1; alu_src_dec = 1; imm_dec = imm_u;
            end
            default: ;
        endcase
    end

    // ═══════════════════════════════════════════════════════
    // Forwarding Unit
    // ═══════════════════════════════════════════════════════
    wire [31:0] fwd_rs1 = (exmem_valid && exmem_reg_write && exmem_rd != 0 && exmem_rd == idex_rs1) ? exmem_alu_result :
                          (memwb_valid && memwb_reg_write && memwb_rd != 0 && memwb_rd == idex_rs1) ? wb_data :
                          idex_rs1_data;

    wire [31:0] fwd_rs2 = (exmem_valid && exmem_reg_write && exmem_rd != 0 && exmem_rd == idex_rs2) ? exmem_alu_result :
                          (memwb_valid && memwb_reg_write && memwb_rd != 0 && memwb_rd == idex_rs2) ? wb_data :
                          idex_rs2_data;

    // ═══════════════════════════════════════════════════════
    // ALU (Execute stage)
    // ═══════════════════════════════════════════════════════
    wire [31:0] alu_a = fwd_rs1;
    wire [31:0] alu_b = idex_alu_src ? idex_imm : fwd_rs2;
    reg  [31:0] alu_result;

    always @(*) begin
        case (idex_alu_op)
            4'b0000: alu_result = alu_a + alu_b;                                    // ADD
            4'b1000: alu_result = alu_a - alu_b;                                    // SUB
            4'b0001: alu_result = alu_a << alu_b[4:0];                              // SLL
            4'b0010: alu_result = ($signed(alu_a) < $signed(alu_b)) ? 32'd1 : 32'd0; // SLT
            4'b0011: alu_result = (alu_a < alu_b) ? 32'd1 : 32'd0;                 // SLTU
            4'b0100: alu_result = alu_a ^ alu_b;                                    // XOR
            4'b0101: alu_result = alu_a >> alu_b[4:0];                              // SRL
            4'b1101: alu_result = $signed(alu_a) >>> alu_b[4:0];                    // SRA
            4'b0110: alu_result = alu_a | alu_b;                                    // OR
            4'b0111: alu_result = alu_a & alu_b;                                    // AND
            4'b1111: alu_result = alu_b;                                            // LUI passthrough
            default: alu_result = 32'd0;
        endcase
    end

    // Branch comparison
    wire branch_eq  = (fwd_rs1 == fwd_rs2);
    wire branch_lt  = ($signed(fwd_rs1) < $signed(fwd_rs2));
    wire branch_ltu = (fwd_rs1 < fwd_rs2);
    reg  branch_take;

    always @(*) begin
        case (idex_funct3)
            3'b000: branch_take = branch_eq;       // BEQ
            3'b001: branch_take = ~branch_eq;      // BNE
            3'b100: branch_take = branch_lt;       // BLT
            3'b101: branch_take = ~branch_lt;      // BGE
            3'b110: branch_take = branch_ltu;      // BLTU
            3'b111: branch_take = ~branch_ltu;     // BGEU
            default: branch_take = 1'b0;
        endcase
    end

    wire ex_branch = idex_valid && idex_branch && branch_take;
    wire ex_jal    = idex_valid && (idex_jal || idex_jalr);

    // ═══════════════════════════════════════════════════════
    // Hazard Detection
    // ═══════════════════════════════════════════════════════
    wire load_use_hazard = idex_valid && idex_mem_read &&
                           ((idex_rd == rf_rs1 && rf_rs1 != 0) ||
                            (idex_rd == rf_rs2 && rf_rs2 != 0));
    assign pc_stall = load_use_hazard;
    assign flush    = ex_branch || ex_jal;

    // ═══════════════════════════════════════════════════════
    // Pipeline Register Updates
    // ═══════════════════════════════════════════════════════
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ifid_valid <= 0; idex_valid <= 0; exmem_valid <= 0; memwb_valid <= 0;
            ifid_pc <= 0; ifid_instr <= 32'h00000013; // NOP
        end else begin
            // IF → ID
            if (pc_stall) begin
                // Stall: keep IF/ID, insert bubble into ID/EX
            end else if (flush) begin
                ifid_valid <= 0;
                ifid_instr <= 32'h00000013;
            end else begin
                ifid_pc    <= pc;
                ifid_instr <= imem_data;
                ifid_valid <= 1'b1;
            end

            // ID → EX
            if (pc_stall || flush) begin
                idex_valid <= 0;
            end else begin
                idex_pc        <= ifid_pc;
                idex_rs1_data  <= rf_rd1;
                idex_rs2_data  <= rf_rd2;
                idex_imm       <= imm_dec;
                idex_rd        <= rf_rd;
                idex_rs1       <= rf_rs1;
                idex_rs2       <= rf_rs2;
                idex_alu_op    <= alu_op_dec;
                idex_funct3    <= funct3;
                idex_alu_src   <= alu_src_dec;
                idex_mem_read  <= mem_read_dec;
                idex_mem_write <= mem_write_dec;
                idex_reg_write <= reg_write_dec;
                idex_mem_to_reg <= mem_to_reg_dec;
                idex_branch    <= branch_dec;
                idex_jal       <= jal_dec;
                idex_jalr      <= jalr_dec;
                idex_valid     <= ifid_valid;
            end

            // EX → MEM
            exmem_alu_result   <= (idex_jal || idex_jalr) ? (idex_pc + 32'd4) : alu_result;
            exmem_rs2_data     <= fwd_rs2;
            exmem_pc_branch    <= idex_jalr ? (fwd_rs1 + idex_imm) : (idex_pc + idex_imm);
            exmem_rd           <= idex_rd;
            exmem_funct3       <= idex_funct3;
            exmem_mem_read     <= idex_mem_read;
            exmem_mem_write    <= idex_mem_write;
            exmem_reg_write    <= idex_reg_write;
            exmem_mem_to_reg   <= idex_mem_to_reg;
            exmem_branch_taken <= ex_branch || ex_jal;
            exmem_valid        <= idex_valid;

            // MEM → WB
            memwb_alu_result <= exmem_alu_result;
            memwb_mem_data   <= dmem_rdata;
            memwb_rd         <= exmem_rd;
            memwb_reg_write  <= exmem_reg_write;
            memwb_mem_to_reg <= exmem_mem_to_reg;
            memwb_valid      <= exmem_valid;
        end
    end

    // ═══════════════════════════════════════════════════════
    // Memory Interface
    // ═══════════════════════════════════════════════════════
    assign dmem_addr  = exmem_alu_result;
    assign dmem_wdata = exmem_rs2_data;
    assign dmem_wen   = exmem_valid & exmem_mem_write;
    assign dmem_ren   = exmem_valid & exmem_mem_read;

    // Byte enable generation
    assign dmem_be = (exmem_funct3[1:0] == 2'b00) ? (4'b0001 << exmem_alu_result[1:0]) :
                     (exmem_funct3[1:0] == 2'b01) ? (exmem_alu_result[1] ? 4'b1100 : 4'b0011) :
                     4'b1111;
endmodule
