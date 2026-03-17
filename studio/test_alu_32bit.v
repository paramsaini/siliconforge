// test_alu_32bit.v — Full 32-bit ALU with flags (ARM-style)
// Expected: ~2,500 gates (32-bit adder, shifter, logic, flag gen)
// Tests: wide datapath, multiple operations, carry chain, flag generation

module alu_32bit (
    input  [31:0] a,
    input  [31:0] b,
    input  [3:0]  op,
    input         carry_in,
    output reg [31:0] result,
    output        zero,
    output        negative,
    output        carry_out,
    output        overflow
);
    wire [32:0] add_result = {1'b0, a} + {1'b0, b} + {32'b0, carry_in};
    wire [32:0] sub_result = {1'b0, a} - {1'b0, b} - {32'b0, ~carry_in};
    wire [31:0] shift_left  = a << b[4:0];
    wire [31:0] shift_right = a >> b[4:0];
    wire [31:0] shift_arith = $signed(a) >>> b[4:0];

    reg c_out;

    always @(*) begin
        c_out = 1'b0;
        case (op)
            4'b0000: begin result = add_result[31:0]; c_out = add_result[32]; end   // ADD
            4'b0001: begin result = sub_result[31:0]; c_out = sub_result[32]; end   // SUB
            4'b0010: begin result = a & b;  end                                      // AND
            4'b0011: begin result = a | b;  end                                      // OR
            4'b0100: begin result = a ^ b;  end                                      // XOR
            4'b0101: begin result = ~a;     end                                      // NOT
            4'b0110: begin result = a & ~b; end                                      // BIC
            4'b0111: begin result = a | ~b; end                                      // ORN
            4'b1000: begin result = shift_left;  end                                 // LSL
            4'b1001: begin result = shift_right; end                                 // LSR
            4'b1010: begin result = shift_arith; end                                 // ASR
            4'b1011: begin result = {a[0], a[31:1]}; end                             // ROR by 1
            4'b1100: begin result = (a < b) ? 32'd1 : 32'd0; end                    // SLT unsigned
            4'b1101: begin result = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0; end  // SLT signed
            4'b1110: begin result = b; end                                           // MOV
            4'b1111: begin result = ~b; end                                          // MVN
            default: begin result = 32'h0; end
        endcase
    end

    assign zero     = (result == 32'h0);
    assign negative = result[31];
    assign carry_out = c_out;
    assign overflow  = (op == 4'b0000) ?
                       ((a[31] == b[31]) && (result[31] != a[31])) :
                       (op == 4'b0001) ?
                       ((a[31] != b[31]) && (result[31] != a[31])) :
                       1'b0;
endmodule
