`define WIDTH 8
`define HALF_WIDTH 4
`ifdef WIDTH
`define ACTIVE 1
`endif

module all_behavioral_test(
    input clk,
    input rst,
    input [`WIDTH-1:0] a,
    input [`WIDTH-1:0] b,
    input [2:0] sel,
    input en,
    output reg [`WIDTH-1:0] result,
    output reg [`WIDTH-1:0] shifted,
    output reg [`WIDTH-1:0] arith_out,
    output reg done
);

// Parameter for generate
parameter NUM_STAGES = 4;

// Function definition
function [`WIDTH-1:0] add_one;
    input [`WIDTH-1:0] val;
    begin
        add_one = val + 1;
    end
endfunction

// Generate block: create pipeline registers
genvar i;
generate
    for (i = 0; i < NUM_STAGES; i = i + 1) begin : stage
        reg [`WIDTH-1:0] pipe_reg;
    end
endgenerate

// Always block with full operator coverage
always @(posedge clk) begin
    if (rst) begin
        result <= 8'h00;
        shifted <= 8'h00;
        arith_out <= 8'h00;
        done <= 1'b0;
    end else begin
        case (sel)
            3'b000: result <= a + b;          // addition
            3'b001: result <= a - b;          // subtraction
            3'b010: result <= a * b;          // multiplication
            3'b011: result <= a & b;          // bitwise AND
            3'b100: result <= a | b;          // bitwise OR
            3'b101: result <= a ^ b;          // bitwise XOR
            3'b110: result <= ~a;             // bitwise NOT
            default: result <= a;
        endcase

        // Shift operations
        shifted <= a << sel;

        // Part-select assignment
        arith_out[7:4] <= a[3:0];
        arith_out[3:0] <= b[7:4];

        // Function call
        done <= (add_one(a) == b) ? 1'b1 : 1'b0;
    end
end

// Combinational always block
always @(*) begin
    if (en && (a > b))
        arith_out = a;
    else
        arith_out = b;
end

endmodule
