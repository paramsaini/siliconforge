// test_always_comb.v
// Tests always @(*) combinational logic, for loops, blocking assigns, ternary

module priority_encoder (
    input [7:0] data_in,
    output [2:0] encoded,
    output valid
);

    reg [2:0] enc;
    reg v;

    always @(*) begin
        enc = 3'b000;
        v = 1'b0;
        if (data_in[7]) begin
            enc = 3'b111;
            v = 1'b1;
        end else if (data_in[6]) begin
            enc = 3'b110;
            v = 1'b1;
        end else if (data_in[5]) begin
            enc = 3'b101;
            v = 1'b1;
        end else if (data_in[4]) begin
            enc = 3'b100;
            v = 1'b1;
        end else if (data_in[3]) begin
            enc = 3'b011;
            v = 1'b1;
        end else if (data_in[2]) begin
            enc = 3'b010;
            v = 1'b1;
        end else if (data_in[1]) begin
            enc = 3'b001;
            v = 1'b1;
        end else if (data_in[0]) begin
            enc = 3'b000;
            v = 1'b1;
        end
    end

    assign encoded = enc;
    assign valid = v;

endmodule
