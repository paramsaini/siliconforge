// test_decoder_encoder.v — 3:8 decoder + 8:3 priority encoder
// Expected: ~50 gates (decoder ~24 AND/NOT, encoder ~26 OR/gates)
// Tests: combinational decode/encode, priority logic, one-hot

module decoder_3to8 (
    input  [2:0] sel,
    input        en,
    output [7:0] out
);
    assign out[0] = en & ~sel[2] & ~sel[1] & ~sel[0];
    assign out[1] = en & ~sel[2] & ~sel[1] &  sel[0];
    assign out[2] = en & ~sel[2] &  sel[1] & ~sel[0];
    assign out[3] = en & ~sel[2] &  sel[1] &  sel[0];
    assign out[4] = en &  sel[2] & ~sel[1] & ~sel[0];
    assign out[5] = en &  sel[2] & ~sel[1] &  sel[0];
    assign out[6] = en &  sel[2] &  sel[1] & ~sel[0];
    assign out[7] = en &  sel[2] &  sel[1] &  sel[0];
endmodule

module priority_encoder_8to3 (
    input  [7:0] in,
    output reg [2:0] out,
    output       valid
);
    assign valid = |in;

    always @(*) begin
        casez (in)
            8'b1???_????: out = 3'd7;
            8'b01??_????: out = 3'd6;
            8'b001?_????: out = 3'd5;
            8'b0001_????: out = 3'd4;
            8'b0000_1???: out = 3'd3;
            8'b0000_01??: out = 3'd2;
            8'b0000_001?: out = 3'd1;
            8'b0000_0001: out = 3'd0;
            default:      out = 3'd0;
        endcase
    end
endmodule

module dec_enc_top (
    input  [2:0] addr,
    input        chip_en,
    output [2:0] encoded,
    output       enc_valid
);
    wire [7:0] decoded;

    decoder_3to8 dec (
        .sel(addr), .en(chip_en), .out(decoded)
    );

    priority_encoder_8to3 enc (
        .in(decoded), .out(encoded), .valid(enc_valid)
    );
endmodule
