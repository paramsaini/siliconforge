// test_gray_code.v — Gray-to-binary and binary-to-Gray converters
// Expected: ~90 gates (XOR chains + register stages)
// Tests: XOR reduction, pipelined conversion, clock domain prep

module bin2gray #(parameter N = 8) (
    input  [N-1:0] bin,
    output [N-1:0] gray
);
    assign gray = bin ^ (bin >> 1);
endmodule

module gray2bin #(parameter N = 8) (
    input  [N-1:0] gray,
    output [N-1:0] bin
);
    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : g2b
            assign bin[i] = ^gray[N-1:i];
        end
    endgenerate
endmodule

module gray_counter #(parameter N = 8) (
    input              clk,
    input              rst_n,
    input              enable,
    output [N-1:0]     gray_out,
    output [N-1:0]     bin_out,
    output             wrap
);
    reg [N-1:0] count;

    wire [N-1:0] gray_val;
    wire [N-1:0] bin_back;

    bin2gray #(N) b2g (.bin(count), .gray(gray_val));
    gray2bin #(N) g2b (.gray(gray_val), .bin(bin_back));

    assign gray_out = gray_val;
    assign bin_out  = count;
    assign wrap     = enable & (&count);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            count <= {N{1'b0}};
        else if (enable)
            count <= count + 1'b1;
    end
endmodule
