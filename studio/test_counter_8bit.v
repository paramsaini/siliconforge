// test_counter_8bit.v — Basic synchronous 8-bit counter with enable/reset
// Expected: ~80 gates (8 T-flip-flops + increment logic)
// Tests: sequential logic, sync reset, enable gating

module counter_8bit (
    input        clk,
    input        rst_n,
    input        enable,
    output reg [7:0] count,
    output       overflow
);
    assign overflow = (count == 8'hFF) & enable;

    always @(posedge clk) begin
        if (!rst_n)
            count <= 8'h00;
        else if (enable)
            count <= count + 8'h01;
    end
endmodule
