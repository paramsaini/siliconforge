// test_adder.v — 4-bit adder test
// Expected: 8 PIs, 4 POs, 17 gates (1 HA=2 gates + 3 FA=15 gates)

module adder4 (
    input [3:0] a,
    input [3:0] b,
    output [3:0] sum
);
    assign sum = a + b;
endmodule
