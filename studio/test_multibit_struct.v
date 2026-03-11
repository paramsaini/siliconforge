// test_multibit_structural.v — Pure structural multi-bit test (no always blocks)
// Expected: 4 PIs (a[0..3]), 4 POs (result[0..3]), 4 XOR gates

module bit_xor (
    input [3:0] a,
    input [3:0] b,
    output [3:0] result
);
    assign result = a ^ b;
endmodule
