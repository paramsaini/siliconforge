// test_behavioral.v
// Simple behavioral ALU testing the new Phase 15 Behavioral Synthesizer

module behavioral_alu (
    input clk,
    input a,
    input b,
    output q
);
    wire sum;
    wire xor_ab;
    
    // Arithmetic Operator logic (will lower to Half-Adder)
    assign sum = a + b;
    assign xor_ab = a ^ b;
    
    // Procedural behavior (will lower to D-Flip Flop Mux logic)
    always @(posedge clk) begin
        if (sum == 1) begin
            q <= xor_ab;
        end else begin
            q <= sum;
        end
    end

endmodule
