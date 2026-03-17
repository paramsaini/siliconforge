// test_arbiter_rr.v — 8-port round-robin arbiter with priority override
// Expected: ~600 gates (8 request channels, masking, rotation logic)
// Tests: FSM control, priority encoding, fairness logic, one-hot output

module arbiter_round_robin #(parameter N = 8) (
    input              clk,
    input              rst_n,
    input  [N-1:0]     req,
    input  [N-1:0]     priority_req,   // high-priority override
    output reg [N-1:0] grant,
    output reg         valid,
    output reg [2:0]   winner_id
);
    reg [N-1:0] mask;
    wire [N-1:0] masked_req  = req & mask;
    wire [N-1:0] active_req  = |priority_req ? priority_req :
                               (|masked_req ? masked_req : req);

    // Find lowest set bit (first requestor)
    wire [N-1:0] grant_next;
    assign grant_next = active_req & (~active_req + 1'b1);

    // Encode winner
    function [2:0] encode_onehot;
        input [N-1:0] onehot;
        integer j;
        begin
            encode_onehot = 3'd0;
            for (j = 0; j < N; j = j + 1)
                if (onehot[j])
                    encode_onehot = j[2:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant     <= {N{1'b0}};
            mask      <= {N{1'b1}};
            valid     <= 1'b0;
            winner_id <= 3'd0;
        end else begin
            if (|req) begin
                grant     <= grant_next;
                valid     <= 1'b1;
                winner_id <= encode_onehot(grant_next);
                // Update mask: clear all bits at and below winner for round-robin
                mask <= ~((grant_next - 1'b1) | grant_next);
                if (!(|(masked_req & ~grant_next)))
                    mask <= {N{1'b1}};  // Reset mask when wrapped
            end else begin
                grant <= {N{1'b0}};
                valid <= 1'b0;
            end
        end
    end
endmodule
