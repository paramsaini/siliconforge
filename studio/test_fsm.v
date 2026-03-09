// Simple FSM Test: Traffic Light Controller
module traffic_fsm(
    input clk,
    output wire red,
    output wire green,
    output wire yellow
);

    reg [1:0] state;

    always @(posedge clk) begin
        if (state == 0) begin // RED
            state <= 1;
        end else if (state == 1) begin // GREEN
            state <= 2;
        end else if (state == 2) begin // YELLOW
            state <= 0;
        end
    end

    assign red = (state == 0);
    assign green = (state == 1);
    assign yellow = (state == 2);

endmodule
