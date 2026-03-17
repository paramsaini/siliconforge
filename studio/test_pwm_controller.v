// test_pwm_controller.v — 8-channel PWM generator with dead-time insertion
// Expected: ~900 gates (8 comparators, dead-time logic, prescaler)
// Tests: multi-channel generate, counter comparison, configurable duty

module pwm_controller #(
    parameter N_CHANNELS = 8,
    parameter RESOLUTION = 8    // 2^8 = 256 levels
) (
    input                           clk,
    input                           rst_n,
    input                           enable,
    input  [RESOLUTION-1:0]         period,
    input  [N_CHANNELS*RESOLUTION-1:0] duty_flat,  // packed duty cycles
    input  [3:0]                    dead_time,       // dead-time in clock cycles
    input  [3:0]                    prescale,        // clock prescaler
    output [N_CHANNELS-1:0]         pwm_h,           // high-side outputs
    output [N_CHANNELS-1:0]         pwm_l            // low-side (complementary)
);
    reg [RESOLUTION-1:0] counter;
    reg [3:0]            pre_cnt;
    wire                 tick = (pre_cnt == prescale);

    // Prescaler
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pre_cnt <= 0;
            counter <= 0;
        end else if (enable) begin
            if (tick) begin
                pre_cnt <= 0;
                counter <= (counter >= period) ? 0 : counter + 1;
            end else
                pre_cnt <= pre_cnt + 1;
        end
    end

    // Per-channel PWM with dead-time
    genvar ch;
    generate
        for (ch = 0; ch < N_CHANNELS; ch = ch + 1) begin : pwm_gen
            wire [RESOLUTION-1:0] duty = duty_flat[ch*RESOLUTION +: RESOLUTION];
            wire raw_pwm = (counter < duty);

            // Dead-time: delay rising edge
            reg [3:0] rise_dly;
            reg [3:0] fall_dly;
            reg       pwm_h_reg, pwm_l_reg;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    rise_dly  <= 0;
                    fall_dly  <= 0;
                    pwm_h_reg <= 0;
                    pwm_l_reg <= 0;
                end else begin
                    if (raw_pwm && !pwm_h_reg) begin
                        if (rise_dly >= dead_time) begin
                            pwm_h_reg <= 1'b1;
                            rise_dly  <= 0;
                        end else
                            rise_dly <= rise_dly + 1;
                        pwm_l_reg <= 1'b0;
                        fall_dly  <= 0;
                    end else if (!raw_pwm && pwm_h_reg) begin
                        pwm_h_reg <= 1'b0;
                        rise_dly  <= 0;
                        if (fall_dly >= dead_time) begin
                            pwm_l_reg <= 1'b1;
                            fall_dly  <= 0;
                        end else
                            fall_dly <= fall_dly + 1;
                    end else if (!raw_pwm) begin
                        pwm_l_reg <= 1'b1;
                    end
                end
            end

            assign pwm_h[ch] = pwm_h_reg & enable;
            assign pwm_l[ch] = pwm_l_reg & enable;
        end
    endgenerate
endmodule
