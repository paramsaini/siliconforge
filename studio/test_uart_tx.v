// test_uart_tx.v — UART transmitter with configurable baud rate
// Expected: ~400 gates (baud generator, shift register, FSM)
// Tests: parameterized counter, state machine, serial protocol

module uart_tx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200,
    parameter DATA_BITS = 8,
    parameter STOP_BITS = 1,
    parameter PARITY_EN = 1,
    parameter PARITY_ODD = 0
) (
    input                   clk,
    input                   rst_n,
    input  [DATA_BITS-1:0]  tx_data,
    input                   tx_valid,
    output reg              tx_out,
    output reg              tx_busy,
    output reg              tx_done
);
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam CNT_W = $clog2(CLKS_PER_BIT);

    localparam [2:0] IDLE    = 3'd0,
                     START   = 3'd1,
                     DATA    = 3'd2,
                     PARITY  = 3'd3,
                     STOP    = 3'd4,
                     CLEANUP = 3'd5;

    reg [2:0]            state;
    reg [CNT_W-1:0]      clk_cnt;
    reg [2:0]            bit_idx;
    reg [DATA_BITS-1:0]  shift_reg;
    reg                  parity_bit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            tx_out    <= 1'b1;
            tx_busy   <= 1'b0;
            tx_done   <= 1'b0;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= 0;
        end else begin
            tx_done <= 1'b0;

            case (state)
                IDLE: begin
                    tx_out  <= 1'b1;
                    tx_busy <= 1'b0;
                    if (tx_valid) begin
                        shift_reg  <= tx_data;
                        parity_bit <= PARITY_ODD ? ~(^tx_data) : ^tx_data;
                        tx_busy    <= 1'b1;
                        state      <= START;
                        clk_cnt    <= 0;
                    end
                end

                START: begin
                    tx_out <= 1'b0;  // Start bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= DATA;
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                DATA: begin
                    tx_out <= shift_reg[0];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt   <= 0;
                        shift_reg <= {1'b0, shift_reg[DATA_BITS-1:1]};
                        if (bit_idx == DATA_BITS - 1) begin
                            state <= PARITY_EN ? PARITY : STOP;
                        end else
                            bit_idx <= bit_idx + 1;
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                PARITY: begin
                    tx_out <= parity_bit;
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        state   <= STOP;
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                STOP: begin
                    tx_out <= 1'b1;  // Stop bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        tx_done <= 1'b1;
                        state   <= CLEANUP;
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                CLEANUP: begin
                    tx_busy <= 1'b0;
                    state   <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
