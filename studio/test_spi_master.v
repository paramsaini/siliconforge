// test_spi_master.v — Full SPI master with configurable clock divider
// Expected: ~800 gates (shift reg, clock div, FSM, CPOL/CPHA)
// Tests: protocol-level FSM, clock generation, configurable polarity/phase

module spi_master #(
    parameter DATA_WIDTH = 8,
    parameter CLK_DIV    = 4    // SCLK = clk / (2 * CLK_DIV)
) (
    input                       clk,
    input                       rst_n,
    // Control
    input                       cpol,       // Clock polarity
    input                       cpha,       // Clock phase
    input                       start,
    input  [DATA_WIDTH-1:0]     mosi_data,
    output reg [DATA_WIDTH-1:0] miso_data,
    output reg                  busy,
    output reg                  done,
    // SPI bus
    output reg                  sclk,
    output reg                  mosi,
    input                       miso,
    output reg                  cs_n
);
    localparam CNT_W = $clog2(CLK_DIV);

    localparam [2:0] ST_IDLE    = 3'd0,
                     ST_CS_LOW  = 3'd1,
                     ST_LEADING = 3'd2,
                     ST_TRAIL   = 3'd3,
                     ST_CS_HIGH = 3'd4,
                     ST_DONE    = 3'd5;

    reg [2:0]            state;
    reg [CNT_W-1:0]      clk_cnt;
    reg [3:0]            bit_cnt;
    reg [DATA_WIDTH-1:0] tx_shift;
    reg [DATA_WIDTH-1:0] rx_shift;
    reg                  sclk_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            sclk     <= 1'b0;
            mosi     <= 1'b0;
            cs_n     <= 1'b1;
            busy     <= 1'b0;
            done     <= 1'b0;
            clk_cnt  <= 0;
            bit_cnt  <= 0;
            tx_shift <= 0;
            rx_shift <= 0;
            sclk_reg <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    sclk <= cpol;
                    sclk_reg <= cpol;
                    cs_n <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        tx_shift <= mosi_data;
                        rx_shift <= 0;
                        bit_cnt  <= 0;
                        clk_cnt  <= 0;
                        busy     <= 1'b1;
                        state    <= ST_CS_LOW;
                    end
                end

                ST_CS_LOW: begin
                    cs_n <= 1'b0;
                    if (!cpha)
                        mosi <= tx_shift[DATA_WIDTH-1];
                    state <= ST_LEADING;
                    clk_cnt <= 0;
                end

                ST_LEADING: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt  <= 0;
                        sclk_reg <= ~sclk_reg;
                        sclk     <= ~sclk_reg;
                        if (cpha)
                            mosi <= tx_shift[DATA_WIDTH-1];
                        else
                            rx_shift <= {rx_shift[DATA_WIDTH-2:0], miso};
                        state <= ST_TRAIL;
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                ST_TRAIL: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt  <= 0;
                        sclk_reg <= ~sclk_reg;
                        sclk     <= ~sclk_reg;
                        if (cpha)
                            rx_shift <= {rx_shift[DATA_WIDTH-2:0], miso};
                        else begin
                            tx_shift <= {tx_shift[DATA_WIDTH-2:0], 1'b0};
                        end
                        if (cpha)
                            tx_shift <= {tx_shift[DATA_WIDTH-2:0], 1'b0};

                        if (bit_cnt == DATA_WIDTH - 1) begin
                            state <= ST_CS_HIGH;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                            if (!cpha)
                                mosi <= tx_shift[DATA_WIDTH-2];
                            state <= ST_LEADING;
                        end
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                ST_CS_HIGH: begin
                    cs_n      <= 1'b1;
                    miso_data <= rx_shift;
                    done      <= 1'b1;
                    state     <= ST_DONE;
                end

                ST_DONE: begin
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
