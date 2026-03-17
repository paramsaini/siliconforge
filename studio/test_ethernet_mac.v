// test_ethernet_mac.v — 10/100 Ethernet MAC with CRC, FIFO, MII interface
// Expected: ~8,000 gates (TX FSM, RX FSM, CRC-32, FIFOs, MII)
// Tests: protocol-level design, dual-clock domain prep, CRC generation

module ethernet_mac (
    input         clk,
    input         rst_n,
    // TX interface
    input  [7:0]  tx_data,
    input         tx_valid,
    input         tx_last,
    output        tx_ready,
    // RX interface
    output reg [7:0] rx_data,
    output reg       rx_valid,
    output reg       rx_last,
    output reg       rx_crc_ok,
    // MII interface
    output reg       mii_tx_en,
    output reg [3:0] mii_txd,
    input            mii_rx_dv,
    input  [3:0]     mii_rxd,
    input            mii_tx_clk,
    input            mii_rx_clk
);
    // ── TX Path ──
    localparam [2:0] TX_IDLE     = 3'd0,
                     TX_PREAMBLE = 3'd1,
                     TX_SFD      = 3'd2,
                     TX_DATA     = 3'd3,
                     TX_FCS      = 3'd4,
                     TX_IFG      = 3'd5;

    reg [2:0]  tx_state;
    reg [3:0]  tx_cnt;
    reg [5:0]  tx_byte_cnt;
    reg [31:0] tx_crc;
    reg [7:0]  tx_fifo [0:63];
    reg [5:0]  tx_fifo_wr, tx_fifo_rd;
    reg [6:0]  tx_fifo_len;
    reg        tx_nibble_sel;
    reg [7:0]  tx_cur_byte;

    assign tx_ready = (tx_state == TX_IDLE) && (tx_fifo_len < 60);

    // CRC-32 for one byte
    function [31:0] crc32_step;
        input [31:0] crc;
        input [7:0] data;
        reg [31:0] c;
        integer b;
        begin
            c = crc ^ {24'hFFFFFF, data};
            for (b = 0; b < 8; b = b + 1)
                c = c[0] ? (c >> 1) ^ 32'hEDB88320 : (c >> 1);
            crc32_step = c;
        end
    endfunction

    // TX FIFO write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_fifo_wr <= 0; tx_fifo_len <= 0;
        end else if (tx_valid && tx_ready) begin
            tx_fifo[tx_fifo_wr] <= tx_data;
            tx_fifo_wr <= tx_fifo_wr + 1;
            if (tx_last) tx_fifo_len <= tx_fifo_wr + 1;
        end
    end

    // TX state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE; mii_tx_en <= 0; mii_txd <= 0;
            tx_cnt <= 0; tx_byte_cnt <= 0; tx_crc <= 32'hFFFFFFFF;
            tx_fifo_rd <= 0; tx_nibble_sel <= 0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    mii_tx_en <= 0;
                    if (tx_fifo_len > 0) begin
                        tx_state <= TX_PREAMBLE;
                        tx_cnt <= 0;
                        tx_crc <= 32'hFFFFFFFF;
                        tx_fifo_rd <= 0;
                    end
                end

                TX_PREAMBLE: begin
                    mii_tx_en <= 1;
                    mii_txd <= 4'h5;  // 0101 pattern
                    tx_cnt <= tx_cnt + 1;
                    if (tx_cnt == 14) // 7 bytes × 2 nibbles
                        tx_state <= TX_SFD;
                end

                TX_SFD: begin
                    if (!tx_nibble_sel) begin
                        mii_txd <= 4'h5;
                        tx_nibble_sel <= 1;
                    end else begin
                        mii_txd <= 4'hD;  // SFD = 0xD5
                        tx_nibble_sel <= 0;
                        tx_state <= TX_DATA;
                        tx_byte_cnt <= 0;
                    end
                end

                TX_DATA: begin
                    if (!tx_nibble_sel) begin
                        tx_cur_byte <= tx_fifo[tx_fifo_rd];
                        mii_txd <= tx_fifo[tx_fifo_rd][3:0];
                        tx_nibble_sel <= 1;
                    end else begin
                        mii_txd <= tx_cur_byte[7:4];
                        tx_nibble_sel <= 0;
                        tx_crc <= crc32_step(tx_crc, tx_cur_byte);
                        tx_fifo_rd <= tx_fifo_rd + 1;
                        tx_byte_cnt <= tx_byte_cnt + 1;
                        if (tx_byte_cnt + 1 >= tx_fifo_len)
                            tx_state <= TX_FCS;
                    end
                end

                TX_FCS: begin
                    if (!tx_nibble_sel) begin
                        mii_txd <= ~tx_crc[3:0];
                        tx_nibble_sel <= 1;
                    end else begin
                        mii_txd <= ~tx_crc[7:4];
                        tx_nibble_sel <= 0;
                        tx_crc <= {8'h00, tx_crc[31:8]};
                        tx_cnt <= tx_cnt + 1;
                        if (tx_cnt >= 3) begin
                            tx_state <= TX_IFG;
                            tx_cnt <= 0;
                        end
                    end
                end

                TX_IFG: begin
                    mii_tx_en <= 0;
                    tx_cnt <= tx_cnt + 1;
                    if (tx_cnt >= 11) begin // 12 byte IFG
                        tx_state <= TX_IDLE;
                        tx_fifo_len <= 0;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // ── RX Path ──
    localparam [1:0] RX_IDLE = 2'd0, RX_PREAMBLE = 2'd1, RX_DATA = 2'd2, RX_CHECK = 2'd3;
    reg [1:0] rx_state;
    reg [7:0] rx_byte;
    reg       rx_nibble_sel_r;
    reg [31:0] rx_crc_reg;
    reg [5:0] rx_len;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE; rx_valid <= 0; rx_last <= 0;
            rx_nibble_sel_r <= 0; rx_crc_reg <= 32'hFFFFFFFF; rx_len <= 0;
        end else begin
            rx_valid <= 0; rx_last <= 0;

            case (rx_state)
                RX_IDLE: begin
                    if (mii_rx_dv && mii_rxd == 4'hD) begin
                        rx_state <= RX_DATA;
                        rx_nibble_sel_r <= 0;
                        rx_crc_reg <= 32'hFFFFFFFF;
                        rx_len <= 0;
                    end else if (mii_rx_dv && mii_rxd == 4'h5) begin
                        rx_state <= RX_PREAMBLE;
                    end
                end

                RX_PREAMBLE: begin
                    if (!mii_rx_dv) rx_state <= RX_IDLE;
                    else if (mii_rxd == 4'hD) begin
                        rx_state <= RX_DATA;
                        rx_nibble_sel_r <= 0;
                        rx_crc_reg <= 32'hFFFFFFFF;
                    end
                end

                RX_DATA: begin
                    if (!mii_rx_dv) begin
                        rx_last   <= 1;
                        rx_valid  <= 1;
                        rx_crc_ok <= (rx_crc_reg == 32'hDEBB20E3);
                        rx_state  <= RX_IDLE;
                    end else if (!rx_nibble_sel_r) begin
                        rx_byte[3:0] <= mii_rxd;
                        rx_nibble_sel_r <= 1;
                    end else begin
                        rx_byte[7:4] <= mii_rxd;
                        rx_nibble_sel_r <= 0;
                        rx_data   <= {mii_rxd, rx_byte[3:0]};
                        rx_valid  <= 1;
                        rx_crc_reg <= crc32_step(rx_crc_reg, {mii_rxd, rx_byte[3:0]});
                        rx_len <= rx_len + 1;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end
endmodule
