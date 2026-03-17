// test_i2c_master.v — I2C master controller (100kHz/400kHz)
// Expected: ~700 gates (FSM, bit counter, shift register, ACK logic)
// Tests: bidirectional I/O, open-drain protocol, complex state machine

module i2c_master #(
    parameter CLK_DIV = 125  // 50MHz / (4 * 100kHz) = 125
) (
    input            clk,
    input            rst_n,
    // Command interface
    input      [6:0] slave_addr,
    input            rw,           // 0=write, 1=read
    input      [7:0] wr_data,
    input            start_cmd,
    output reg [7:0] rd_data,
    output reg       busy,
    output reg       ack_error,
    output reg       done,
    // I2C bus (directly active-low with OD pull — output 0 or high-Z)
    output reg       scl_oe,
    output reg       sda_oe,
    input            sda_in
);
    localparam [3:0] ST_IDLE     = 4'd0,
                     ST_START    = 4'd1,
                     ST_ADDR     = 4'd2,
                     ST_ADDR_ACK = 4'd3,
                     ST_WR_DATA  = 4'd4,
                     ST_WR_ACK   = 4'd5,
                     ST_RD_DATA  = 4'd6,
                     ST_RD_ACK   = 4'd7,
                     ST_STOP     = 4'd8,
                     ST_DONE     = 4'd9;

    reg [3:0]  state;
    reg [7:0]  clk_cnt;
    reg [1:0]  phase;    // 4 phases per SCL cycle
    reg [3:0]  bit_cnt;
    reg [7:0]  shift;
    reg        rw_reg;

    wire phase_tick = (clk_cnt == CLK_DIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            scl_oe    <= 1'b0;  // high-Z = SCL high
            sda_oe    <= 1'b0;  // high-Z = SDA high
            busy      <= 1'b0;
            ack_error <= 1'b0;
            done      <= 1'b0;
            clk_cnt   <= 0;
            phase     <= 0;
            bit_cnt   <= 0;
            shift     <= 0;
        end else begin
            done <= 1'b0;

            if (state != ST_IDLE && state != ST_DONE) begin
                if (phase_tick) begin
                    clk_cnt <= 0;
                    phase   <= phase + 1;
                end else
                    clk_cnt <= clk_cnt + 1;
            end

            case (state)
                ST_IDLE: begin
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                    busy   <= 1'b0;
                    if (start_cmd) begin
                        busy     <= 1'b1;
                        ack_error <= 1'b0;
                        rw_reg   <= rw;
                        shift    <= {slave_addr, rw};
                        bit_cnt  <= 0;
                        clk_cnt  <= 0;
                        phase    <= 0;
                        state    <= ST_START;
                    end
                end

                ST_START: begin
                    // SDA goes low while SCL is high
                    case (phase)
                        2'd0: begin sda_oe <= 1'b0; scl_oe <= 1'b0; end
                        2'd1: begin sda_oe <= 1'b1; end  // SDA low
                        2'd2: begin scl_oe <= 1'b1; end  // SCL low
                        2'd3: begin state <= ST_ADDR; bit_cnt <= 0; phase <= 0; clk_cnt <= 0; end
                    endcase
                end

                ST_ADDR: begin
                    if (phase_tick && phase == 2'd3) begin
                        if (bit_cnt == 7) begin
                            state   <= ST_ADDR_ACK;
                            bit_cnt <= 0;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                            shift   <= {shift[6:0], 1'b0};
                        end
                    end
                    // Drive SDA on phase 0, SCL high on phase 1, sample on 2, SCL low on 3
                    case (phase)
                        2'd0: sda_oe <= ~shift[7];  // drive bit (0=OE means high)
                        2'd1: scl_oe <= 1'b0;       // SCL high
                        2'd3: scl_oe <= 1'b1;       // SCL low
                        default: ;
                    endcase
                end

                ST_ADDR_ACK: begin
                    case (phase)
                        2'd0: sda_oe <= 1'b0;  // Release SDA for ACK
                        2'd1: scl_oe <= 1'b0;  // SCL high
                        2'd2: begin
                            if (sda_in) ack_error <= 1'b1;  // NACK
                        end
                        2'd3: begin
                            scl_oe <= 1'b1;
                            if (ack_error) state <= ST_STOP;
                            else if (rw_reg) begin
                                state <= ST_RD_DATA; bit_cnt <= 0;
                            end else begin
                                shift <= wr_data;
                                state <= ST_WR_DATA; bit_cnt <= 0;
                            end
                        end
                    endcase
                end

                ST_WR_DATA: begin
                    case (phase)
                        2'd0: sda_oe <= ~shift[7];
                        2'd1: scl_oe <= 1'b0;
                        2'd3: begin
                            scl_oe <= 1'b1;
                            shift  <= {shift[6:0], 1'b0};
                            if (bit_cnt == 7) state <= ST_WR_ACK;
                            else bit_cnt <= bit_cnt + 1;
                        end
                        default: ;
                    endcase
                end

                ST_WR_ACK: begin
                    case (phase)
                        2'd0: sda_oe <= 1'b0;
                        2'd1: scl_oe <= 1'b0;
                        2'd2: if (sda_in) ack_error <= 1'b1;
                        2'd3: begin scl_oe <= 1'b1; state <= ST_STOP; end
                    endcase
                end

                ST_RD_DATA: begin
                    case (phase)
                        2'd0: sda_oe <= 1'b0;  // Release SDA
                        2'd1: scl_oe <= 1'b0;
                        2'd2: shift <= {shift[6:0], sda_in};
                        2'd3: begin
                            scl_oe <= 1'b1;
                            if (bit_cnt == 7) begin
                                rd_data <= {shift[6:0], sda_in};
                                state   <= ST_RD_ACK;
                            end else
                                bit_cnt <= bit_cnt + 1;
                        end
                    endcase
                end

                ST_RD_ACK: begin
                    case (phase)
                        2'd0: sda_oe <= 1'b1;  // Send NACK (last byte)
                        2'd1: scl_oe <= 1'b0;
                        2'd3: begin scl_oe <= 1'b1; state <= ST_STOP; end
                        default: ;
                    endcase
                end

                ST_STOP: begin
                    case (phase)
                        2'd0: sda_oe <= 1'b1;  // SDA low
                        2'd1: scl_oe <= 1'b0;  // SCL high
                        2'd2: sda_oe <= 1'b0;  // SDA high (stop condition)
                        2'd3: begin state <= ST_DONE; end
                    endcase
                end

                ST_DONE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
