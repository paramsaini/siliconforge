// test_cache_controller.v — Direct-mapped write-back cache controller
// Expected: ~6,000 gates (FSM, tag compare, dirty tracking, LRU, data mux)
// Tests: memory hierarchy FSM, tag/data arrays, write-back protocol

module cache_controller #(
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 32,
    parameter CACHE_LINES = 64,     // Number of cache lines
    parameter LINE_SIZE   = 4       // Words per line
) (
    input                       clk,
    input                       rst_n,
    // CPU interface
    input  [ADDR_WIDTH-1:0]     cpu_addr,
    input  [DATA_WIDTH-1:0]     cpu_wdata,
    input                       cpu_rd,
    input                       cpu_wr,
    output reg [DATA_WIDTH-1:0] cpu_rdata,
    output reg                  cpu_ready,
    // Main memory interface
    output reg [ADDR_WIDTH-1:0] mem_addr,
    output reg [DATA_WIDTH-1:0] mem_wdata,
    output reg                  mem_rd,
    output reg                  mem_wr,
    input  [DATA_WIDTH-1:0]     mem_rdata,
    input                       mem_ready,
    // Status
    output reg                  hit,
    output reg                  miss
);
    localparam INDEX_BITS  = $clog2(CACHE_LINES);
    localparam OFFSET_BITS = $clog2(LINE_SIZE) + 2; // +2 for byte offset
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;

    // Extract address fields
    wire [TAG_BITS-1:0]    addr_tag    = cpu_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0]  addr_index  = cpu_addr[OFFSET_BITS +: INDEX_BITS];
    wire [$clog2(LINE_SIZE)-1:0] addr_word = cpu_addr[2 +: $clog2(LINE_SIZE)];

    // Cache storage
    reg [TAG_BITS-1:0]    tag_array   [0:CACHE_LINES-1];
    reg                   valid_array [0:CACHE_LINES-1];
    reg                   dirty_array [0:CACHE_LINES-1];
    reg [DATA_WIDTH-1:0]  data_array  [0:CACHE_LINES-1][0:LINE_SIZE-1];

    // State machine
    localparam [2:0] ST_IDLE      = 3'd0,
                     ST_COMPARE   = 3'd1,
                     ST_WRITEBACK = 3'd2,
                     ST_WB_WAIT   = 3'd3,
                     ST_FILL      = 3'd4,
                     ST_FILL_WAIT = 3'd5,
                     ST_UPDATE    = 3'd6;

    reg [2:0] state;
    reg [$clog2(LINE_SIZE)-1:0] word_cnt;
    reg [ADDR_WIDTH-1:0] saved_addr;
    reg [DATA_WIDTH-1:0] saved_wdata;
    reg                  saved_wr;

    // Tag match
    wire tag_match = valid_array[addr_index] && (tag_array[addr_index] == addr_tag);
    wire is_dirty  = dirty_array[addr_index];

    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            cpu_ready <= 1'b0;
            mem_rd    <= 1'b0;
            mem_wr    <= 1'b0;
            hit       <= 1'b0;
            miss      <= 1'b0;
            word_cnt  <= 0;
            for (j = 0; j < CACHE_LINES; j = j + 1) begin
                valid_array[j] <= 1'b0;
                dirty_array[j] <= 1'b0;
            end
        end else begin
            cpu_ready <= 1'b0;
            hit       <= 1'b0;
            miss      <= 1'b0;
            mem_rd    <= 1'b0;
            mem_wr    <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (cpu_rd || cpu_wr) begin
                        saved_addr  <= cpu_addr;
                        saved_wdata <= cpu_wdata;
                        saved_wr    <= cpu_wr;
                        state       <= ST_COMPARE;
                    end
                end

                ST_COMPARE: begin
                    if (tag_match) begin
                        // Cache hit
                        hit <= 1'b1;
                        if (saved_wr) begin
                            data_array[addr_index][addr_word] <= saved_wdata;
                            dirty_array[addr_index] <= 1'b1;
                        end else begin
                            cpu_rdata <= data_array[addr_index][addr_word];
                        end
                        cpu_ready <= 1'b1;
                        state     <= ST_IDLE;
                    end else begin
                        // Cache miss
                        miss <= 1'b1;
                        if (valid_array[addr_index] && is_dirty)
                            state <= ST_WRITEBACK;
                        else
                            state <= ST_FILL;
                        word_cnt <= 0;
                    end
                end

                ST_WRITEBACK: begin
                    mem_addr  <= {tag_array[addr_index], addr_index, {OFFSET_BITS{1'b0}}} + (word_cnt << 2);
                    mem_wdata <= data_array[addr_index][word_cnt];
                    mem_wr    <= 1'b1;
                    state     <= ST_WB_WAIT;
                end

                ST_WB_WAIT: begin
                    if (mem_ready) begin
                        if (word_cnt == LINE_SIZE - 1) begin
                            dirty_array[addr_index] <= 1'b0;
                            word_cnt <= 0;
                            state    <= ST_FILL;
                        end else begin
                            word_cnt <= word_cnt + 1;
                            state    <= ST_WRITEBACK;
                        end
                    end
                end

                ST_FILL: begin
                    mem_addr <= {addr_tag, addr_index, {OFFSET_BITS{1'b0}}} + (word_cnt << 2);
                    mem_rd   <= 1'b1;
                    state    <= ST_FILL_WAIT;
                end

                ST_FILL_WAIT: begin
                    if (mem_ready) begin
                        data_array[addr_index][word_cnt] <= mem_rdata;
                        if (word_cnt == LINE_SIZE - 1) begin
                            tag_array[addr_index]   <= addr_tag;
                            valid_array[addr_index] <= 1'b1;
                            state <= ST_UPDATE;
                        end else begin
                            word_cnt <= word_cnt + 1;
                            state    <= ST_FILL;
                        end
                    end
                end

                ST_UPDATE: begin
                    if (saved_wr) begin
                        data_array[addr_index][addr_word] <= saved_wdata;
                        dirty_array[addr_index] <= 1'b1;
                    end else begin
                        cpu_rdata <= data_array[addr_index][addr_word];
                    end
                    cpu_ready <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
