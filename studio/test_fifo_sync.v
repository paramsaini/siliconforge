// test_fifo_sync.v — Synchronous FIFO with configurable depth
// Expected: ~1,800 gates (16-deep × 8-bit = 128 DFFs + control)
// Tests: memory array, pointer logic, full/empty flags, generate

module fifo_sync #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4     // depth = 2^ADDR_WIDTH = 16
) (
    input                       clk,
    input                       rst_n,
    input                       wr_en,
    input                       rd_en,
    input  [DATA_WIDTH-1:0]     wr_data,
    output reg [DATA_WIDTH-1:0] rd_data,
    output                      full,
    output                      empty,
    output                      almost_full,
    output                      almost_empty,
    output [ADDR_WIDTH:0]       count
);
    localparam DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH:0]   wr_ptr;
    reg [ADDR_WIDTH:0]   rd_ptr;

    wire wr_valid = wr_en & ~full;
    wire rd_valid = rd_en & ~empty;

    assign count        = wr_ptr - rd_ptr;
    assign full         = (count == DEPTH);
    assign empty        = (count == 0);
    assign almost_full  = (count >= DEPTH - 2);
    assign almost_empty = (count <= 2);

    // Write logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
        end else if (wr_valid) begin
            mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
            wr_ptr <= wr_ptr + 1;
        end
    end

    // Read logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr  <= 0;
            rd_data <= {DATA_WIDTH{1'b0}};
        end else if (rd_valid) begin
            rd_data <= mem[rd_ptr[ADDR_WIDTH-1:0]];
            rd_ptr  <= rd_ptr + 1;
        end
    end
endmodule
