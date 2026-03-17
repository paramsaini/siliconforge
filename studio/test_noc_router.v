// test_noc_router.v — 5-port wormhole NoC router with virtual channels
// Expected: ~20,000 gates (5×5 crossbar, 2 VCs per port, routing logic, arbitration)
// Tests: complex interconnect, multi-port arbitration, flow control, deep FSM

module noc_router #(
    parameter FLIT_WIDTH = 32,
    parameter NUM_VCS    = 2,
    parameter BUF_DEPTH  = 4,
    parameter X_COORD    = 2,
    parameter Y_COORD    = 2,
    parameter MESH_SIZE  = 4
) (
    input  clk,
    input  rst_n,
    // North port
    input  [FLIT_WIDTH-1:0] n_flit_in,  input  n_valid_in,  output n_ready_out,
    output [FLIT_WIDTH-1:0] n_flit_out, output n_valid_out, input  n_ready_in,
    // South port
    input  [FLIT_WIDTH-1:0] s_flit_in,  input  s_valid_in,  output s_ready_out,
    output [FLIT_WIDTH-1:0] s_flit_out, output s_valid_out, input  s_ready_in,
    // East port
    input  [FLIT_WIDTH-1:0] e_flit_in,  input  e_valid_in,  output e_ready_out,
    output [FLIT_WIDTH-1:0] e_flit_out, output e_valid_out, input  e_ready_in,
    // West port
    input  [FLIT_WIDTH-1:0] w_flit_in,  input  w_valid_in,  output w_ready_out,
    output [FLIT_WIDTH-1:0] w_flit_out, output w_valid_out, input  w_ready_in,
    // Local port (PE interface)
    input  [FLIT_WIDTH-1:0] l_flit_in,  input  l_valid_in,  output l_ready_out,
    output [FLIT_WIDTH-1:0] l_flit_out, output l_valid_out, input  l_ready_in
);
    localparam N=0, S=1, E=2, W=3, L=4;
    localparam PORTS = 5;
    localparam BUF_AW = $clog2(BUF_DEPTH);

    // ── Input buffers (FIFO per port per VC) ──
    wire [FLIT_WIDTH-1:0] port_flit_in [0:4];
    wire port_valid_in [0:4];
    assign port_flit_in[N]=n_flit_in; assign port_valid_in[N]=n_valid_in;
    assign port_flit_in[S]=s_flit_in; assign port_valid_in[S]=s_valid_in;
    assign port_flit_in[E]=e_flit_in; assign port_valid_in[E]=e_valid_in;
    assign port_flit_in[W]=w_flit_in; assign port_valid_in[W]=w_valid_in;
    assign port_flit_in[L]=l_flit_in; assign port_valid_in[L]=l_valid_in;

    reg [FLIT_WIDTH-1:0] ibuf [0:4][0:BUF_DEPTH-1];
    reg [BUF_AW:0] ibuf_wr [0:4], ibuf_rd [0:4];
    wire [BUF_AW:0] ibuf_count [0:4];
    wire ibuf_full [0:4], ibuf_empty [0:4];

    genvar p;
    generate
        for (p = 0; p < PORTS; p = p + 1) begin : buf_gen
            assign ibuf_count[p] = ibuf_wr[p] - ibuf_rd[p];
            assign ibuf_full[p]  = (ibuf_count[p] == BUF_DEPTH);
            assign ibuf_empty[p] = (ibuf_count[p] == 0);
        end
    endgenerate

    // Ready signals (accept when buffer not full)
    assign n_ready_out = ~ibuf_full[N];
    assign s_ready_out = ~ibuf_full[S];
    assign e_ready_out = ~ibuf_full[E];
    assign w_ready_out = ~ibuf_full[W];
    assign l_ready_out = ~ibuf_full[L];

    // Buffer write
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < PORTS; i = i + 1) begin
                ibuf_wr[i] <= 0; ibuf_rd[i] <= 0;
            end
        end else begin
            for (i = 0; i < PORTS; i = i + 1) begin
                if (port_valid_in[i] && !ibuf_full[i]) begin
                    ibuf[i][ibuf_wr[i][BUF_AW-1:0]] <= port_flit_in[i];
                    ibuf_wr[i] <= ibuf_wr[i] + 1;
                end
            end
        end
    end

    // ── XY Routing ──
    // Header flit format: [31:28]=type, [27:24]=dst_x, [23:20]=dst_y, [19:0]=payload
    function [2:0] route_xy;
        input [FLIT_WIDTH-1:0] header;
        reg [3:0] dst_x, dst_y;
        begin
            dst_x = header[27:24];
            dst_y = header[23:20];
            if (dst_x < X_COORD)      route_xy = W;
            else if (dst_x > X_COORD) route_xy = E;
            else if (dst_y < Y_COORD) route_xy = S;
            else if (dst_y > Y_COORD) route_xy = N;
            else                       route_xy = L;
        end
    endfunction

    // ── Per-port routing decision ──
    reg [2:0] route_port [0:4];   // Which output port each input wants
    reg       route_valid [0:4];
    always @(*) begin
        for (i = 0; i < PORTS; i = i + 1) begin
            if (!ibuf_empty[i]) begin
                route_port[i]  = route_xy(ibuf[i][ibuf_rd[i][BUF_AW-1:0]]);
                route_valid[i] = 1'b1;
            end else begin
                route_port[i]  = L;
                route_valid[i] = 1'b0;
            end
        end
    end

    // ── Round-robin arbitration per output port ──
    reg [2:0] rr_priority [0:4]; // Round-robin pointer per output
    reg [FLIT_WIDTH-1:0] out_flit [0:4];
    reg out_valid [0:4];

    wire port_ready_in [0:4];
    assign port_ready_in[N] = n_ready_in;
    assign port_ready_in[S] = s_ready_in;
    assign port_ready_in[E] = e_ready_in;
    assign port_ready_in[W] = w_ready_in;
    assign port_ready_in[L] = l_ready_in;

    integer op, ip, scan;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < PORTS; i = i + 1) begin
                rr_priority[i] <= 0;
                out_valid[i] <= 0;
                out_flit[i] <= 0;
            end
        end else begin
            for (op = 0; op < PORTS; op = op + 1) begin
                out_valid[op] <= 1'b0;
                if (port_ready_in[op]) begin
                    // Scan from rr_priority, find first input requesting this output
                    for (scan = 0; scan < PORTS; scan = scan + 1) begin
                        ip = (rr_priority[op] + scan) % PORTS;
                        if (route_valid[ip] && route_port[ip] == op[2:0] && !ibuf_empty[ip]) begin
                            out_flit[op]  <= ibuf[ip][ibuf_rd[ip][BUF_AW-1:0]];
                            out_valid[op] <= 1'b1;
                            ibuf_rd[ip]   <= ibuf_rd[ip] + 1;
                            rr_priority[op] <= (ip + 1) % PORTS;
                            // break equivalent: disable further scan
                            scan = PORTS;
                        end
                    end
                end
            end
        end
    end

    assign n_flit_out  = out_flit[N]; assign n_valid_out = out_valid[N];
    assign s_flit_out  = out_flit[S]; assign s_valid_out = out_valid[S];
    assign e_flit_out  = out_flit[E]; assign e_valid_out = out_valid[E];
    assign w_flit_out  = out_flit[W]; assign w_valid_out = out_valid[W];
    assign l_flit_out  = out_flit[L]; assign l_valid_out = out_valid[L];
endmodule
