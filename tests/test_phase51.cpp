// test_phase51.cpp — SystemVerilog Phase 3: Interfaces & Modports
// Tests: interface/endinterface, modport, interface as port type,
//        interface signal expansion, mixed with Phase 1/2 features

#include "frontend/verilog_parser.hpp"
#include "synth/behavioral_synth.hpp"
#include <cassert>
#include <iostream>

using namespace sf;

static VerilogParseResult parse_sv(const std::string& src, Netlist& nl, VerilogParser& vp) {
    auto r = vp.parse_string(src, nl);
    if (!r.success) {
        std::cerr << "Parse failed: " << r.error << "\n";
        std::cerr << "Source:\n" << src << "\n";
    }
    return r;
}

// =============================================================================
// TEST 1: Basic interface definition — no crash
// =============================================================================
static void test_interface_basic() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface simple_bus;
            logic [7:0] data;
            logic       valid;
            logic       ready;
        endinterface

        module top(input clk, output reg out);
            assign out = clk;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: basic interface definition\n";
}

// =============================================================================
// TEST 2: Interface with modport
// =============================================================================
static void test_interface_modport() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface axi_lite;
            logic [31:0] addr;
            logic [31:0] wdata;
            logic [31:0] rdata;
            logic        valid;
            logic        ready;
            modport master (output addr, output wdata, input rdata, output valid, input ready);
            modport slave  (input addr, input wdata, output rdata, input valid, output ready);
        endinterface

        module top(input clk, output reg out);
            assign out = clk;
        endmodule
    )", nl, vp);
    assert(r.success);
    std::cout << "  PASS: interface with modport\n";
}

// =============================================================================
// TEST 3: Interface port with modport — master direction
// =============================================================================
static void test_interface_port_master() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface simple_if;
            logic [7:0] data;
            logic       valid;
            modport master (output data, output valid);
            modport slave  (input data, input valid);
        endinterface

        module sender(simple_if.master bus);
            assign bus_data = 8'hFF;
            assign bus_valid = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    // master: data(8 output) + valid(1 output) = 9 outputs
    assert(r.num_outputs == 9);
    std::cout << "  PASS: interface port master (9 output bits)\n";
}

// =============================================================================
// TEST 4: Interface port with modport — slave direction
// =============================================================================
static void test_interface_port_slave() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface simple_if;
            logic [7:0] data;
            logic       valid;
            modport master (output data, output valid);
            modport slave  (input data, input valid);
        endinterface

        module receiver(simple_if.slave bus);
            wire [7:0] local_data;
            assign local_data = bus_data;
        endmodule
    )", nl, vp);
    assert(r.success);
    // slave: data(8 input) + valid(1 input) = 9 inputs
    assert(r.num_inputs == 9);
    std::cout << "  PASS: interface port slave (9 input bits)\n";
}

// =============================================================================
// TEST 5: Interface port without modport (all as wires)
// =============================================================================
static void test_interface_port_no_modport() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface data_bus;
            logic [15:0] data;
            logic        en;
        endinterface

        module passthrough(data_bus bus);
            wire [15:0] local;
            assign local = bus_data;
        endmodule
    )", nl, vp);
    assert(r.success);
    // Without modport, signals are wires: 16 + 1 = 17 wires
    assert(r.num_wires >= 17);
    std::cout << "  PASS: interface port without modport (17 wires)\n";
}

// =============================================================================
// TEST 6: Interface with mixed signal widths
// =============================================================================
static void test_interface_mixed_widths() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface mem_if;
            logic [31:0] addr;
            logic [63:0] data;
            logic [3:0]  be;
            logic        we;
            logic        re;
            modport ctrl (output addr, output data, output be, output we, output re);
        endinterface

        module mem_ctrl(mem_if.ctrl bus);
            assign bus_we = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    // ctrl: addr(32) + data(64) + be(4) + we(1) + re(1) = 102 outputs
    assert(r.num_outputs == 102);
    std::cout << "  PASS: interface mixed widths (102 output bits)\n";
}

// =============================================================================
// TEST 7: Multiple modports in one interface
// =============================================================================
static void test_multi_modport() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface wishbone;
            logic [31:0] adr;
            logic [31:0] dat_m;
            logic [31:0] dat_s;
            logic        stb;
            logic        ack;
            modport master (output adr, output dat_m, input dat_s, output stb, input ack);
            modport slave  (input adr, input dat_m, output dat_s, input stb, output ack);
        endinterface

        module wb_master(wishbone.master wb);
            assign wb_stb = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    // master: adr(32 out) + dat_m(32 out) + dat_s(32 in) + stb(1 out) + ack(1 in)
    assert(r.num_outputs == 65); // 32+32+1
    assert(r.num_inputs == 33);  // 32+1
    std::cout << "  PASS: multiple modports (master view)\n";
}

// =============================================================================
// TEST 8: Interface with regular ports
// =============================================================================
static void test_interface_plus_regular_ports() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface ctrl_if;
            logic [7:0] cmd;
            logic       valid;
            modport sender (output cmd, output valid);
        endinterface

        module controller(input clk, input rst, ctrl_if.sender ctrl, output reg done);
            assign ctrl_cmd = 8'h42;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 2);   // clk + rst
    assert(r.num_outputs >= 9);  // cmd(8) + valid(1) from interface + done(1) = 10
    std::cout << "  PASS: interface + regular ports\n";
}

// =============================================================================
// TEST 9: Interface with byte type signal
// =============================================================================
static void test_interface_byte_signal() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface byte_if;
            byte payload;
            logic valid;
            modport tx (output payload, output valid);
        endinterface

        module transmitter(byte_if.tx port);
            assign port_valid = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    // tx: payload(8) + valid(1) = 9 outputs
    assert(r.num_outputs == 9);
    std::cout << "  PASS: interface with byte signal\n";
}

// =============================================================================
// TEST 10: Empty interface
// =============================================================================
static void test_empty_interface() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface empty_if;
        endinterface

        module top(input a, output y);
            assign y = a;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 1);
    assert(r.num_outputs == 1);
    std::cout << "  PASS: empty interface (no crash)\n";
}

// =============================================================================
// TEST 11: Two interfaces in same file
// =============================================================================
static void test_two_interfaces() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface if_a;
            logic [3:0] sig_a;
            modport master (output sig_a);
        endinterface

        interface if_b;
            logic [7:0] sig_b;
            modport master (output sig_b);
        endinterface

        module dual(if_a.master a, if_b.master b);
            assign a_sig_a = 4'b0;
            assign b_sig_b = 8'b0;
        endmodule
    )", nl, vp);
    assert(r.success);
    // a: sig_a(4 out), b: sig_b(8 out) = 12 outputs
    assert(r.num_outputs == 12);
    std::cout << "  PASS: two interfaces (12 output bits)\n";
}

// =============================================================================
// TEST 12: Interface + package + typedef combined
// =============================================================================
static void test_interface_package_typedef() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package types;
            typedef logic [7:0] byte_t;
        endpackage

        interface data_if;
            logic [7:0] data;
            logic       valid;
            modport source (output data, output valid);
        endinterface

        module producer(input clk, data_if.source dbus, output reg [7:0] debug);
            import types::*;
            byte_t tmp;
            always_ff @(posedge clk)
                debug <= tmp;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    std::cout << "  PASS: interface + package + typedef\n";
}

// =============================================================================
// TEST 13: Interface modport with inout
// =============================================================================
static void test_interface_inout() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface bidir_if;
            logic [7:0] data;
            logic       oe;
            modport port (inout data, input oe);
        endinterface

        module bidir_ctrl(bidir_if.port bus);
            wire [7:0] local;
            assign local = bus_data;
        endmodule
    )", nl, vp);
    assert(r.success);
    // inout treated as output, input oe = 1 input
    assert(r.num_outputs == 8);
    assert(r.num_inputs == 1);
    std::cout << "  PASS: interface with inout\n";
}

// =============================================================================
// TEST 14: Interface with multiple signals per modport direction
// =============================================================================
static void test_interface_multi_signal_dir() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface spi_if;
            logic sclk;
            logic mosi;
            logic miso;
            logic cs_n;
            modport master (output sclk, output mosi, input miso, output cs_n);
        endinterface

        module spi_master(spi_if.master spi);
            assign spi_sclk = 1'b0;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_outputs == 3); // sclk + mosi + cs_n
    assert(r.num_inputs == 1);  // miso
    std::cout << "  PASS: modport multi-signal directions\n";
}

// =============================================================================
// TEST 15: Interface with always_ff using expanded signals
// =============================================================================
static void test_interface_with_always() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface req_if;
            logic [7:0] data;
            logic       req;
            logic       ack;
            modport initiator (output data, output req, input ack);
        endinterface

        module requester(input clk, req_if.initiator bus);
            always_ff @(posedge clk) begin
                bus_req <= 1'b1;
                bus_data <= 8'hAB;
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    assert(r.num_inputs >= 1); // clk + ack
    std::cout << "  PASS: interface with always_ff\n";
}

// =============================================================================
// TEST 16: Interface with wide bus signals
// =============================================================================
static void test_interface_wide_bus() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface axi_data;
            logic [255:0] data;
            logic [31:0]  strb;
            logic         last;
            modport writer (output data, output strb, output last);
        endinterface

        module writer(axi_data.writer ch);
            assign ch_last = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    // writer: data(256) + strb(32) + last(1) = 289 outputs
    assert(r.num_outputs == 289);
    std::cout << "  PASS: interface wide bus (289 output bits)\n";
}

// =============================================================================
// TEST 17: Interface port + enum typedef in same module
// =============================================================================
static void test_interface_enum_combo() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef enum logic [1:0] {IDLE, BUSY, DONE, ERR} status_t;

        interface status_if;
            logic [1:0] status;
            modport reporter (output status);
        endinterface

        module device(input clk, status_if.reporter stat, output reg [1:0] out);
            status_t s;
            always_ff @(posedge clk)
                out <= s;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_outputs >= 2); // status(2) from interface + out(2)
    std::cout << "  PASS: interface + enum typedef\n";
}

// =============================================================================
// TEST 18: Interface with scalar signals only
// =============================================================================
static void test_interface_scalar() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface handshake;
            logic req;
            logic ack;
            modport sender   (output req, input ack);
            modport receiver (input req, output ack);
        endinterface

        module producer(handshake.sender hs);
            assign hs_req = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_outputs == 1); // req
    assert(r.num_inputs == 1);  // ack
    std::cout << "  PASS: interface scalar signals\n";
}

// =============================================================================
// TEST 19: Interface slave view of same interface
// =============================================================================
static void test_interface_slave_view() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface handshake;
            logic req;
            logic ack;
            modport sender   (output req, input ack);
            modport receiver (input req, output ack);
        endinterface

        module consumer(handshake.receiver hs);
            assign hs_ack = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 1);  // req
    assert(r.num_outputs == 1); // ack
    std::cout << "  PASS: interface slave/receiver view\n";
}

// =============================================================================
// TEST 20: Interface with multiple signals declared on same line
// =============================================================================
static void test_interface_multi_decl() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface gpio_if;
            logic pin0, pin1, pin2, pin3;
            modport out (output pin0, output pin1, output pin2, output pin3);
        endinterface

        module gpio_driver(gpio_if.out gpio);
            assign gpio_pin0 = 1'b0;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_outputs == 4);
    std::cout << "  PASS: interface multi-signal declaration\n";
}

// =============================================================================
// TEST 21: Full SV Phase 1+2+3 integration
// =============================================================================
static void test_full_integration() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        package cpu_pkg;
            typedef enum logic [3:0] {
                NOP, LOAD, STORE, ADD, SUB, AND, OR, XOR,
                SHL, SHR, BEQ, BNE, JMP, HALT
            } opcode_t;
            typedef logic [31:0] word_t;
        endpackage

        interface mem_bus;
            logic [31:0] addr;
            logic [31:0] wdata;
            logic [31:0] rdata;
            logic        we;
            logic        re;
            modport master (output addr, output wdata, input rdata, output we, output re);
            modport slave  (input addr, input wdata, output rdata, input we, input re);
        endinterface

        module cpu_core(
            input logic clk,
            input logic rst,
            mem_bus.master mem,
            output logic [3:0] state_out
        );
            import cpu_pkg::*;
            opcode_t current_op;
            word_t   pc;
            word_t   acc;

            always_ff @(posedge clk) begin
                if (rst) begin
                    pc <= 32'b0;
                    acc <= 32'b0;
                end else begin
                    pc <= pc + 32'd4;
                end
            end

            always_comb begin
                priority case (current_op)
                    LOAD:    state_out = 4'b0001;
                    STORE:   state_out = 4'b0010;
                    default: state_out = 4'b0000;
                endcase
            end
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(vp.has_behavioral_blocks);
    assert(r.num_inputs >= 2);  // clk, rst + mem interface inputs
    assert(r.num_outputs >= 4); // state_out + mem interface outputs
    std::cout << "  PASS: full Phase 1+2+3 integration\n";
}

// =============================================================================
// TEST 22: Interface with parameter in declaration (skip gracefully)
// =============================================================================
static void test_interface_with_param() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface param_if;
            parameter WIDTH = 8;
            logic [7:0] data;
            modport src (output data);
        endinterface

        module test(param_if.src port);
            assign port_data = 8'b0;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_outputs == 8);
    std::cout << "  PASS: interface with parameter\n";
}

// =============================================================================
// TEST 23: Two interface ports in same module
// =============================================================================
static void test_two_interface_ports() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface ch_if;
            logic [7:0] data;
            logic       valid;
            modport tx (output data, output valid);
            modport rx (input data, input valid);
        endinterface

        module bridge(ch_if.rx in_ch, ch_if.tx out_ch);
            assign out_ch_data = in_ch_data;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 9);  // in_ch: 8+1
    assert(r.num_outputs == 9); // out_ch: 8+1
    std::cout << "  PASS: two interface ports\n";
}

// =============================================================================
// TEST 24: Interface + struct packed port
// =============================================================================
static void test_interface_struct_port() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        typedef struct packed {
            logic [7:0] header;
            logic [7:0] payload;
        } frame_t;

        interface stream_if;
            logic [15:0] data;
            logic        valid;
            modport src (output data, output valid);
        endinterface

        module framer(input frame_t frame_in, stream_if.src stream);
            assign stream_data = frame_in;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_inputs == 16);   // frame_in (struct packed 16-bit)
    assert(r.num_outputs == 17);  // stream: data(16) + valid(1)
    std::cout << "  PASS: interface + struct packed port\n";
}

// =============================================================================
// TEST 25: Interface with bit type signals
// =============================================================================
static void test_interface_bit_signals() {
    Netlist nl; VerilogParser vp;
    auto r = parse_sv(R"(
        interface bit_if;
            bit [3:0] nibble;
            bit       flag;
            modport out (output nibble, output flag);
        endinterface

        module nibbler(bit_if.out port);
            assign port_flag = 1'b1;
        endmodule
    )", nl, vp);
    assert(r.success);
    assert(r.num_outputs == 5); // nibble(4) + flag(1)
    std::cout << "  PASS: interface with bit signals\n";
}

int main() {
    std::cout << "=== Phase 51: SystemVerilog Phase 3 — Interfaces & Modports ===\n";

    test_interface_basic();             // 1
    test_interface_modport();           // 2
    test_interface_port_master();       // 3
    test_interface_port_slave();        // 4
    test_interface_port_no_modport();   // 5
    test_interface_mixed_widths();      // 6
    test_multi_modport();               // 7
    test_interface_plus_regular_ports();// 8
    test_interface_byte_signal();       // 9
    test_empty_interface();             // 10
    test_two_interfaces();              // 11
    test_interface_package_typedef();   // 12
    test_interface_inout();             // 13
    test_interface_multi_signal_dir();  // 14
    test_interface_with_always();       // 15
    test_interface_wide_bus();          // 16
    test_interface_enum_combo();        // 17
    test_interface_scalar();            // 18
    test_interface_slave_view();        // 19
    test_interface_multi_decl();        // 20
    test_full_integration();            // 21
    test_interface_with_param();        // 22
    test_two_interface_ports();         // 23
    test_interface_struct_port();       // 24
    test_interface_bit_signals();       // 25

    std::cout << "=== All 25 Phase 51 tests PASSED ===\n";
    return 0;
}
