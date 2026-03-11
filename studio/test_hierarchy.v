// test_hierarchy.v — Module instantiation test
module inverter (input a, output y);
    assign y = ~a;
endmodule

module top (input x, output z);
    wire w;
    inverter u1 (.a(x), .y(w));
    assign z = w;
endmodule
