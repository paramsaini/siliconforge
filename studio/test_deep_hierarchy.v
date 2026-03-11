// test_deep_hierarchy.v — 3-level module hierarchy test
module inv (input a, output y);
    assign y = ~a;
endmodule

module and_gate (input a, input b, output y);
    assign y = a & b;
endmodule

module mux2 (input a, input b, input sel, output y);
    wire n_sel, and_a, and_b;
    inv u_inv (.a(sel), .y(n_sel));
    and_gate u_and0 (.a(a), .b(n_sel), .y(and_a));
    and_gate u_and1 (.a(b), .b(sel),   .y(and_b));
    assign y = and_a | and_b;
endmodule

module top (input d0, input d1, input s, output q);
    mux2 u_mux (.a(d0), .b(d1), .sel(s), .y(q));
endmodule
