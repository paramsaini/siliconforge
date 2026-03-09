// SiliconForge Test Design — 8-bit 2:1 Multiplexer with Register
// Structural gate-level netlist for end-to-end flow testing.
// This design has enough gates and nets to show meaningful visualization.

module mux8_reg (a0, a1, a2, a3, a4, a5, a6, a7,
                 b0, b1, b2, b3, b4, b5, b6, b7,
                 sel, clk, rst,
                 y0, y1, y2, y3, y4, y5, y6, y7);

  input a0, a1, a2, a3, a4, a5, a6, a7;
  input b0, b1, b2, b3, b4, b5, b6, b7;
  input sel, clk, rst;
  output y0, y1, y2, y3, y4, y5, y6, y7;

  wire sel_n;
  wire sa0, sa1, sa2, sa3, sa4, sa5, sa6, sa7;
  wire sb0, sb1, sb2, sb3, sb4, sb5, sb6, sb7;
  wire m0, m1, m2, m3, m4, m5, m6, m7;

  // Invert select
  not U_inv (sel_n, sel);

  // AND with sel_n for 'a' inputs
  and U_a0 (sa0, a0, sel_n);
  and U_a1 (sa1, a1, sel_n);
  and U_a2 (sa2, a2, sel_n);
  and U_a3 (sa3, a3, sel_n);
  and U_a4 (sa4, a4, sel_n);
  and U_a5 (sa5, a5, sel_n);
  and U_a6 (sa6, a6, sel_n);
  and U_a7 (sa7, a7, sel_n);

  // AND with sel for 'b' inputs
  and U_b0 (sb0, b0, sel);
  and U_b1 (sb1, b1, sel);
  and U_b2 (sb2, b2, sel);
  and U_b3 (sb3, b3, sel);
  and U_b4 (sb4, b4, sel);
  and U_b5 (sb5, b5, sel);
  and U_b6 (sb6, b6, sel);
  and U_b7 (sb7, b7, sel);

  // OR to form mux output
  or U_m0 (m0, sa0, sb0);
  or U_m1 (m1, sa1, sb1);
  or U_m2 (m2, sa2, sb2);
  or U_m3 (m3, sa3, sb3);
  or U_m4 (m4, sa4, sb4);
  or U_m5 (m5, sa5, sb5);
  or U_m6 (m6, sa6, sb6);
  or U_m7 (m7, sa7, sb7);

  // Output assignments (directly from mux for structural netlist)
  assign y0 = m0;
  assign y1 = m1;
  assign y2 = m2;
  assign y3 = m3;
  assign y4 = m4;
  assign y5 = m5;
  assign y6 = m6;
  assign y7 = m7;

endmodule
