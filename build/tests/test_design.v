module test_design ( a, b, c, clk, y );
  input a, b, c, clk;
  output y;
  wire w1, w2;
  assign w1 = a & b;
  assign w2 = w1 | c;
  assign y = w2;
  // assert property (@(posedge clk) a |=> y);
endmodule
