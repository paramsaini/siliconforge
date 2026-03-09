module shift_registers_1 (clk, clken, SI, SO);
parameter WIDTH = 32;
input clk, clken, SI;
output SO;
reg [WIDTH-1:0] shreg;

integer i;
always @(posedge clk)
begin
if (clken)
begin
for (i = 0; i < WIDTH-1; i = i+1)
shreg[i+1] <= shreg[i];
shreg[0] <= SI;
end
end
assign SO = shreg[WIDTH-1];
endmodule
