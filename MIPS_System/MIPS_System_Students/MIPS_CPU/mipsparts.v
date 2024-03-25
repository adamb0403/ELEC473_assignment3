
module regfile(input         clk, 
               input         we, 
               input  [4:0]  ra1, ra2, wa, 
               input  [31:0] wd, 
               output [31:0] rd1, rd2);

  reg [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clock
  // register 0 hardwired to 0

  always @(posedge clk)
    if (we) rf[wa] <= wd;	

  assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
endmodule


module alu(input      [31:0] a, b, 
           input      [3:0]  alucont, 
           output reg [31:0] result,
           output            zero);

  wire [31:0] b2, sum, slt;

  assign b2 = alucont[3] ? ~b:b; 
  assign sum = a + b2 + alucont[3];
  assign slt = sum[31];

  always@(*)
    case(alucont[2:0])
      3'b000: result <= a & b;
      3'b001: result <= a | b;
      3'b110: result <= sum;
      3'b111: result <= slt;
      3'b011: result <= ~(a | b); // nor
      default: result <= 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
    endcase

  assign zero = (result == 32'b0);

endmodule


module adder(input [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule



module sl2(input  [31:0] a,
           output [31:0] y);

  // shift left by 2
  assign y = {a[29:0], 2'b00};
endmodule



module sign_zero_ext(input      [15:0] a,
                     input             signext,
                     output reg [31:0] y);
              
   always @(*)
	begin
	   if (signext)  y <= {{16{a[15]}}, a[15:0]};
	   else          y <= {16'b0, a[15:0]};
	end

endmodule



module shift_left_16(input      [31:0] a,
		               input         shiftl16,
                     output reg [31:0] y);

   always @(*)
	begin
	   if (shiftl16) y = {a[15:0],16'b0};
	   else          y = a[31:0];
	end
              
endmodule



module flopr #(parameter WIDTH = 8)
              (input                  clk, reset,
               input      [WIDTH-1:0] d, 
               output reg [WIDTH-1:0] q);

  always @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;

endmodule



module flopenr #(parameter WIDTH = 8)
                (input                  clk, reset,
                 input                  en,
                 input      [WIDTH-1:0] d, 
                 output reg [WIDTH-1:0] q);
 
  always @(posedge clk, posedge reset)
    if      (reset) q <= 0;
    else if (en)    q <= d;

endmodule



module mux2 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, 
              input              s, 
              output [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 

endmodule

// lb instruction
module lb_mux (input [1:0] sel,
               input [7:0] mux_input0, mux_input1, mux_input2, mux_input3,
               output reg [7:0] mux_out);

  always @(*) begin
    case(sel)
      2'b00: mux_out <= mux_input0;
      2'b01: mux_out <= mux_input1;
      2'b10: mux_out <= mux_input2;
      2'b11: mux_out <= mux_input3;
    endcase
  end

endmodule

module sign_ext_24(input [7:0] data_in,
                   output reg [31:0] data_out);
              
  always @(*) begin
    data_out[7:0] <= data_in[7:0];
    if (data_in[7])
      data_out[31:8] <= 24'b111111111111111111111111;
    else
      data_out[31:8] <= 24'b000000000000000000000000;
	end

endmodule