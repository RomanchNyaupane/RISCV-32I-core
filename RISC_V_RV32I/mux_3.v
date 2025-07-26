module mux_3 (
    input  wire [31:0] mux_3_in0,
    input  wire [31:0] mux_3_in1,
    input  wire [31:0] mux_3_in2,
    input  wire [1:0]  mux_3_sel,
    output wire [31:0] mux_3_out
);

assign mux_3_out = (mux_3_sel == 2'b00)? mux_3_in0 :
                   (mux_3_sel == 2'b01)? mux_3_in1 : 
                   (mux_3_sel == 2'b10)? mux_3_in2 : 32'b0;

endmodule