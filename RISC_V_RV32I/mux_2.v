module mux_2 (
    input  wire [31:0] mux_2_in0,
    input  wire [31:0] mux_2_in1,
    input  wire        mux_2_sel,
    output wire [31:0] mux_2_out
);

assign mux_2_out = mux_2_sel ? mux_2_in1 : mux_2_in0;

endmodule