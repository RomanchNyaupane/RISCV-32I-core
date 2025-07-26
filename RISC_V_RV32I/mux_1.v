module mux_1 (
    input  wire [31:0] mux_1_in0,
    input  wire [31:0] mux_1_in1,
    input  wire        mux_1_sel,
    output wire [31:0] mux_1_out
);

assign mux_1_out = mux_1_sel ? mux_1_in1 : mux_1_in0;

endmodule