module reg_bank_address_register (
    input wire [4:0] rs_1_in, rs_2_in, rd_in,
    input wire rs_1_wr_en, rs_2_wr_en, rd_wr_en, reg_clk, reg_rst,
    output reg [4:0] rs_1_out, rs_2_out, rd_out
);
reg [4:0] address[0:2]; //address[0] for rs1, address[1] for rs2 and address[2] for rd 
always @(posedge reg_clk ) begin
    if (reg_rst) begin
        address[0] <= 5'b0;
        address[1] <= 5'b0;
        address[2] <= 5'b0;
    end else begin
        if (rs_1_wr_en) address[0] <= rs_1_in;
        if (rs_2_wr_en) address[1] <= rs_2_in;
        if (rd_wr_en) address[2] <= rd_in;
    end
end

always @ (*) begin
    if (rs_1_wr_en) rs_1_out = rs_1_in; else rs_1_out <= address[0];
    if (rs_2_wr_en) rs_2_out = rs_2_in; else rs_2_out <= address[1];
    if (rd_wr_en) rd_out <= rd_in; else rd_out <= address[2];
end
    
endmodule