module register_bank(reg_rd_addr_1, reg_rd_addr_2, reg_wr_addr, reg_wr_en, reg_in, reg_out_1, reg_out_2, reg_clk, reg_rst);
input wire [4:0] reg_rd_addr_1, reg_rd_addr_2, reg_wr_addr;
input wire reg_clk, reg_wr_en, reg_rst;
input wire [31:0] reg_in;

output wire [31:0] reg_out_1;
output wire [31:0] reg_out_2;
reg [31:0] register_bank [0:31];

integer i;
always @ (posedge reg_clk) begin
    if(reg_rst) begin
        for(i = 0; i < 32; i = i + 1)
            register_bank[i] <= 32'b0;
    end else begin
    if (reg_wr_en) begin
        register_bank[reg_wr_addr] <= reg_in;
    end
    end

end

assign reg_out_1 = register_bank[reg_rd_addr_1];
assign reg_out_2 = register_bank[reg_rd_addr_2];
endmodule