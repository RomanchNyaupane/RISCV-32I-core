module memory_address_register (mar_in, mar_out, mar_wr_en, mar_clk, mar_rst);
    input wire [31:0] mar_in;
    input wire mar_wr_en, mar_clk, mar_rst;

    output reg [31:0] mar_out;

    reg [31:0] memory_address_register;

    always @(posedge mar_clk) begin
        if (mar_rst) memory_address_register <= 32'b0;
        else if(mar_wr_en)  memory_address_register <= mar_in;
    end
    always @(*) begin
    if(mar_wr_en) mar_out = mar_in;
    else mar_out = memory_address_register;
    end
endmodule