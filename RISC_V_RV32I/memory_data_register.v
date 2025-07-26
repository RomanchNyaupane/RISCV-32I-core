module memory_data_register( mdr_in, mdr_out, mdr_rd, mdr_rst, mdr_clk);
    input wire [31:0] mdr_in;
    input mdr_rd, mdr_rst, mdr_clk;

    output reg [31:0] mdr_out;

    reg [31:0] memory_data_register;

    always @(posedge mdr_clk ) begin
        if (mdr_rst) mdr_out <= 32'b0; 
        else begin
            memory_data_register <= mdr_in;

        end
    end
    always @(*) begin
            if(mdr_rd) mdr_out = mdr_in;
            else mdr_out = memory_data_register;
    end

endmodule