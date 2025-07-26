module instruction_register(ir_in, ir_out, ir_wr_en, ir_rst, ir_clk);
    input wire [31:0] ir_in;
    input wire ir_wr_en, ir_clk, ir_rst;

    output reg [31:0] ir_out;
    reg [31:0] instruction_register;

    always @(posedge ir_clk) begin
        if(ir_rst) instruction_register <= 32'b0;
        else begin
            if(ir_wr_en) instruction_register <= ir_in;
        end
    end
    always @(*) begin
        if(ir_wr_en) ir_out = ir_in; else ir_out = instruction_register;
    end
endmodule