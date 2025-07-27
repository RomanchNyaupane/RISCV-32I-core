module instruction_counter(ic_out, ic_clk, ic_rst, ic_dir, ic_cnt, ic_in, ic_wr_en);
    input wire ic_clk, ic_rst, ic_dir, ic_cnt, ic_wr_en;
    input wire [4:0] ic_in;
// dir = 1 for decremental count else incremental count
    output reg [4:0] ic_out;

    reg [4:0] instruction_count;
    always @(posedge ic_clk) begin
        if(ic_rst) instruction_count <= 5'b0;
        else begin
            if(ic_wr_en) instruction_count = ic_in;
            if(ic_cnt & !ic_wr_en)
                if (ic_dir) instruction_count <= instruction_count - 1;
                else instruction_count <= instruction_count + 1;
        end
    end
    always @(*) begin
        ic_out = instruction_count;
    end

endmodule