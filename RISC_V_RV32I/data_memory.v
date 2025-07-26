// 32 bit in and 32 bit out. 16 bit support not included currently
module data_memory #(
    parameter MEMORY_DEPTH = 8,// 8 bits deep
    parameter ADDRESS_WIDTH = 32, //32 bit addressing
    parameter SIZE = 16'h0FFF
)(
    data_mem_in, data_mem_out, data_mem_addr, data_mem_clk, data_mem_wr_en
);
input wire [31:0] data_mem_in;
input wire [29:0] data_mem_addr;
input wire data_mem_clk, data_mem_wr_en;

output reg [31:0] data_mem_out;


reg [MEMORY_DEPTH-1:0] data_mem_bank_3 [0: SIZE - 1];
reg [MEMORY_DEPTH-1:0] data_mem_bank_2 [0: SIZE - 1];
reg [MEMORY_DEPTH-1:0] data_mem_bank_1 [0: SIZE - 1];
reg [MEMORY_DEPTH-1:0] data_mem_bank_0 [0: SIZE - 1];

always @(posedge data_mem_clk) begin   
        data_mem_bank_3[0] <= 8'b0;
        data_mem_bank_2[0] <= 8'b1;
        data_mem_bank_1[0] <= 8'b0;
        data_mem_bank_0[0] <= 8'b100;
        
        data_mem_bank_3[1] <= 8'b0;
        data_mem_bank_2[1] <= 8'b00100000;
        data_mem_bank_1[1] <= 8'b00010000;
        data_mem_bank_0[1] <= 8'b10010010;
    if (data_mem_wr_en) begin
        data_mem_bank_3[data_mem_addr] <= data_mem_in[31:24];
        data_mem_bank_2[data_mem_addr] <= data_mem_in[23:16];
        data_mem_bank_1[data_mem_addr] <= data_mem_in[15:8];
        data_mem_bank_0[data_mem_addr] <= data_mem_in[7:0];

    end
    data_mem_out <={
                    data_mem_bank_3[data_mem_addr], 
                    data_mem_bank_2[data_mem_addr], 
                    data_mem_bank_1[data_mem_addr], 
                    data_mem_bank_0[data_mem_addr]
                   };
    
end

endmodule;