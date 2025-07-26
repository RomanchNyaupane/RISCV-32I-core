// 32 bit in and 32 bit out. 16 bit support not included currently
module instruction_memory #(
    parameter MEMORY_DEPTH = 8,// 8 bits deep
    parameter ADDRESS_WIDTH = 32, //32 bit addressing
    parameter SIZE = 16'h0FFF
)(
    instr_mem_out, instr_mem_addr, instr_mem_clk
);
input wire [4:0] instr_mem_addr;
input wire instr_mem_clk;

output reg [31:0] instr_mem_out;


reg [MEMORY_DEPTH-1:0] instr_mem_bank_3 [0: SIZE - 1];
reg [MEMORY_DEPTH-1:0] instr_mem_bank_2 [0: SIZE - 1];
reg [MEMORY_DEPTH-1:0] instr_mem_bank_1 [0: SIZE - 1];
reg [MEMORY_DEPTH-1:0] instr_mem_bank_0 [0: SIZE - 1];

always @(posedge instr_mem_clk) begin
    instr_mem_bank_3[0] <= 8'b0;            // load at register 0(memory read)
    instr_mem_bank_2[0] <= 8'b1;
    instr_mem_bank_1[0] <= 8'b10100000;
    instr_mem_bank_0[0] <= 8'b11;
    
    instr_mem_bank_3[1] <= 8'b0;            // load at register 1 (memory read)
    instr_mem_bank_2[1] <= 8'b00010001;
    instr_mem_bank_1[1] <= 8'b10100000;
    instr_mem_bank_0[1] <= 8'b10000011;
    
    instr_mem_bank_3[2] <= 8'b0;            // add and store at destination register
    instr_mem_bank_2[2] <= 8'b0;
    instr_mem_bank_1[2] <= 8'b10000001;
    instr_mem_bank_0[2] <= 8'b00110011;

    instr_mem_bank_3[3] <= 8'b0;            // store register at memory (memory write)
    instr_mem_bank_2[3] <= 8'b00100001;
    instr_mem_bank_1[3] <= 8'b10100001;
    instr_mem_bank_0[3] <= 8'b00100011;

    instr_mem_out <={
                    instr_mem_bank_3[instr_mem_addr], 
                    instr_mem_bank_2[instr_mem_addr], 
                    instr_mem_bank_1[instr_mem_addr], 
                    instr_mem_bank_0[instr_mem_addr]
                   };
    
end

endmodule;