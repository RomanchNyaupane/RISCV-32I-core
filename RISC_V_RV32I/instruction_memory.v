module instruction_memory #(
    parameter ADDRESS_WIDTH = 5  // 2^5 = 32 addresses max
)(
    output reg [31:0] instr_mem_out,
    input wire [ADDRESS_WIDTH-1:0] instr_mem_addr,
    input wire instr_mem_clk
);

always @(instr_mem_addr) begin
    case (instr_mem_addr)
        5'd0:  instr_mem_out <= 32'b00000000_00000001_10100000_00000011; // load at register 0(memory read)
        5'd1:  instr_mem_out <= 32'b00000000_00010001_10100000_10000011; // load at register 1 (memory read)
        5'd2:  instr_mem_out <= 32'b00000000_00000000_10000001_00110011; // add and store at destination register
        5'd3:  instr_mem_out <= 32'b00000000_00100001_10100001_00100011; // store register at memory (memory write)
        5'd4:  instr_mem_out <= 32'b00000000_10010001_10000010_00010011; // add immediate rd = rs1 + imm. loads 09 to register 5
        5'd5:  instr_mem_out <= 32'b01000000_01000000_11010010_10110011; // shift right arithmetic
        5'd6:  instr_mem_out <= 32'b00000000_00000000_10010100_01100011; // branch at rs1 != rs2 to memory location 14
        5'd14: instr_mem_out <= 32'b00000000_10110001_10000010_00010011; // add immediate rd = rs1 + imm. loads 0b in register 5
        default: instr_mem_out <= 32'b0; 
    endcase
end

endmodule
