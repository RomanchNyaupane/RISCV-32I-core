module alu (alu_in_1, alu_in_2, alu_out, alu_opcode, alu_carry, alu_zero, alu_opcode_wr);
    input wire [31:0] alu_in_1, alu_in_2;
    input wire [3:0] alu_opcode;
    input wire alu_opcode_wr;

    output reg alu_carry, alu_zero;
    output reg [31:0] alu_out;

    reg [32:0] alu_result;
    reg [3:0] alu_opcode_reg;

    always @(*) begin
        if(alu_opcode_wr) alu_opcode_reg = alu_opcode;
        case (alu_opcode_reg)
            0001: alu_result = alu_in_1 + alu_in_2;
            0010: alu_result = alu_in_1 - alu_in_2;
            0011: alu_result = alu_in_1 ^ alu_in_2;
            0100: alu_result = alu_in_1 | alu_in_2;
            0101: alu_result = alu_in_1 & alu_in_2;
            0110: alu_result = alu_in_1 << alu_in_2[4:0];
            0111: alu_result = alu_in_1 >> alu_in_2[4:0];
            1000: alu_result = $signed(alu_in_1) >>> alu_in_2[4:0];
            1001: alu_result = ($signed(alu_in_1) < $signed(alu_in_2)) ? 33'b1 : 33'b0;
            1010: alu_result = (alu_in_1 < alu_in_2) ? 33'b1 : 33'b0;
            default: alu_result = 32'b0;
        endcase
        alu_carry = alu_zero;
        alu_zero = (alu_result == 33'b0);
        alu_out = alu_result[31:0];
        
    end
endmodule