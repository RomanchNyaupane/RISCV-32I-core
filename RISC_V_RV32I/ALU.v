module alu (alu_in_1, alu_in_2, alu_out, alu_opcode, alu_carry, alu_zero);
    input wire [31:0] alu_in_1, alu_in_2;
    input wire [3:0] alu_opcode;

    output reg alu_carry, alu_zero;
    output reg [31:0] alu_out;

    reg [32:0] alu_result;

    always @(*) begin
        case (alu_opcode)
            4'b0001: alu_result = alu_in_1 + alu_in_2;
            4'b0010: alu_result = alu_in_1 - alu_in_2;
            4'b0011: alu_result = alu_in_1 ^ alu_in_2;
            4'b0100: alu_result = alu_in_1 | alu_in_2;
            4'b0101: alu_result = alu_in_1 & alu_in_2;
            4'b0110: alu_result = alu_in_1 << alu_in_2[4:0];
            4'b0111: alu_result = alu_in_1 >> alu_in_2[4:0];
            4'b1000: alu_result = $signed({1'd0, alu_in_1}) >>> alu_in_2[4:0];
            4'b1001: alu_result = ($signed(alu_in_1) < $signed(alu_in_2)) ? 33'b1 : 33'b0;
            4'b1010: alu_result = (alu_in_1 < alu_in_2) ? 33'b1 : 33'b0;
            default: alu_result = 33'b0;
        endcase
        alu_carry = alu_zero;
        alu_zero = (alu_result == 33'b0);
        alu_out = alu_result[31:0];
        
    end
endmodule