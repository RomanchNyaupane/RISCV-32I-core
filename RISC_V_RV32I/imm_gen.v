module imm_gen (imm_gen_in, imm_gen_instr_type, imm_gen_out, imm_gen_instr_wr_en);
    input wire [31:0] imm_gen_in;
    input wire [3:0] imm_gen_instr_type;
    input wire imm_gen_instr_wr_en;
    
    output reg [31:0] imm_gen_out;
    reg [31:0] imm_reg;
    reg[3:0] instr_type;
    always@(*) begin
        if(imm_gen_instr_wr_en) instr_type = imm_gen_instr_type;
        case (instr_type)
            4'b0001: imm_reg = 31'b0; //r_type instruction - no immediate value
            4'b0010, 4'b0011, 4'b0100, 4'b0101: begin                //i_type_1 instruction
                imm_reg = (imm_gen_in[31] == 0)? {21'h0, imm_gen_in[30:20]} : {21'hFFFFF1, imm_gen_in[30:20]};
            end
            4'b0110: begin
                imm_reg = (imm_gen_in[31] == 0)? {21'h0, {imm_gen_in[30:25],imm_gen_in[11:7]}} : {21'hFFFFF1, {imm_gen_in[30:25],imm_gen_in[11:7]}};
            end
            4'b0111: begin //b type
                imm_reg = (imm_gen_in[31] == 0)? {20'b0, {imm_gen_in[7], imm_gen_in[30:25], imm_gen_in[11:8]}, 1'b0 } : {20'hFFFFF, imm_gen_in[7], imm_gen_in[30:25], imm_gen_in[11:8], 1'b0 };
            end

            default: imm_reg = 32'b0;
        endcase
        imm_gen_out = imm_reg;
    end
endmodule