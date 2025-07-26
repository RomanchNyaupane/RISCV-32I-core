// the control unit is state machine based instead of microcode based that i was used to

/*
RISC V instruction decoding
R instruction: [31:25] - funct7
               [24:20] - rs2 address(source register)
               [19:15] - rs1 address
               [14:12] - funct3
               [11:7] - rd address (destination register)
               [6:0] - opcode
*/
module control_unit(instr_in, ctrl_clk, ctrl_rst,carry_in, zero_in,alu_opcode_wr_en, mem_wr_en,
                    alu_opcode, ir_wr_en, ic_count, ic_dir, imm_gen_instr_wr_en,
                    mdr_rd_en, mux_1_sel, mux_2_sel, mux_3_sel, demux_1_sel, instr_type, reg_wr_en, 
                    mar_wr_en, reg_rs_1_addr_wr_en,reg_rs_2_addr_wr_en, reg_rd_addr_wr_en);

input wire[31:0] instr_in;
input wire ctrl_clk, ctrl_rst;
input wire carry_in, zero_in;

output reg [3:0] alu_opcode;
output reg ir_wr_en, ic_count, reg_wr_en, ic_dir, alu_opcode_wr_en, mem_wr_en;
//output reg [3:0] instr_type;
output reg mdr_rd_en, mar_wr_en, imm_gen_instr_wr_en;
output wire reg_rs_1_addr_wr_en, reg_rs_2_addr_wr_en, reg_rd_addr_wr_en;

parameter state_1  = 4'd1, state_2  = 4'd2, state_3  = 4'd3, state_4  = 4'd4, state_5  = 4'd5;
parameter state_6  = 4'd6, state_7  = 4'd7, state_8  = 4'd8, state_9  = 4'd9, state_10 = 4'd10;
parameter state_11 = 4'd11, state_12 = 4'd12, state_13 = 4'd13, state_14 = 4'd14, state_15 = 4'd15;
parameter state_16 = 4'd0;

wire [6:0] opcode = instr_in[6:0];
wire [2:0] funct3 = instr_in[14:12];
wire [6:0] funct7 = instr_in[31:25];

parameter R_type = 4'd1, I_type_1 = 4'd2, I_type_2 = 4'd3, I_type_3=4'd4, I_type_4=4'd5;
parameter S_type = 4'd6, B_type = 4'd7, U_type = 4'd8, J_type = 4'd9;

output wire [3:0] instr_type;
assign instr_type = (opcode == 7'b0110011)? R_type :
                          (opcode == 7'b0010011)? I_type_1 :
                          (opcode == 7'b0000011)? I_type_2:
                          (opcode == 7'b1100111 & funct3 == 3'b000)? I_type_3:
                          (opcode == 7'b1110011 & funct3 == 3'b000)? I_type_4:                          
                          (opcode == 7'b0100011)? S_type :
                          (opcode == 7'b1100011)? B_type :
                          (opcode == 7'b1100111 & funct3 != 3'b000)? J_type:
                          (opcode == 7'b0110111 | 7'b0010111)? U_type : 4'b0;

wire rs_1_addr_wr_en, rs_2_addr_wr_en, rd_addr_wr_en;
wire is_r_instr, is_i_instr, is_s_instr, is_b_instr, is_j_instr, is_u_instr;


assign is_r_instr = (instr_type == R_type) ? 1 :0;
assign is_i_instr = (instr_type == I_type_1 | instr_type == I_type_2 | instr_type == I_type_3 | instr_type == I_type_4);
assign is_s_instr = (instr_type == S_type)? 1 : 0;
assign is_b_instr = (instr_type == B_type)? 1 : 0;
assign is_j_instr = (instr_type == J_type)? 1 : 0;
assign is_u_instr = (instr_type == U_type)? 1 : 0;

assign reg_rs_1_addr_wr_en = (is_r_instr | is_i_instr | is_s_instr | is_b_instr);
assign reg_rs_2_addr_wr_en = (is_r_instr | is_s_instr | is_b_instr);
assign reg_rd_addr_wr_en = (is_r_instr | is_i_instr | is_u_instr | is_j_instr);

output wire demux_1_sel, mux_1_sel, mux_2_sel;
output wire [1:0] mux_3_sel;
reg rs_2_out_en, rs_1_out_en, pc_out_en, alu_out_en;

assign mux_1_sel = (rs_2_out_en) ? 0 : 1; // to toggle mux1
assign mux_2_sel = (rs_1_out_en) ? 0 : 1; // to toggle mux2
assign demux_1_sel = (mar_wr_en == 1) ? 0 : 1;
assign mux_3_sel = (alu_out_en == 1) ? 2'b00 : (mdr_rd_en == 1) ? 2'b01 : (pc_out_en == 1) ? 2'b10 : 11;



reg [3:0] state, next_state; // this can cover 16 state which i think will be sufficient
//reg[4:0] reg_rd_addr_1, reg_rd_addr_2, reg_wr_addr;
//reg carry_in, zero_in, reg_wr_en;

always @(posedge ctrl_clk) begin
    if(ctrl_rst) state <= state_1; else state <= next_state;
end
always @(state, opcode, instr_in) begin            // always block not under clock so blocking statements used
    ic_count = 0; ir_wr_en = 0; reg_wr_en = 0;
    //instr_type = 0;
    mdr_rd_en = 0; ic_dir = 0;
    alu_opcode = 0; mar_wr_en = 0; alu_opcode_wr_en=0; imm_gen_instr_wr_en=0;
    mem_wr_en = 0;
    rs_2_out_en = 1; rs_1_out_en = 1; alu_out_en = 0; pc_out_en = 0;

    //reg_rs_1_addr_wr_en = rs_1_addr_wr_en; reg_rs_2_addr_wr_en = rs_2_addr_wr_en; reg_rd_addr_wr_en = rd_addr_wr_en;
    case(state)
        state_1: next_state<=state_2;
        state_2: begin  //fetch instruction
            next_state = state_3; 
            ir_wr_en = 1;
            ic_count = 1;
        end
        state_3: begin  //decode
            case({opcode, funct3, funct7})  
                {7'b0110011, 3'b000, 7'h00}: begin alu_opcode = 4'b0001; next_state=state_4; alu_opcode_wr_en=1; end //add operation
                {7'b0110011, 3'b000, 7'h20}: begin alu_opcode = 4'b0010; next_state=state_4; alu_opcode_wr_en=1; end //sub operation
                {7'b0110011, 3'b100, 7'h00}: begin alu_opcode = 4'b0011; next_state=state_4; alu_opcode_wr_en=1; end //xor
                {7'b0110011, 3'b110, 7'h00}: begin alu_opcode = 4'b0100; next_state=state_4; alu_opcode_wr_en=1; end //or
                {7'b0110011, 3'b111, 7'h00}: begin alu_opcode = 4'b0101; next_state=state_4; alu_opcode_wr_en=1; end //and
                {7'b0110011, 3'b001, 7'h00}: begin alu_opcode = 4'b0110; next_state=state_4; alu_opcode_wr_en=1; end //sll
                {7'b0110011, 3'b101, 7'h00}: begin alu_opcode = 4'b0111; next_state=state_4; alu_opcode_wr_en=1; end //srl
                {7'b0110011, 3'b101, 7'h20}: begin alu_opcode = 4'b1000; next_state=state_4; alu_opcode_wr_en=1; end //sra
                {7'b0110011, 3'b010, 7'h00}: begin alu_opcode = 4'b1001; next_state=state_4; alu_opcode_wr_en=1; end //slt
                {7'b0110011, 3'b011, 7'h00}: begin alu_opcode = 4'b1010; next_state=state_4; alu_opcode_wr_en=1; end //stlu

                {7'b0000011, 3'b010, 7'h00}: begin //load word(lw)
                    //instr_type = instruction_type;  //instr_type is input to imm_gen
                    imm_gen_instr_wr_en = 1;
                    next_state = state_5;
                end
                {7'b0100011, 3'b010, 7'h00}: begin //store word(lw)
                    //instr_type = instruction_type;  //instr_type is input to imm_gen
                    imm_gen_instr_wr_en = 1;
                    next_state = state_7;
                    rs_2_out_en = 1;
                    rs_1_out_en = 0;
                    alu_out_en = 1;
                    alu_opcode = 4'b0001;
                    alu_opcode_wr_en = 1;
                    mar_wr_en = 1;
                    
                end
                default: next_state = state_8;
                    
            endcase

        end
        state_4: begin  //execution of R instructions
            rs_2_out_en = 1;
            rs_1_out_en = 1;
            alu_out_en = 1;
            reg_wr_en = 1;
            ic_count = 1;
            next_state = state_2;
        end
        state_5: begin  //execution of load instruction(i_type_1)
            rs_2_out_en = 1;
            rs_1_out_en = 0;
            alu_out_en = 1;
            alu_opcode = 4'b0001;
            alu_opcode_wr_en = 1;
            mar_wr_en = 1;
            next_state = state_6; //address provided to memory. time to read memory
        end
        state_6: begin //final step of load_instruction execution
            mdr_rd_en = 1;
            reg_wr_en = 1;
            next_state = state_2;
        end
        state_7: begin
            mem_wr_en = 1;
            next_state = state_2;
        end
        state_8: next_state = state_8;//halt
            

    endcase
end

endmodule