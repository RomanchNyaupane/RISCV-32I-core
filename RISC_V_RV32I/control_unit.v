// The control unit is state machine based instead of microcode based


module control_unit(
    input  wire [31:0] instr_in,
    input  wire        ctrl_clk,
    input  wire        ctrl_rst,
    input  wire        carry_in,
    input  wire        zero_in,
    input  wire        bc_in,

    output reg  [3:0]  alu_opcode,
    output reg         ir_wr_en,
    output reg         ic_count,
    output reg         reg_wr_en,
    output reg         ic_dir,
    output reg         mem_wr_en,
    output reg         ic_wr_en,
    output reg         mdr_rd_en,
    output reg         mar_wr_en,
    output reg         imm_gen_instr_wr_en,

    output wire        reg_rs_1_addr_wr_en,
    output wire        reg_rs_2_addr_wr_en,
    output wire        reg_rd_addr_wr_en,
    output wire        bc_en,

    output wire        demux_1_sel,
    output wire        mux_1_sel,
    output wire        mux_2_sel,
    output wire [1:0]  mux_3_sel,
    output wire [3:0]  instr_type
);

parameter state_1  = 4'd1,  state_2  = 4'd2,  state_3  = 4'd3,  state_4  = 4'd4,  state_5  = 4'd5;

wire [6:0] opcode = instr_in[6:0];
wire [2:0] funct3 = instr_in[14:12];
wire [6:0] funct7 = instr_in[31:25];


parameter R_type    = 4'd1,
          I_type_1  = 4'd2,
          I_type_2  = 4'd3,
          I_type_3  = 4'd4,
          I_type_4  = 4'd5,
          S_type    = 4'd6,
          B_type    = 4'd7,
          U_type    = 4'd8,
          J_type    = 4'd9;

// Instruction type decoding
assign instr_type = (opcode == 7'b0110011) ? R_type :
                    (opcode == 7'b0010011) ? I_type_1 :
                    (opcode == 7'b0000011) ? I_type_2 :
                    (opcode == 7'b1100111 && funct3 == 3'b000) ? I_type_3 :
                    (opcode == 7'b1110011 && funct3 == 3'b000) ? I_type_4 :
                    (opcode == 7'b0100011) ? S_type :
                    (opcode == 7'b1100011) ? B_type :
                    (opcode == 7'b1100111 && funct3 != 3'b000) ? J_type :
                    ((opcode == 7'b0110111) || (opcode == 7'b0010111)) ? U_type :
                    4'b0;

//instruction type
wire is_r_instr = (instr_type == R_type);
wire is_i_instr = (instr_type == I_type_1 || instr_type == I_type_2 || instr_type == I_type_3 || instr_type == I_type_4);
wire is_s_instr = (instr_type == S_type);
wire is_b_instr = (instr_type == B_type);
wire is_j_instr = (instr_type == J_type);
wire is_u_instr = (instr_type == U_type);

assign reg_rs_1_addr_wr_en = is_r_instr || is_i_instr || is_s_instr || is_b_instr;
assign reg_rs_2_addr_wr_en = is_r_instr || is_s_instr || is_b_instr;
assign reg_rd_addr_wr_en   = is_r_instr || is_i_instr || is_u_instr || is_j_instr;

assign bc_en = is_b_instr;

reg rs_2_out_en, rs_1_out_en, pc_out_en, alu_out_en;

assign mux_1_sel   = rs_1_out_en ? 1'b0 : 1'b1;
assign mux_2_sel   = rs_2_out_en ? 1'b0 : 1'b1;
assign demux_1_sel = mar_wr_en   ? 1'b0 : 1'b1;
assign mux_3_sel   = alu_out_en  ? 2'b00 :
                     mdr_rd_en   ? 2'b01 :
                     pc_out_en   ? 2'b10 : 2'b11;

//ALU opcode decoding
always @(*) begin
    alu_opcode = 4'b0000; //->default
    case ({opcode, funct3, funct7})
        {7'b0110011, 3'b000, 7'h20}: alu_opcode = 4'b0010; //sub
        {7'b0110011, 3'b000, 7'h00}: alu_opcode = 4'b0001; //add
        {7'b0110011, 3'b100, 7'h00}: alu_opcode = 4'b0011; //xor
        {7'b0110011, 3'b110, 7'h00}: alu_opcode = 4'b0100; //or
        {7'b0110011, 3'b111, 7'h00}: alu_opcode = 4'b0101; //and
        {7'b0110011, 3'b001, 7'h00}: alu_opcode = 4'b0110; //sll
        {7'b0110011, 3'b101, 7'h00}: alu_opcode = 4'b0111; //srl
        {7'b0110011, 3'b101, 7'h20}: alu_opcode = 4'b1000; //sra
        {7'b0110011, 3'b010, 7'h00}: alu_opcode = 4'b1001; //slt
        {7'b0110011, 3'b011, 7'h00}: alu_opcode = 4'b1010; //sltu
    endcase
    case ({opcode, funct3}) // I instructions 
        {7'b0010011, 3'b000}: alu_opcode = 4'b0001; //addi
        {7'b0100011, 3'b010}: alu_opcode = 4'b0001; //add in case of store
        {7'b0000011, 3'b010}: alu_opcode = 4'b0001; //add in case of load 
        {7'b1100011, 3'b001}: alu_opcode = 4'b0001; //add in case of branch
        {7'b0010011, 3'b100}: alu_opcode = 4'b0011; //xori
        {7'b0010011, 3'b110}: alu_opcode = 4'b0100; //ori
        {7'b0010011, 3'b111}: alu_opcode = 4'b0101; //andi
        {7'b0010011, 3'b001}: alu_opcode = 4'b0110; //slli
        {7'b0010011, 3'b101}: begin
            case (funct7)
                7'h00: alu_opcode = 4'b0111; //srli
                7'h20: alu_opcode = 4'b1000; //srai
            endcase
        end
        {7'b0010011, 3'b010}: alu_opcode = 4'b1001; //slti
        {7'b0010011, 3'b011}: alu_opcode = 4'b1010; //sltiu
    endcase
end


reg [3:0] state, next_state;

always @(posedge ctrl_clk) begin
    if (ctrl_rst)
        state <= state_1;
    else
        state <= next_state;
end

always @(state, opcode, instr_in) begin
    ic_count = 0;
    ir_wr_en = 0;
    reg_wr_en = 0;
    mem_wr_en = 0;
    mar_wr_en = 0;
    rs_2_out_en = 0;
    rs_1_out_en = 0;
    alu_out_en = 0;
    pc_out_en = 0;
    mdr_rd_en = 0;
    imm_gen_instr_wr_en = 0;
    ic_dir = 0;
    ic_wr_en = 0;

    case (state)
        state_1: next_state <= state_2; 

        state_2: begin 
            ir_wr_en = 1;
            case (instr_type)
                R_type: begin
                    rs_2_out_en = 1;
                    rs_1_out_en = 1;
                    alu_out_en = 1;
                    reg_wr_en = 1;
                    temp_var = 1;
                    ic_count = 1;
                    next_state = state_1;
                end
                I_type_1: begin
                    rs_2_out_en = 0;
                    rs_1_out_en = 1;
                    alu_out_en = 1;
                    reg_wr_en = 1;
                    ic_count = 1;
                    next_state = state_1;
                end
                I_type_2: begin // load word
                    imm_gen_instr_wr_en = 1;
                    rs_2_out_en = 0;
                    rs_1_out_en = 1;
                    alu_out_en = 1;
                    ic_count = 1;
                    mar_wr_en = 1;
                    next_state = state_3;
                end
                S_type: begin //store word
                    imm_gen_instr_wr_en = 1;
                    next_state = state_4;
                    rs_2_out_en = 0;
                    rs_1_out_en = 1;
                    alu_out_en = 1;
                    ic_count = 1;
                    mar_wr_en = 1;
                end
                B_type: begin
                    imm_gen_instr_wr_en = 1;
                    rs_1_out_en = 0;
                    rs_2_out_en = 0;
                    ic_wr_en = 1;
                    ic_count = 0;
                    next_state = state_1;
                end
                default: next_state = state_5;
            endcase
        end

        state_3: begin // final step of load instruction execution
            mdr_rd_en = 1;
            reg_wr_en = 1;
            next_state = state_2;
        end

        state_4: begin
            mem_wr_en = 1;
            next_state = state_2;
        end

        state_5: next_state = state_5; // halt
    endcase
end

endmodule