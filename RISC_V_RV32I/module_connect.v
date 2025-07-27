`include "ALU.v"
`include "memory_data_register.v"
`include "data_memory.v"
`include "register_bank.v"
`include "memory_address_register.v"
`include "instruction_register.v"
`include "instruction_memory.v"
`include "instruction_counter.v"
`include "control_unit.v"
`include "imm_gen.v"
`include "mux_1.v"
`include "mux_2.v"
`include "mux_3.v"
`include "demux_1.v"
`include "reg_bank_address_register.v"
`include "branch_comparator.v"

module module_connect(
    main_clk,
    main_rst
);
    input wire main_clk;
    input wire main_rst;

    // ? dp- data paths , ctrl- control lines
    // only output names of modules are used to name wires connecting any ports. if output of A is connected to input of B, the wire common to
    // both will be named after output of A

    //register bank
    wire [31:0] dp_reg_out_1, dp_reg_out_2;

    //alu
    wire [31:0] dp_alu_out;

    //memory
    wire [31:0] dp_memory_out, dp_mdr_out;
    wire [31:0] dp_mar_out;

    // register bank address outs
    wire [4:0] dp_reg_bank_rs_1_addr;
    wire [4:0] dp_reg_bank_rs_2_addr;
    wire [4:0] dp_reg_bank_rd_addr;

    //instruction counter and registers
    wire [31:0] dp_instr_mem_out, dp_ic_out, dp_ir_out;

    //mux and demux
    wire [31:0] dp_mux_1_out, dp_mux_2_out, dp_mux_3_out;
    wire [31:0] dp_demux_1_out_0, dp_demux_1_out_1;

    //imm_gen
    wire [31:0] dp_imm_gen_out;

    // branch comparator

    wire ctrl_mem_wr_en, ctrl_reg_wr_en;
    wire ctrl_mem_rd_en; // actually it is mdr_rd_en but giving mem_rd_en makes it more intuitive in terms of memory reading
    wire ctrl_mar_wr_en, ctrl_ir_wr_en, ctrl_ic_dir, ctrl_ic_cnt, ctrl_instr_mem_wr_en;
    wire ctrl_reg_rs_1_addr_wr_en, ctrl_reg_rs_2_addr_wr_en, ctrl_reg_rd_addr_wr_en;
    wire ctrl_mux_1_sel, ctrl_mux_2_sel, ctrl_demux_1_sel, ctrl_bc_en, ctrl_bc_out;
    wire [1:0] ctrl_mux_3_sel;
    wire [3:0] ctrl_instr_type, dp_alu_opcode;

    alu alu_inst(
        .alu_in_2(dp_mux_2_out),
        .alu_in_1(dp_mux_1_out),
        .alu_out(dp_alu_out),
        .alu_opcode(dp_alu_opcode),
        .alu_carry(ctrl_carry_out),
        .alu_zero(ctrl_zero_out)
    );

    data_memory data_mem_inst(
        .data_mem_in(dp_reg_out_2),
        .data_mem_out(dp_memory_out),
        .data_mem_addr(dp_mar_out),
        .data_mem_clk(main_clk),
        .data_mem_wr_en(ctrl_mem_wr_en)
    );

    register_bank reg_bank_inst(
        .reg_rd_addr_1(dp_reg_bank_rs_1_addr),
        .reg_rd_addr_2(dp_reg_bank_rs_2_addr),
        .reg_wr_addr(dp_reg_bank_rd_addr),
        .reg_wr_en(ctrl_reg_wr_en),
        .reg_in(dp_mux_3_out),
        .reg_out_1(dp_reg_out_1),
        .reg_out_2(dp_reg_out_2),
        .reg_clk(main_clk),
        .reg_rst(main_rst)
    );

    memory_data_register mdr_inst(
        .mdr_in(dp_memory_out),
        .mdr_out(dp_mdr_out),
        .mdr_rd(ctrl_mem_rd_en),
        .mdr_rst(main_rst),
        .mdr_clk(main_clk)
    );

    memory_address_register mar_inst(
        .mar_in(dp_demux_1_out_0),
        .mar_out(dp_mar_out),
        .mar_wr_en(ctrl_mar_wr_en),
        .mar_clk(main_clk),
        .mar_rst(main_rst)
    );

    instruction_counter ic_inst(
        .ic_out(dp_ic_out),
        .ic_in(dp_alu_out[4:0]),
        .ic_clk(main_clk),
        .ic_rst(main_rst),
        .ic_dir(ctrl_ic_dir),
        .ic_cnt(ctrl_ic_cnt),
        .ic_wr_en(ctrl_ic_wr_en)
    );

    instruction_memory instr_mem_inst(
        .instr_mem_out(dp_instr_mem_out),
        .instr_mem_addr(dp_ic_out),
        .instr_mem_clk(main_clk)
    );

    instruction_register instr_reg_inst(
        .ir_in(dp_instr_mem_out),
        .ir_out(dp_ir_out),
        .ir_wr_en(ctrl_ir_wr_en),
        .ir_rst(main_rst),
        .ir_clk(main_clk)
    );

    control_unit control_unit_inst(
        .ctrl_clk(main_clk), 
        .instr_in(dp_ir_out),
        .ctrl_rst(main_rst),
        .carry_in(ctrl_carry_out), 
        .zero_in(ctrl_zero_out),
        .alu_opcode(dp_alu_opcode), 
        .ir_wr_en(ctrl_ir_wr_en),
        .ic_count(ctrl_ic_cnt),
        .mdr_rd_en(ctrl_mem_rd_en),
        .mux_1_sel(ctrl_mux_1_sel), 
        .mux_2_sel(ctrl_mux_2_sel),
        .mux_3_sel(ctrl_mux_3_sel),
        .demux_1_sel(ctrl_demux_1_sel),
        .instr_type(ctrl_instr_type),
        .reg_wr_en(ctrl_reg_wr_en),
        .mar_wr_en(ctrl_mar_wr_en),
        .ic_dir(ctrl_ic_dir),
        .ic_wr_en(ctrl_ic_wr_en),
        .bc_en(ctrl_bc_en),
        .bc_in(ctrl_bc_out),
        .imm_gen_instr_wr_en(ctrl_imm_gen_instr_wr_en),
        .reg_rs_1_addr_wr_en(ctrl_reg_rs_1_addr_wr_en),
        .reg_rs_2_addr_wr_en(ctrl_reg_rs_2_addr_wr_en),
        .reg_rd_addr_wr_en(ctrl_reg_rd_addr_wr_en),
        .mem_wr_en(ctrl_mem_wr_en)
    );

    imm_gen imm_gen_inst(
        .imm_gen_in(dp_ir_out),
        .imm_gen_instr_type(ctrl_instr_type),
        .imm_gen_out(dp_imm_gen_out),
        .imm_gen_instr_wr_en(ctrl_imm_gen_instr_wr_en)
    );

    reg_bank_address_register reg_bank_addr_reg_inst(
        .rs_1_in(dp_ir_out[19:15]), 
        .rs_2_in(dp_ir_out[24:20]), 
        .rd_in(dp_ir_out[11:7]),
        .rs_1_wr_en(ctrl_reg_rs_1_addr_wr_en), 
        .rs_2_wr_en(ctrl_reg_rs_2_addr_wr_en), 
        .rd_wr_en(ctrl_reg_rd_addr_wr_en), 
        .reg_clk(main_clk),
        .reg_rst(main_rst),
        .rs_1_out(dp_reg_bank_rs_1_addr), 
        .rs_2_out(dp_reg_bank_rs_2_addr), 
        .rd_out(dp_reg_bank_rd_addr)
    );

    mux_1 mux_1_inst(
        .mux_1_in0(dp_reg_out_1),
        .mux_1_in1(dp_ic_out),
        .mux_1_sel(ctrl_mux_1_sel),
        .mux_1_out(dp_mux_1_out)
    );

    mux_2 mux_2_inst(
        .mux_2_in0(dp_reg_out_2),
        .mux_2_in1(dp_imm_gen_out),
        .mux_2_sel(ctrl_mux_2_sel),
        .mux_2_out(dp_mux_2_out)
    );

    mux_3 mux_3_inst(
        .mux_3_in0(dp_demux_1_out_1),
        .mux_3_in1(dp_mdr_out),
        .mux_3_in2(dp_ic_out),
        .mux_3_sel(ctrl_mux_3_sel),
        .mux_3_out(dp_mux_3_out)
    );

    demux_1 demux_1_inst(
        .demux_in(dp_alu_out),
        .demux_sel(ctrl_demux_1_sel),
        .demux_out0(dp_demux_1_out_0),
        .demux_out1(dp_demux_1_out_1)
    );

    branch_comparator bc_inst(
        .bc_in_1(dp_reg_out_1),
        .bc_in_2(dp_reg_out_2),
        .bc_en(ctrl_bc_en),
        .bc_out(ctrl_bc_out),
        .bc_opcode(dp_ir_out[14:12])
    );

endmodule