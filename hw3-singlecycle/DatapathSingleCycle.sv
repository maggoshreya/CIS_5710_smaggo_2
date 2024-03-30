`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`include "../hw2a/divider_unsigned.sv"
`include "../hw2b/cla.sv"

module RegFile (
 input logic [4:0] rd,
 input logic [`REG_SIZE] rd_data,
 input logic [4:0] rs1,
 output logic [`REG_SIZE] rs1_data,
 input logic [4:0] rs2,
 output logic [`REG_SIZE] rs2_data,

 input logic clk,
 input logic we,
 input logic rst
);
 localparam int NumRegs = 32;
 logic [`REG_SIZE] regs[NumRegs];

 logic [31:0] reg_outs[NumRegs]; // 2D wire array
 assign regs[0] = 32'd0; // x0 is always zero
 assign regs = reg_outs;
 /* Port assignment */
 assign rs1_data = reg_outs[rs1]; // 1st read port
 assign rs2_data = reg_outs[rs2]; // 2nd read port
 // assign rs3_data = reg_outs[3];
 // assign rs4_data = reg_outs[4];
 // assign rs5_data = reg_outs[5];
 // assign rs6_data = reg_outs[6];
 // assign rs7_data = reg_outs[7];
 // assign rs8_data = reg_outs[8];
 // assign rs9_data = reg_outs[9];
 // assign rs10_data = reg_outs[10];
 // assign rs11_data = reg_outs[11];
 // assign rs12_data = reg_outs[12];
 // assign rs13_data = reg_outs[13];
 // assign rs14_data = reg_outs[14];
 // assign rs15_data = reg_outs[15];
 // assign rs16_data = reg_outs[16];
 // assign rs17_data = reg_outs[17];
 // assign rs18_data = reg_outs[18];
 // assign rs19_data = reg_outs[19];
 // assign rs20_data = reg_outs[20];
 // assign rs21_data = reg_outs[21];
 // assign rs22_data = reg_outs[22];
 // assign rs23_data = reg_outs[23];
 // assign rs24_data = reg_outs[24];
 // assign rs25_data = reg_outs[25];
 // assign rs26_data = reg_outs[26];
 // assign rs27_data = reg_outs[27];
 // assign rs28_data = reg_outs[28];
 // assign rs29_data = reg_outs[29];
 // assign rs30_data = reg_outs[30];
 // assign rs31_data = reg_outs[31];

 always_ff @(posedge clk) begin

 // $display("we_value: %d | rs1_value: %d | rs1_data: %d | rs2_value: %d | rs2_data: %d | rd_value: %d ", we, rs1, rs1_data, rs2, rs2_data, rd_data);
 if (rst) begin
 /* Clear existing data/contents within registers */
 reg_outs[1] <= 32'd0;
 reg_outs[2] <= 32'd0;
 reg_outs[3] <= 32'd0;
 reg_outs[4] <= 32'd0;
 reg_outs[5] <= 32'd0;
 reg_outs[6] <= 32'd0;
 reg_outs[7] <= 32'd0;
 reg_outs[8] <= 32'd0;
 reg_outs[9] <= 32'd0;
 reg_outs[10] <= 32'd0;
 reg_outs[11] <= 32'd0;
 reg_outs[12] <= 32'd0;
 reg_outs[13] <= 32'd0;
 reg_outs[14] <= 32'd0;
 reg_outs[15] <= 32'd0;
 reg_outs[16] <= 32'd0;
 reg_outs[17] <= 32'd0;
 reg_outs[18] <= 32'd0;
 reg_outs[19] <= 32'd0;
 reg_outs[20] <= 32'd0;
 reg_outs[21] <= 32'd0;
 reg_outs[22] <= 32'd0;
 reg_outs[23] <= 32'd0;
 reg_outs[24] <= 32'd0;
 reg_outs[25] <= 32'd0;
 reg_outs[26] <= 32'd0;
 reg_outs[27] <= 32'd0;
 reg_outs[28] <= 32'd0;
 reg_outs[29] <= 32'd0;
 reg_outs[30] <= 32'd0;
 reg_outs[31] <= 32'd0;
 end else begin
 if (we && rd != 0) begin
 reg_outs[rd] <= rd_data;
 end
 end
 end 

endmodule

module DatapathSingleCycle (
 input wire clk,
 input wire rst,
 output logic halt,
 output logic [`REG_SIZE] pc_to_imem,
 input wire [`REG_SIZE] insn_from_imem,
 // addr_to_dmem is a read-write port
 output wire [`REG_SIZE] addr_to_dmem,
 input logic [`REG_SIZE] load_data_from_dmem,
 output wire [`REG_SIZE] store_data_to_dmem,
 output wire [3:0] store_we_to_dmem
);

/************************************************************************
* Regfile instance
************************************************************************/
RegFile rf(
 .rd(insn_from_imem[11:7]),
 .rd_data(regfile_rd_data),
 .rs1(insn_rs1),
 .rs1_data(regfile_rs1_data),
 .rs2(insn_rs2),
 .rs2_data(regfile_rs2_data),
 .clk(clk),
 .we(regfile_we_flag),
 .rst(rst)
);

/************************************************************************
* Regfile components
************************************************************************/
 // components of the instruction
 wire [6:0] insn_funct7;
 wire [4:0] insn_rs2;
 wire [4:0] insn_rs1;
 logic [2:0] insn_funct3;
 wire [4:0] insn_rd;
 logic [31:0] regfile_rd_data;
 wire [31:0] regfile_rs1_data;
 wire [31:0] regfile_rs2_data;
 wire [31:0] regfile_rs1_data_nc;
 wire [31:0] regfile_rs2_data_nc;
 logic regfile_we_flag; /* Write enable flag */
 wire [19:0] regfile_immediate_val;

/************************************************************************
* CLA Instance 0 (instruction arguments)
************************************************************************/
cla cla_inst_0(
 .a(cla_input_a),
 .b(cla_input_b),
 .cin(cla_cin),
 .sum(cla_sum_out)
);

/************************************************************************
* Regfile components
************************************************************************/
logic[31:0] cla_input_a;
logic[31:0] cla_input_b;
logic cla_cin;
logic [31:0] cla_sum_out;

/************************************************************************
* CLA Instance 0 (instruction arguments)
************************************************************************/
cla pc_cla_inst(
 .a(pc_cla_input_a),
 .b(pc_cla_input_b),
 .cin(pc_cla_cin),
 .sum(pc_cla_sum_out)
);

/************************************************************************
* Regfile components
************************************************************************/
logic[31:0] pc_cla_input_a;
logic[31:0] pc_cla_input_b;
logic pc_cla_cin;
logic [31:0] pc_cla_sum_out;

 wire [`OPCODE_SIZE] insn_opcode;

 // split R-type instruction - see section 2.2 of RiscV spec
 assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;

 // setup for I, S, B & J type instructions
 // I - short immediates and loads
 wire [11:0] imm_i;
 assign imm_i = insn_from_imem[31:20];
 wire [ 4:0] imm_shamt = insn_from_imem[24:20];

 // S - stores
 wire [11:0] imm_s;
 assign imm_s[11:5] = insn_funct7, imm_s[4:0] = insn_rd;

 // B - conditionals
 wire [12:0] imm_b;
 assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

 // J - unconditional jumps
 wire [20:0] imm_j;
 assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};

 wire [`REG_SIZE] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
 wire [`REG_SIZE] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
 wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
 wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

 // opcodes - see section 19 of RiscV spec
 localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
 localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
 localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
 localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
 localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
 localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

 localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
 localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
 localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

 localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
 localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

 wire insn_lui = insn_opcode == OpLui;
 wire insn_auipc = insn_opcode == OpAuipc;
 wire insn_jal = insn_opcode == OpJal;
 wire insn_jalr = insn_opcode == OpJalr;

 wire insn_beq = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b000;
 wire insn_bne = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b001;
 wire insn_blt = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b100;
 wire insn_bge = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b101;
 wire insn_bltu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b110;
 wire insn_bgeu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b111;

 wire insn_lb = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b000;
 wire insn_lh = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b001;
 wire insn_lw = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b010;
 wire insn_lbu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b100;
 wire insn_lhu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b101;

 wire insn_sb = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b000;
 wire insn_sh = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b001;
 wire insn_sw = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b010;

 wire insn_addi = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b000;
 wire insn_slti = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b010;
 wire insn_sltiu = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b011;
 wire insn_xori = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b100;
 wire insn_ori = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b110;
 wire insn_andi = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b111;

 wire insn_slli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
 wire insn_srli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
 wire insn_srai = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;

 wire insn_add = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'd0;
 wire insn_sub = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'b0100000;
 wire insn_sll = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
 wire insn_slt = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b010 && insn_from_imem[31:25] == 7'd0;
 wire insn_sltu = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b011 && insn_from_imem[31:25] == 7'd0;
 wire insn_xor = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b100 && insn_from_imem[31:25] == 7'd0;
 wire insn_srl = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
 wire insn_sra = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;
 wire insn_or = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b110 && insn_from_imem[31:25] == 7'd0;
 wire insn_and = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b111 && insn_from_imem[31:25] == 7'd0;

 wire insn_mul = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b000;
 wire insn_mulh = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b001;
 wire insn_mulhsu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b010;
 wire insn_mulhu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b011;
 wire insn_div = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b100;
 wire insn_divu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b101;
 wire insn_rem = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b110;
 wire insn_remu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b111;

 wire insn_ecall = insn_opcode == OpEnviron && insn_from_imem[31:7] == 25'd0;
 wire insn_fence = insn_opcode == OpMiscMem;

 // synthesis translate_off
 // this code is only for simulation, not synthesis
 `include "RvDisassembler.sv"
 string disasm_string;
 always_comb begin
 disasm_string = rv_disasm(insn_from_imem);
 end
 // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic...
 wire [(8*32)-1:0] disasm_wire;
 genvar i;
 for (i = 0; i < 32; i = i + 1) begin : gen_disasm
 assign disasm_wire[(((i+1))*8)-1:((i)*8)] = disasm_string[31-i];
 end
 // synthesis translate_on

 // program counter
 logic [`REG_SIZE] pcNext, pcCurrent;
 always @(posedge clk) begin
 if (rst) begin
 pcCurrent <= 32'd0;
 end else begin
 pcCurrent <= pcNext;
 end
 end
 assign pc_to_imem = pcCurrent;

 // cycle/insn_from_imem counters
 logic [`REG_SIZE] cycles_current, num_insns_current;
 always @(posedge clk) begin
 if (rst) begin
 cycles_current <= 0;
 num_insns_current <= 0;
 end else begin
 cycles_current <= cycles_current + 1;
 if (!rst) begin
 num_insns_current <= num_insns_current + 1;
 end
 end
 end

 logic illegal_insn;

 logic halt_signal;
 
 always_comb begin

 //default values for signals
 halt_signal = 1'b0;

 /* Ecall/halt handle */
 // if (rst == 1) begin
 // halt = 0;
 // end

 // $display("insn_opcode: %d | imm_b: %b | halt_state:", insn_opcode, imm_b);
 // $display("immediate value: 0x%x", {{20{imm_i[11]}}, imm_i});
 // $display("func3: %x", insn_funct3[2:0]);
 illegal_insn = 1'b0;
 case (insn_opcode)
 OpLui: begin
 /* TODO: start here by implementing lui */
 regfile_rd_data = {insn_from_imem[31:12], 12'b0};
 regfile_we_flag = 1; 
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 // pcNext = pcCurrent + 4;
 end
 

 OpRegImm: begin
 case ({4'b0, insn_funct3[2:0]})
 000: begin /* addi */
 // $display("insn_rs1: %d | insn_rs2: %d", insn_rs1, insn_rs2);
 // regfile_rd_data = regfile_rs1_data + {{20{imm_i[11]}}, imm_i};
 cla_input_a = regfile_rs1_data; 
 cla_input_b = {{20{imm_i[11]}}, imm_i};
 regfile_rd_data = cla_sum_out;
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 010: begin /* slti */
 // regfile_rd_data = {{20{imm_i[11]}}, imm_i};
 if(regfile_rs1_data < {{20{imm_i[11]}}, imm_i}) begin
 regfile_rd_data = 1;
 end else begin
 regfile_rd_data = 0;
 end
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 011: begin /* sltiu */
 if(regfile_rs1_data < {20'b0, imm_i}) begin
 regfile_rd_data = 1;
 end else begin
 regfile_rd_data = 0;
 end
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end


 100: begin /* xori */
 regfile_rd_data = regfile_rs1_data ^ {{20{imm_i[11]}}, imm_i};
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 regfile_we_flag = 1;
 end

 110: begin /* ori */
 regfile_rd_data = regfile_rs1_data | {{20{imm_i[11]}}, imm_i};
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 regfile_we_flag = 1;
 end

 111: begin /* andi */
 regfile_rd_data = regfile_rs1_data & {{20{imm_i[11]}}, imm_i};
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 regfile_we_flag = 1;
 end

 001: begin /* slli */
 regfile_rd_data = regfile_rs1_data << imm_i[4:0];
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out; 
 regfile_we_flag = 1;
 end

 101: begin /* srli (0x0) and srai (0x20) */
 if(insn_funct7 == 0) begin
 regfile_rd_data = regfile_rs1_data >> imm_i[4:0];
 end else begin
 regfile_rd_data = regfile_rs1_data >>> imm_i[4:0];
 end
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 regfile_we_flag = 1;

 end
 endcase
 end

 OpBranch: begin
 case ({4'b0, insn_funct3[2:0]}) 
 000: begin /* beq */
 if(regfile_rs1_data == regfile_rs2_data) begin
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = {{19{imm_b[12]}}, imm_b};
 pcNext = pc_cla_sum_out;
 // pcNext += {{19{imm_b[12]}}, imm_b};
 end
 end

 001: begin /* bne */
 if(regfile_rs1_data != regfile_rs2_data) begin
 // pcNext += {{19{imm_b[12]}}, imm_b};
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = {{19{imm_b[12]}}, imm_b};
 pcNext = pc_cla_sum_out;
 end
 end

 100: begin /* blt */
 if(regfile_rs1_data < regfile_rs2_data) begin
 // pcNext += {{19{imm_b[12]}}, imm_b};
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = {{19{imm_b[12]}}, imm_b};
 pcNext = pc_cla_sum_out;
 end
 end

 101: begin /* bge */
 if(regfile_rs1_data >= regfile_rs2_data) begin
 // pcNext += {{19{imm_b[12]}}, imm_b};
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = {{19{imm_b[12]}}, imm_b};
 pcNext = pc_cla_sum_out;
 end
 end

 110: begin /* bltu */
 if(regfile_rs1_data < regfile_rs2_data) begin
 // pcNext += {{19{imm_b[12]}}, imm_b};
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = {{19{imm_b[12]}}, imm_b};
 pcNext = pc_cla_sum_out;
 end
 end

 111: begin /* bgeu */
 if(regfile_rs1_data >= regfile_rs2_data) begin
 // pcNext += {{19{imm_b[12]}}, imm_b};
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = {{19{imm_b[12]}}, imm_b};
 pcNext = pc_cla_sum_out;
 end
 end
 endcase
 end

 OpRegReg: begin
 case ({4'b0, insn_funct3[2:0]}) //unsure what this should be 
 000: begin /* add */
 // $display("Entered add");

 if(insn_funct7 == 0) begin //add
 cla_input_a = regfile_rs1_data;
 cla_input_b = regfile_rs2_data;
 // regfile_rd_data = regfile_rs1_data + regfile_rs2_data;
 regfile_rd_data = cla_sum_out;
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 end else begin //sub
 cla_input_a = regfile_rs1_data;
 cla_input_b = (~regfile_rs2_data + 32'b1);
 // regfile_rd_data = regfile_rs1_data + (~regfile_rs2_data + 32'b1);
 regfile_rd_data = cla_sum_out;
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 end

 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 001: begin // sll
 // $display("Entered sll");
 regfile_rd_data = regfile_rs1_data<<regfile_rs2_data[4:0];
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 010: begin // slt
 // $display("Entered slt");
 if(regfile_rs1_data < regfile_rs2_data) begin
 regfile_rd_data = 1;
 end else begin
 regfile_rd_data = 0;
 end
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 011: begin //sltu
 // $display("sltu");
 if(regfile_rs1_data < regfile_rs2_data) begin
 regfile_rd_data = 1;
 end else begin
 regfile_rd_data = 0;
 end
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 100: begin //xor
 // $display("Entered xor");
 regfile_rd_data = regfile_rs1_data ^ regfile_rs2_data;
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 101: begin //srl
 // $display("srl");
 if(insn_funct7 == 0) begin 
 regfile_rd_data = regfile_rs1_data >> insn_rs2[4:0];
 regfile_we_flag = 1;
 end else begin //sra
 regfile_rd_data = regfile_rs1_data >>> insn_rs2[4:0];
 regfile_we_flag = 1;
 $display("Executing sra");
 end


 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 110: begin //or
 regfile_rd_data = regfile_rs1_data ^ regfile_rs2_data;
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end

 111: begin //and 
 regfile_rd_data = regfile_rs1_data & regfile_rs2_data;
 regfile_we_flag = 1;
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 end
 endcase
 end 

 /* Ecall handle */
 OpEnviron: begin 
 halt_signal = 1'b1; 
 // pcNext = pcCurrent + 4;
 pc_cla_input_a = pcCurrent;
 pc_cla_input_b = 32'd4;
 pcNext = pc_cla_sum_out;
 regfile_we_flag = 1;
 end

 default: begin
 illegal_insn = 1'b1;
 regfile_we_flag = 0;
 end
 endcase
end

assign halt = halt_signal;

endmodule

/* A memory module that supports 1-cycle reads and writes, with one read-only port
 * and one read+write port.
 */
module MemorySingleCycle #(
 parameter int NUM_WORDS = 512
) (
 // rst for both imem and dmem
 input wire rst,

 // clock for both imem and dmem. See RiscvProcessor for clock details.
 input wire clock_mem,

 // must always be aligned to a 4B boundary
 input wire [`REG_SIZE] pc_to_imem,

 // must always be aligned to a 4B boundary
 input wire [`REG_SIZE] addr_to_dmem,

 // the value at memory location pc_to_imem
 output logic [`REG_SIZE] insn_from_imem,

 // the value at memory location addr_to_dmem
 output logic [`REG_SIZE] load_data_from_dmem,

 // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
 input wire [`REG_SIZE] store_data_to_dmem,

 // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
 // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
 input wire [3:0] store_we_to_dmem
);

 // memory is arranged as an array of 4B words
 logic [`REG_SIZE] mem[NUM_WORDS];

 initial begin
 $readmemh("mem_initial_contents.hex", mem, 0);
 end

 always_comb begin
 // memory addresses should always be 4B-aligned
 assert (pc_to_imem[1:0] == 2'b00);
 assert (addr_to_dmem[1:0] == 2'b00);
 end

 localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
 localparam int AddrLsb = 2;

 always @(posedge clock_mem) begin
 if (rst) begin
 end else begin
 insn_from_imem <= mem[{pc_to_imem[AddrMsb:AddrLsb]}];
 end
 end

 always @(negedge clock_mem) begin
 if (rst) begin
 end else begin
 if (store_we_to_dmem[0]) begin
 mem[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
 end
 if (store_we_to_dmem[1]) begin
 mem[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
 end
 if (store_we_to_dmem[2]) begin
 mem[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
 end
 if (store_we_to_dmem[3]) begin
 mem[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
 end
 // dmem is "read-first": read returns value before the write
 load_data_from_dmem <= mem[{addr_to_dmem[AddrMsb:AddrLsb]}];
 end
 end
endmodule

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

 ____
 proc: | |______
 ____
 mem: ___| |___
*/
module RiscvProcessor (
 input wire clock_proc,
 input wire clock_mem,
 input wire rst,
 output logic halt
);

 wire [`REG_SIZE] pc_to_imem, insn_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
 wire [3:0] mem_data_we;

 MemorySingleCycle #(
 .NUM_WORDS(8192)
 ) mem (
 .rst (rst),
 .clock_mem (clock_mem),
 // imem is read-only
 .pc_to_imem(pc_to_imem),
 .insn_from_imem(insn_from_imem),
 // dmem is read-write
 .addr_to_dmem(mem_data_addr),
 .load_data_from_dmem(mem_data_loaded_value),
 .store_data_to_dmem (mem_data_to_write),
 .store_we_to_dmem (mem_data_we)
 );

 DatapathSingleCycle datapath (
 .clk(clock_proc),
 .rst(rst),
 .pc_to_imem(pc_to_imem),
 .insn_from_imem(insn_from_imem),
 .addr_to_dmem(mem_data_addr),
 .store_data_to_dmem(mem_data_to_write),
 .store_we_to_dmem(mem_data_we),
 .load_data_from_dmem(mem_data_loaded_value),
 .halt(halt)
 );

endmodule