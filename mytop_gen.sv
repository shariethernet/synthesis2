// Generated by SandPiper(TM) 1.9-2018/02/11-beta from Redwood EDA.
// (Installed here: /d/RISC-V/tlverilog/SandPiper_1.9-2018_02_11-beta_distro.)
// Redwood EDA does not claim intellectual property rights to this file and provides no warranty regarding its correctness or quality.


`include "sandpiper_gen.vh"


genvar dmem, imem, xreg;


//
// Signals declared top-level.
//

// For $br_tgt_pc.
logic [31:0] L0_br_tgt_pc_a0;

// For $dec_bits.
logic [10:0] L0_dec_bits_a0;

// For $dmem1_addr.
logic [$clog2(32)-1:0] L0_dmem1_addr_a0;

// For $dmem1_rd_en.
logic L0_dmem1_rd_en_a0;

// For $dmem1_wr_data.
logic [32-1:0] L0_dmem1_wr_data_a0;

// For $dmem1_wr_en.
logic L0_dmem1_wr_en_a0;

// For $funct3.
logic [2:0] L0_funct3_a0;

// For $funct3_valid.
logic L0_funct3_valid_a0;

// For $funct7.
logic [6:0] L0_funct7_a0;

// For $funct7_valid.
logic L0_funct7_valid_a0;

// For $imm.
logic [31:0] L0_imm_a0;

// For $imm_valid.
logic L0_imm_valid_a0;

// For $instr.
logic [31:0] L0_instr_a0;

// For $is_add.
logic L0_is_add_a0;

// For $is_addi.
logic L0_is_addi_a0;

// For $is_and.
logic L0_is_and_a0;

// For $is_andi.
logic L0_is_andi_a0;

// For $is_auipc.
logic L0_is_auipc_a0;

// For $is_b_instr.
logic L0_is_b_instr_a0;

// For $is_beq.
logic L0_is_beq_a0;

// For $is_bge.
logic L0_is_bge_a0;

// For $is_bgeu.
logic L0_is_bgeu_a0;

// For $is_blt.
logic L0_is_blt_a0;

// For $is_bltu.
logic L0_is_bltu_a0;

// For $is_bne.
logic L0_is_bne_a0;

// For $is_i_instr.
logic L0_is_i_instr_a0;

// For $is_j_instr.
logic L0_is_j_instr_a0;

// For $is_jal.
logic L0_is_jal_a0;

// For $is_jalr.
logic L0_is_jalr_a0;

// For $is_load.
logic L0_is_load_a0;

// For $is_lui.
logic L0_is_lui_a0;

// For $is_or.
logic L0_is_or_a0;

// For $is_ori.
logic L0_is_ori_a0;

// For $is_r_instr.
logic L0_is_r_instr_a0;

// For $is_s_instr.
logic L0_is_s_instr_a0;

// For $is_sll.
logic L0_is_sll_a0;

// For $is_slli.
logic L0_is_slli_a0;

// For $is_slt.
logic L0_is_slt_a0;

// For $is_slti.
logic L0_is_slti_a0;

// For $is_sltiu.
logic L0_is_sltiu_a0;

// For $is_sltu.
logic L0_is_sltu_a0;

// For $is_sra.
logic L0_is_sra_a0;

// For $is_srai.
logic L0_is_srai_a0;

// For $is_srl.
logic L0_is_srl_a0;

// For $is_srli.
logic L0_is_srli_a0;

// For $is_sub.
logic L0_is_sub_a0;

// For $is_u_instr.
logic L0_is_u_instr_a0;

// For $is_xor.
logic L0_is_xor_a0;

// For $is_xori.
logic L0_is_xori_a0;

// For $jalr_tgt_pc.
logic [31:0] L0_jalr_tgt_pc_a0;

// For $ld_data.
logic [31:0] L0_ld_data_a0;

// For $next_pc.
logic [31:0] L0_next_pc_a0,
             L0_next_pc_a1;

// For $opcode.
logic [6:0] L0_opcode_a0;

// For $passed_cond.
logic L0_passed_cond_a0,
      L0_passed_cond_a1,
      L0_passed_cond_a2;

// For $pc.
logic [31:0] L0_pc_a0;

// For $rd.
logic [4:0] L0_rd_a0;

// For $rd_valid.
logic L0_rd_valid_a0;

// For $reset.
logic L0_reset_a0;

// For $result.
logic [31:0] L0_result_a0;

// For $rf1_rd_en1.
logic L0_rf1_rd_en1_a0;

// For $rf1_rd_en2.
logic L0_rf1_rd_en2_a0;

// For $rf1_rd_index1.
logic [$clog2(32)-1:0] L0_rf1_rd_index1_a0;

// For $rf1_rd_index2.
logic [$clog2(32)-1:0] L0_rf1_rd_index2_a0;

// For $rf1_wr_data.
logic [32-1:0] L0_rf1_wr_data_a0;

// For $rf1_wr_en.
logic L0_rf1_wr_en_a0;

// For $rf1_wr_index.
logic [$clog2(32)-1:0] L0_rf1_wr_index_a0;

// For $rs1.
logic [4:0] L0_rs1_a0;

// For $rs1_valid.
logic L0_rs1_valid_a0;

// For $rs2.
logic [4:0] L0_rs2_a0;

// For $rs2_valid.
logic L0_rs2_valid_a0;

// For $sext_rslt.
logic [63:0] L0_sext_rslt_a0;

// For $sltiu_rslt.
logic [31:0] L0_sltiu_rslt_a0;

// For $sltu_rslt.
logic [31:0] L0_sltu_rslt_a0;

// For $sra_rslt.
logic [63:0] L0_sra_rslt_a0;

// For $srai_rslt.
logic [63:0] L0_srai_rslt_a0;

// For $src1_value.
logic [31:0] L0_src1_value_a0;

// For $src2_value.
logic [31:0] L0_src2_value_a0;

// For $taken_br.
logic L0_taken_br_a0;

// For /dmem$value.
logic [32-1:0] Dmem_value_a0 [31:0],
               Dmem_value_a1 [31:0];

// For /xreg$value.
logic [32-1:0] Xreg_value_a0 [31:0],
               Xreg_value_a1 [31:0];



generate

   // For $next_pc.
   always_ff @(posedge clk) L0_next_pc_a1[31:0] <= L0_next_pc_a0[31:0];

   // For $passed_cond.
   always_ff @(posedge clk) L0_passed_cond_a1 <= L0_passed_cond_a0;
   always_ff @(posedge clk) L0_passed_cond_a2 <= L0_passed_cond_a1;


   //
   // Scope: /dmem[31:0]
   //
   for (dmem = 0; dmem <= 31; dmem++) begin : L1gen_Dmem
      // For $value.
      always_ff @(posedge clk) Dmem_value_a1[dmem][32-1:0] <= Dmem_value_a0[dmem][32-1:0];

   end

   //
   // Scope: /xreg[31:0]
   //
   for (xreg = 0; xreg <= 31; xreg++) begin : L1gen_Xreg
      // For $value.
      always_ff @(posedge clk) Xreg_value_a1[xreg][32-1:0] <= Xreg_value_a0[xreg][32-1:0];

   end


endgenerate




generate   // This is awkward, but we need to go into 'generate' context in the line that `includes the declarations file.