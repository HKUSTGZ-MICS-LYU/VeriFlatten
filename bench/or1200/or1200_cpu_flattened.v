module or1200_cpu(
    clk,
    rst,
    ic_en,
    icpu_adr_o,
    icpu_cycstb_o,
    icpu_sel_o,
    icpu_tag_o,
    icpu_dat_i,
    icpu_ack_i,
    icpu_rty_i,
    icpu_err_i,
    icpu_adr_i,
    icpu_tag_i,
    immu_en,
    immu_sxe,
    immu_uxe,
    id_void,
    id_insn,
    ex_void,
    ex_insn,
    ex_freeze,
    wb_insn,
    wb_freeze,
    id_pc,
    ex_pc,
    wb_pc,
    branch_op,
    spr_dat_npc,
    rf_dataw,
    ex_flushpipe,
    du_stall,
    du_addr,
    du_dat_du,
    du_read,
    du_write,
    du_except_stop,
    du_except_trig,
    du_dsr,
    du_dmr1,
    du_hwbkpt,
    du_hwbkpt_ls_r,
    du_dat_cpu,
    du_lsu_store_dat,
    du_lsu_load_dat,
    abort_mvspr,
    abort_ex,
    dc_en,
    dcpu_adr_o,
    dcpu_cycstb_o,
    dcpu_we_o,
    dcpu_sel_o,
    dcpu_tag_o,
    dcpu_dat_o,
    dcpu_dat_i,
    dcpu_ack_i,
    dcpu_rty_i,
    dcpu_err_i,
    dcpu_tag_i,
    sb_en,
    dmmu_en,
    dc_no_writethrough,
    boot_adr_sel_i,
    sig_int,
    sig_tick,
    supv,
    spr_addr,
    spr_dat_cpu,
    spr_dat_pic,
    spr_dat_tt,
    spr_dat_pm,
    spr_dat_dmmu,
    spr_dat_immu,
    spr_dat_du,
    spr_cs,
    spr_we,
    mtspr_dc_done
);
    logic  ___sel_temp_0;
    logic [4:0] ___sel_temp_1;
    logic [15:0] ___sel_temp_2;
    logic [31:0] ___sel_temp_3;
    logic [3:0] ___sel_temp_4;
    logic [4:0] ___sel_temp_5;
    logic [31:0] ___sel_temp_6;
    logic [32:0] ___sel_temp_7;
    logic [4:0] ___sel_temp_8;
    logic [15:0] ___sel_temp_9;
    logic [7:0] ___sel_temp_10;
    logic [2:0] ___sel_temp_11;
    logic [4:0] ___sel_temp_12;
    logic  ___sel_temp_13;
    logic [1:0] ___sel_temp_14;
    logic [1:0] ___sel_temp_15;
    logic  ___sel_temp_16;
    logic [7:0] ___sel_temp_17;
    logic [15:0] ___sel_temp_18;

    input logic clk;
    input logic rst;
    output logic ic_en;
    output logic [31:0] icpu_adr_o;
    output logic icpu_cycstb_o;
    output logic [3:0] icpu_sel_o;
    output logic [3:0] icpu_tag_o;
    input logic [31:0] icpu_dat_i;
    input logic icpu_ack_i;
    input logic icpu_rty_i;
    input logic icpu_err_i;
    input logic [31:0] icpu_adr_i;
    input logic [3:0] icpu_tag_i;
    output logic immu_en;
    input logic immu_sxe;
    input logic immu_uxe;
    output logic id_void;
    output logic [31:0] id_insn;
    output wire logic ex_void;
    output logic [31:0] ex_insn;
    output wire logic ex_freeze;
    output wire logic [31:0] wb_insn;
    output wire logic wb_freeze;
    output logic [31:0] id_pc;
    output logic [31:0] ex_pc;
    output logic [31:0] wb_pc;
    output logic [2:0] branch_op;
    output logic [31:0] spr_dat_npc;
    output logic [31:0] rf_dataw;
    output wire logic ex_flushpipe;
    input logic du_stall;
    input logic [31:0] du_addr;
    input logic [31:0] du_dat_du;
    input logic du_read;
    input logic du_write;
    output logic [13:0] du_except_stop;
    output logic [13:0] du_except_trig;
    input logic [13:0] du_dsr;
    input logic [24:0] du_dmr1;
    input logic du_hwbkpt;
    input logic du_hwbkpt_ls_r;
    output logic [31:0] du_dat_cpu;
    output logic [31:0] du_lsu_store_dat;
    output logic [31:0] du_lsu_load_dat;
    output wire logic abort_mvspr;
    output wire logic abort_ex;
    output logic dc_en;
    output logic [31:0] dcpu_adr_o;
    output logic dcpu_cycstb_o;
    output logic dcpu_we_o;
    output logic [3:0] dcpu_sel_o;
    output logic [3:0] dcpu_tag_o;
    output logic [31:0] dcpu_dat_o;
    input logic [31:0] dcpu_dat_i;
    input logic dcpu_ack_i;
    input logic dcpu_rty_i;
    input logic dcpu_err_i;
    input logic [3:0] dcpu_tag_i;
    output logic sb_en;
    output logic dmmu_en;
    output logic dc_no_writethrough;
    input logic boot_adr_sel_i;
    input logic sig_int;
    input logic sig_tick;
    output logic supv;
    output logic [31:0] spr_addr;
    output logic [31:0] spr_dat_cpu;
    input logic [31:0] spr_dat_pic;
    input logic [31:0] spr_dat_tt;
    input logic [31:0] spr_dat_pm;
    input logic [31:0] spr_dat_dmmu;
    input logic [31:0] spr_dat_immu;
    input logic [31:0] spr_dat_du;
    output logic [31:0] spr_cs;
    output logic spr_we;
    input logic mtspr_dc_done;
    logic or1200_cpu___clk;
    logic or1200_cpu___rst;
    logic or1200_cpu___ic_en;
    logic [31:0] or1200_cpu___icpu_adr_o;
    logic or1200_cpu___icpu_cycstb_o;
    logic [3:0] or1200_cpu___icpu_sel_o;
    logic [3:0] or1200_cpu___icpu_tag_o;
    logic [31:0] or1200_cpu___icpu_dat_i;
    logic or1200_cpu___icpu_ack_i;
    logic or1200_cpu___icpu_rty_i;
    logic or1200_cpu___icpu_err_i;
    logic [31:0] or1200_cpu___icpu_adr_i;
    logic [3:0] or1200_cpu___icpu_tag_i;
    logic or1200_cpu___immu_en;
    logic or1200_cpu___immu_sxe;
    logic or1200_cpu___immu_uxe;
    logic or1200_cpu___id_void;
    logic [31:0] or1200_cpu___id_insn;

    logic [31:0] or1200_cpu___ex_insn;



    logic [31:0] or1200_cpu___id_pc;
    logic [31:0] or1200_cpu___ex_pc;
    logic [31:0] or1200_cpu___wb_pc;




    logic or1200_cpu___du_stall;
    logic [31:0] or1200_cpu___du_addr;
    logic [31:0] or1200_cpu___du_dat_du;
    logic or1200_cpu___du_read;
    logic or1200_cpu___du_write;
    logic [13:0] or1200_cpu___du_except_stop;
    logic [13:0] or1200_cpu___du_except_trig;
    logic [13:0] or1200_cpu___du_dsr;
    logic [24:0] or1200_cpu___du_dmr1;
    logic or1200_cpu___du_hwbkpt;
    logic or1200_cpu___du_hwbkpt_ls_r;
    logic [31:0] or1200_cpu___du_dat_cpu;
    logic [31:0] or1200_cpu___du_lsu_store_dat;
    logic [31:0] or1200_cpu___du_lsu_load_dat;


    logic or1200_cpu___dc_en;
    logic [31:0] or1200_cpu___dcpu_adr_o;
    logic or1200_cpu___dcpu_cycstb_o;
    logic or1200_cpu___dcpu_we_o;
    logic [3:0] or1200_cpu___dcpu_sel_o;
    logic [3:0] or1200_cpu___dcpu_tag_o;
    logic [31:0] or1200_cpu___dcpu_dat_o;
    logic [31:0] or1200_cpu___dcpu_dat_i;
    logic or1200_cpu___dcpu_ack_i;
    logic or1200_cpu___dcpu_rty_i;
    logic or1200_cpu___dcpu_err_i;
    logic [3:0] or1200_cpu___dcpu_tag_i;
    logic or1200_cpu___sb_en;
    logic or1200_cpu___dmmu_en;
    logic or1200_cpu___dc_no_writethrough;
    logic or1200_cpu___boot_adr_sel_i;
    logic or1200_cpu___sig_int;
    logic or1200_cpu___sig_tick;
    logic or1200_cpu___supv;
    logic [31:0] or1200_cpu___spr_addr;
    logic [31:0] or1200_cpu___spr_dat_cpu;
    logic [31:0] or1200_cpu___spr_dat_pic;
    logic [31:0] or1200_cpu___spr_dat_tt;
    logic [31:0] or1200_cpu___spr_dat_pm;
    logic [31:0] or1200_cpu___spr_dat_dmmu;
    logic [31:0] or1200_cpu___spr_dat_immu;
    logic [31:0] or1200_cpu___spr_dat_du;
    logic [31:0] or1200_cpu___spr_cs;
    logic or1200_cpu___spr_we;
    logic or1200_cpu___mtspr_dc_done;



    logic [31:0] or1200_cpu___if_insn;
    logic or1200_cpu___saving_if_insn;
    logic [31:0] or1200_cpu___if_pc;
    logic [4:0] or1200_cpu___rf_addrw;
    logic [4:0] or1200_cpu___rf_addra;
    logic [4:0] or1200_cpu___rf_addrb;
    logic or1200_cpu___rf_rda;
    logic or1200_cpu___rf_rdb;
    logic [31:0] or1200_cpu___id_simm;
    logic [31:2] or1200_cpu___id_branch_addrtarget;
    logic [31:2] or1200_cpu___ex_branch_addrtarget;
    logic [4:0] or1200_cpu___alu_op;
    logic [3:0] or1200_cpu___alu_op2;
    logic [3:0] or1200_cpu___comp_op;
    logic [2:0] or1200_cpu___pre_branch_op;
    logic [3:0] or1200_cpu___id_lsu_op;
    logic or1200_cpu___genpc_freeze;
    logic or1200_cpu___if_freeze;
    logic or1200_cpu___id_freeze;
    logic [1:0] or1200_cpu___sel_a;
    logic [1:0] or1200_cpu___sel_b;
    logic [3:0] or1200_cpu___rfwb_op;
    logic [7:0] or1200_cpu___fpu_op;
    assign or1200_cpu___fpu_op = 8'h0;
    
    logic [31:0] or1200_cpu___rf_dataa;
    logic [31:0] or1200_cpu___rf_datab;
    logic [31:0] or1200_cpu___muxed_a;
    logic [31:0] or1200_cpu___muxed_b;
    logic [31:0] or1200_cpu___wb_forw;
    logic or1200_cpu___wbforw_valid;
    logic [31:0] or1200_cpu___operand_a;
    logic [31:0] or1200_cpu___operand_b;
    logic [31:0] or1200_cpu___alu_dataout;
    logic [31:0] or1200_cpu___lsu_dataout;
    logic [31:0] or1200_cpu___sprs_dataout;
    logic [31:0] or1200_cpu___fpu_dataout;
    assign or1200_cpu___fpu_dataout = 32'sh0;
    
    logic or1200_cpu___fpu_done;
    assign or1200_cpu___fpu_done = 1'h1;
    
    logic [31:0] or1200_cpu___ex_simm;
    logic [2:0] or1200_cpu___multicycle;
    logic [1:0] or1200_cpu___wait_on;
    logic [3:0] or1200_cpu___except_type;
    logic [4:0] or1200_cpu___cust5_op;
    logic [5:0] or1200_cpu___cust5_limm;
    logic or1200_cpu___if_flushpipe;
    logic or1200_cpu___id_flushpipe;
    logic or1200_cpu___wb_flushpipe;
    logic or1200_cpu___extend_flush;
    logic or1200_cpu___ex_branch_taken;
    logic or1200_cpu___flag;
    logic or1200_cpu___flagforw;
    logic or1200_cpu___flag_we;
    logic or1200_cpu___flagforw_alu;
    logic or1200_cpu___flag_we_alu;
    logic or1200_cpu___flagforw_fpu;
    assign or1200_cpu___flagforw_fpu = 1'h0;
    
    logic or1200_cpu___flag_we_fpu;
    assign or1200_cpu___flag_we_fpu = 1'h0;
    
    logic or1200_cpu___carry;
    logic or1200_cpu___cyforw;
    logic or1200_cpu___cy_we_alu;
    logic or1200_cpu___ovforw;
    logic or1200_cpu___ov_we_alu;
    logic or1200_cpu___ovforw_mult_mac;
    logic or1200_cpu___ov_we_mult_mac;
    logic or1200_cpu___cy_we_rf;
    logic or1200_cpu___lsu_stall;
    logic or1200_cpu___epcr_we;
    logic or1200_cpu___eear_we;
    logic or1200_cpu___esr_we;
    logic or1200_cpu___sp_epcr_ghost_we;
    logic or1200_cpu___sp_eear_ghost_we;
    logic or1200_cpu___sp_esr_ghost_we;
    logic or1200_cpu___pc_we;
    logic [31:0] or1200_cpu___epcr;
    logic [31:0] or1200_cpu___eear;
    logic [16:0] or1200_cpu___esr;
    logic [31:0] or1200_cpu___sp_epcr_ghost;
    logic [31:0] or1200_cpu___sp_eear_ghost;
    logic [16:0] or1200_cpu___sp_esr_ghost;
    logic [11:0] or1200_cpu___fpcsr;
    assign or1200_cpu___fpcsr = 12'h0;
    
    logic or1200_cpu___fpcsr_we;
    logic or1200_cpu___sr_we;
    logic [16:0] or1200_cpu___to_sr;
    logic [16:0] or1200_cpu___sr;
    logic or1200_cpu___except_flushpipe;
    logic or1200_cpu___except_start;
    logic or1200_cpu___except_started;
    logic or1200_cpu___fpu_except_started;
    logic or1200_cpu___sig_syscall;
    logic or1200_cpu___sig_trap;
    logic or1200_cpu___sig_range;
    logic or1200_cpu___sig_fp;
    assign or1200_cpu___sig_fp = 1'h0;
    
    logic [31:0] or1200_cpu___spr_dat_cfgr;
    logic [31:0] or1200_cpu___spr_dat_rf;
    logic [31:0] or1200_cpu___spr_dat_ppc;
    logic [31:0] or1200_cpu___spr_dat_mac;
    logic [31:0] or1200_cpu___spr_dat_fpu;
    assign or1200_cpu___spr_dat_fpu = 32'sh0;
    
    logic or1200_cpu___mtspr_done;
    logic or1200_cpu___force_dslot_fetch;
    assign or1200_cpu___force_dslot_fetch = 1'h0;
    
    logic or1200_cpu___no_more_dslot;
    logic or1200_cpu___ex_spr_read;
    logic or1200_cpu___ex_spr_write;
    logic or1200_cpu___if_stall;
    logic or1200_cpu___id_macrc_op;
    logic or1200_cpu___ex_macrc_op;
    logic [2:0] or1200_cpu___id_mac_op;
    logic [2:0] or1200_cpu___mac_op;
    logic [31:0] or1200_cpu___mult_mac_result;
    logic or1200_cpu___mult_mac_stall;
    logic [13:0] or1200_cpu___except_trig;
    logic [13:0] or1200_cpu___except_stop;
    logic or1200_cpu___genpc_refetch;
    logic or1200_cpu___rfe;
    logic or1200_cpu___lsu_unstall;
    logic or1200_cpu___except_align;
    logic or1200_cpu___except_dtlbmiss;
    logic or1200_cpu___except_dmmufault;
    logic or1200_cpu___except_illegal;
    logic or1200_cpu___except_itlbmiss;
    logic or1200_cpu___except_immufault;
    logic or1200_cpu___except_ibuserr;
    logic or1200_cpu___except_dbuserr;
    logic or1200_cpu___gpr_written_to;
    logic [4:0] or1200_cpu___gpr_written_addr;

    logic or1200_cpu___sp_insn_is_exthz;
    logic [5:0] or1200_cpu___sp_return_counter;
    logic [31:0] or1200_cpu___sp_address;
    logic [31:0] or1200_cpu___sp_data;
    logic [31:0] or1200_cpu___sp_strobe;
    logic [31:0] or1200_cpu___sp_assertions_violated;
    logic or1200_cpu___sp_assertion_violated;
    reg [31:0] or1200_cpu___sp_assertions_violated_reg;
    reg or1200_cpu___sp_assertion_violated_reg;
    reg or1200_cpu___sp_exception_hold;
    logic or1200_cpu___sp_exceptionHandled;
    logic or1200_cpu___sp_exceptionGated;
    logic [31:0] or1200_cpu___sp_attack_enable;
    reg [31:0] or1200_cpu___prev_epcr;
    reg [31:0] or1200_cpu___prev_eear;
    reg or1200_cpu___prev_sr0;
    reg [16:0] or1200_cpu___prev_esr;
    reg [31:0] or1200_cpu___prev_if_insn;
    reg or1200_cpu___prev_id_freeze;
    reg [31:0] or1200_cpu___prev_ex_insn;
    logic or1200_cpu_____Vcellinp__or1200_genpc__except_prefix;
    logic or1200_cpu___attack_do;
    logic [31:0] or1200_cpu___attack_dataw;
    logic [4:0] or1200_cpu___attack_addrw;
    logic or1200_cpu___attack_we;
    logic or1200_cpu_____Vcellinp__or1200_rf__sp_exception_comb;
    logic [31:0] or1200_cpu_____Vcellout__or1200_rf__gpr_written_data;
    logic or1200_cpu_____Vcellinp__or1200_rf__spr_cs;
    logic or1200_cpu_____Vcellinp__or1200_rf__supv;
    logic or1200_cpu_____Vcellinp__or1200_fpu__spr_cs;
    logic or1200_cpu_____Vcellinp__or1200_mult_mac__spr_cs;
    logic or1200_cpu_____Vcellinp__or1200_sprs__except_illegal;
    logic or1200_cpu_____Vcellinp__or1200_sprs__ov_we;
    logic or1200_cpu_____Vcellinp__or1200_sprs__ovforw;
    logic [15:0] or1200_cpu_____Vcellinp__or1200_sprs__addrofs;
    logic or1200_cpu_____Vcellinp__or1200_except__fpcsr_fpee;
    logic or1200_cpu_____Vcellinp__or1200_except__sig_illegal;
    logic [31:0] or1200_cpu___sp_assertions_violated_b;
    logic or1200_cpu___insn_clk;
    logic or1200_cpu___gpr_written_data;
    logic or1200_cpu___or1200_genpc___clk;
    logic or1200_cpu___or1200_genpc___rst;
    logic [31:0] or1200_cpu___or1200_genpc___icpu_adr_o;
    logic or1200_cpu___or1200_genpc___icpu_cycstb_o;
    logic [3:0] or1200_cpu___or1200_genpc___icpu_sel_o;
    logic [3:0] or1200_cpu___or1200_genpc___icpu_tag_o;
    logic or1200_cpu___or1200_genpc___icpu_rty_i;
    logic [31:0] or1200_cpu___or1200_genpc___icpu_adr_i;
    logic [2:0] or1200_cpu___or1200_genpc___pre_branch_op;
    logic [2:0] or1200_cpu___or1200_genpc___branch_op;
    logic [3:0] or1200_cpu___or1200_genpc___except_type;
    logic or1200_cpu___or1200_genpc___except_prefix;
    logic [31:2] or1200_cpu___or1200_genpc___id_branch_addrtarget;
    logic [31:2] or1200_cpu___or1200_genpc___ex_branch_addrtarget;
    logic [31:0] or1200_cpu___or1200_genpc___muxed_b;
    logic [31:0] or1200_cpu___or1200_genpc___operand_b;
    logic or1200_cpu___or1200_genpc___flag;
    logic or1200_cpu___or1200_genpc___flagforw;

    logic or1200_cpu___or1200_genpc___except_start;
    logic [31:0] or1200_cpu___or1200_genpc___epcr;
    logic [31:0] or1200_cpu___or1200_genpc___spr_dat_i;
    logic or1200_cpu___or1200_genpc___spr_pc_we;
    logic or1200_cpu___or1200_genpc___genpc_refetch;
    logic or1200_cpu___or1200_genpc___genpc_freeze;
    logic or1200_cpu___or1200_genpc___no_more_dslot;
    logic or1200_cpu___or1200_genpc___lsu_stall;
    logic [31:0] or1200_cpu___or1200_genpc___ex_pc;
    logic [31:0] or1200_cpu___or1200_genpc___sp_epcr_ghost;
    logic [31:0] or1200_cpu___or1200_genpc___sp_attack_enable;

    reg [31:2] or1200_cpu___or1200_genpc___pcreg_default;
    reg or1200_cpu___or1200_genpc___pcreg_select;
    reg [31:2] or1200_cpu___or1200_genpc___pcreg;
    reg [31:0] or1200_cpu___or1200_genpc___pc;
    reg or1200_cpu___or1200_genpc___genpc_refetch_r;
    reg or1200_cpu___or1200_genpc___wait_lsu;
    logic [31:0] or1200_cpu___or1200_genpc___pcreg_boot;
    assign or1200_cpu___or1200_genpc___pcreg_boot = 32'h100;
    
    logic or1200_cpu___or1200_if___clk;
    logic or1200_cpu___or1200_if___rst;
    logic [31:0] or1200_cpu___or1200_if___icpu_dat_i;
    logic or1200_cpu___or1200_if___icpu_ack_i;
    logic or1200_cpu___or1200_if___icpu_err_i;
    logic [31:0] or1200_cpu___or1200_if___icpu_adr_i;
    logic [3:0] or1200_cpu___or1200_if___icpu_tag_i;
    logic or1200_cpu___or1200_if___if_freeze;
    logic [31:0] or1200_cpu___or1200_if___if_insn;
    logic [31:0] or1200_cpu___or1200_if___if_pc;
    logic or1200_cpu___or1200_if___if_flushpipe;
    logic or1200_cpu___or1200_if___saving_if_insn;
    logic or1200_cpu___or1200_if___if_stall;
    logic or1200_cpu___or1200_if___no_more_dslot;
    logic or1200_cpu___or1200_if___genpc_refetch;
    logic or1200_cpu___or1200_if___rfe;
    logic or1200_cpu___or1200_if___except_itlbmiss;
    logic or1200_cpu___or1200_if___except_immufault;
    logic or1200_cpu___or1200_if___except_ibuserr;
    logic [5:0] or1200_cpu___or1200_if___sp_return_counter;
    logic or1200_cpu___or1200_if___save_insn;
    logic or1200_cpu___or1200_if___if_bypass;
    reg or1200_cpu___or1200_if___if_bypass_reg;
    reg [31:0] or1200_cpu___or1200_if___insn_saved;
    reg [31:0] or1200_cpu___or1200_if___addr_saved;
    reg [2:0] or1200_cpu___or1200_if___err_saved;
    reg or1200_cpu___or1200_if___saved;
    logic or1200_cpu___or1200_ctrl___clk;
    logic or1200_cpu___or1200_ctrl___rst;
    logic or1200_cpu___or1200_ctrl___except_flushpipe;
    logic or1200_cpu___or1200_ctrl___extend_flush;
    logic or1200_cpu___or1200_ctrl___if_flushpipe;
    logic or1200_cpu___or1200_ctrl___id_flushpipe;
    logic or1200_cpu___or1200_ctrl___ex_flushpipe;
    logic or1200_cpu___or1200_ctrl___wb_flushpipe;
    logic or1200_cpu___or1200_ctrl___id_freeze;
    logic or1200_cpu___or1200_ctrl___ex_freeze;
    logic or1200_cpu___or1200_ctrl___wb_freeze;
    logic [31:0] or1200_cpu___or1200_ctrl___if_insn;
    reg [31:0] or1200_cpu___or1200_ctrl___id_insn;
    reg [31:0] or1200_cpu___or1200_ctrl___ex_insn;
    logic or1200_cpu___or1200_ctrl___abort_mvspr;


    logic or1200_cpu___or1200_ctrl___ex_branch_taken;
    logic or1200_cpu___or1200_ctrl___pc_we;
    logic [4:0] or1200_cpu___or1200_ctrl___rf_addra;
    logic [4:0] or1200_cpu___or1200_ctrl___rf_addrb;
    logic or1200_cpu___or1200_ctrl___rf_rda;
    logic or1200_cpu___or1200_ctrl___rf_rdb;






    logic [7:0] or1200_cpu___or1200_ctrl___fpu_op;
    reg [31:0] or1200_cpu___or1200_ctrl___wb_insn;


    logic [31:2] or1200_cpu___or1200_ctrl___id_branch_addrtarget;




    logic [4:0] or1200_cpu___or1200_ctrl___cust5_op;
    logic [5:0] or1200_cpu___or1200_ctrl___cust5_limm;
    logic [31:0] or1200_cpu___or1200_ctrl___id_pc;
    logic [31:0] or1200_cpu___or1200_ctrl___ex_pc;
    logic or1200_cpu___or1200_ctrl___du_hwbkpt;


    logic or1200_cpu___or1200_ctrl___wbforw_valid;


    logic or1200_cpu___or1200_ctrl___force_dslot_fetch;
    logic or1200_cpu___or1200_ctrl___no_more_dslot;


    logic or1200_cpu___or1200_ctrl___ex_spr_read;
    logic or1200_cpu___or1200_ctrl___ex_spr_write;

    logic or1200_cpu___or1200_ctrl___id_macrc_op;

    logic or1200_cpu___or1200_ctrl___rfe;

    logic or1200_cpu___or1200_ctrl___dc_no_writethrough;
    logic or1200_cpu___or1200_ctrl___sp_exception;
    logic [31:0] or1200_cpu___or1200_ctrl___sp_attack_enable;

    logic or1200_cpu___or1200_ctrl___if_maci_op;
    reg [2:0] or1200_cpu___or1200_ctrl___ex_mac_op;
    reg [4:0] or1200_cpu___or1200_ctrl___wb_rfaddrw;
    reg or1200_cpu___or1200_ctrl___sel_imm;
    logic or1200_cpu___or1200_ctrl___wb_void;
    reg or1200_cpu___or1200_ctrl___ex_delayslot_dsi;
    reg or1200_cpu___or1200_ctrl___ex_delayslot_nop;
    reg or1200_cpu___or1200_ctrl___spr_read;
    reg or1200_cpu___or1200_ctrl___spr_write;
    reg or1200_cpu___or1200_ctrl___syscall_prev;
    reg or1200_cpu___or1200_ctrl___syscall_prev_prev;
    logic or1200_cpu___or1200_ctrl___attack;
    logic or1200_cpu___or1200_rf___clk;
    logic or1200_cpu___or1200_rf___rst;
    logic or1200_cpu___or1200_rf___cy_we_i;
    logic or1200_cpu___or1200_rf___cy_we_o;
    logic or1200_cpu___or1200_rf___supv;
    logic or1200_cpu___or1200_rf___wb_freeze;
    logic [4:0] or1200_cpu___or1200_rf___addrw;
    logic [31:0] or1200_cpu___or1200_rf___dataw;
    logic or1200_cpu___or1200_rf___we;
    logic or1200_cpu___or1200_rf___flushpipe;
    logic or1200_cpu___or1200_rf___id_freeze;
    logic [4:0] or1200_cpu___or1200_rf___addra;
    logic [4:0] or1200_cpu___or1200_rf___addrb;
    logic [31:0] or1200_cpu___or1200_rf___dataa;
    logic [31:0] or1200_cpu___or1200_rf___datab;
    logic or1200_cpu___or1200_rf___rda;
    logic or1200_cpu___or1200_rf___rdb;
    logic or1200_cpu___or1200_rf___spr_cs;
    logic or1200_cpu___or1200_rf___spr_write;
    logic [31:0] or1200_cpu___or1200_rf___spr_addr;
    logic [31:0] or1200_cpu___or1200_rf___spr_dat_i;
    logic [31:0] or1200_cpu___or1200_rf___spr_dat_o;
    logic or1200_cpu___or1200_rf___du_read;
    logic [31:0] or1200_cpu___or1200_rf___sp_attack_enable;
    logic or1200_cpu___or1200_rf___gpr_written_to;
    logic [4:0] or1200_cpu___or1200_rf___gpr_written_addr;
    logic [31:0] or1200_cpu___or1200_rf___gpr_written_data;
    logic or1200_cpu___or1200_rf___sp_exception_comb;
    logic [16:0] or1200_cpu___or1200_rf___sr;
    logic [31:0] or1200_cpu___or1200_rf___ex_insn;


    logic [31:0] or1200_cpu___or1200_rf___from_rfa;
    logic [31:0] or1200_cpu___or1200_rf___from_rfb;
    logic [4:0] or1200_cpu___or1200_rf___rf_addra;
    logic [4:0] or1200_cpu___or1200_rf___rf_addrw;
    logic [31:0] or1200_cpu___or1200_rf___rf_dataw;
    logic or1200_cpu___or1200_rf___rf_we;
    logic or1200_cpu___or1200_rf___spr_valid;
    logic or1200_cpu___or1200_rf___rf_ena;
    logic or1200_cpu___or1200_rf___rf_enb;
    reg or1200_cpu___or1200_rf___rf_we_allow;
    reg or1200_cpu___or1200_rf___spr_du_cs;
    logic or1200_cpu___or1200_rf___spr_cs_fe;
    reg [4:0] or1200_cpu___or1200_rf___addra_last;
    logic or1200_cpu___or1200_rf___attack;
    logic or1200_cpu___or1200_rf_____Vcellinp__rf_a__we_b;
    logic or1200_cpu___or1200_rf_____Vcellinp__rf_a__ce_b;
    logic or1200_cpu___or1200_rf_____Vcellinp__rf_b__we_b;
    logic or1200_cpu___or1200_rf_____Vcellinp__rf_b__ce_b;
    logic or1200_cpu___or1200_rf___rf_a___clk_a;
    logic or1200_cpu___or1200_rf___rf_a___ce_a;
    logic [4:0] or1200_cpu___or1200_rf___rf_a___addr_a;
    logic [31:0] or1200_cpu___or1200_rf___rf_a___do_a;
    logic or1200_cpu___or1200_rf___rf_a___clk_b;
    logic or1200_cpu___or1200_rf___rf_a___ce_b;
    logic or1200_cpu___or1200_rf___rf_a___we_b;
    logic [4:0] or1200_cpu___or1200_rf___rf_a___addr_b;
    logic [31:0] or1200_cpu___or1200_rf___rf_a___di_b;


    reg [31:0] or1200_cpu___or1200_rf___rf_a___mem [31:0];
    reg [4:0] or1200_cpu___or1200_rf___rf_a___addr_a_reg;
    reg [31:0] or1200_cpu___or1200_rf___rf_a___k;
    logic or1200_cpu___or1200_rf___rf_b___clk_a;
    logic or1200_cpu___or1200_rf___rf_b___ce_a;
    logic [4:0] or1200_cpu___or1200_rf___rf_b___addr_a;
    logic [31:0] or1200_cpu___or1200_rf___rf_b___do_a;
    logic or1200_cpu___or1200_rf___rf_b___clk_b;
    logic or1200_cpu___or1200_rf___rf_b___ce_b;
    logic or1200_cpu___or1200_rf___rf_b___we_b;
    logic [4:0] or1200_cpu___or1200_rf___rf_b___addr_b;
    logic [31:0] or1200_cpu___or1200_rf___rf_b___di_b;


    reg [31:0] or1200_cpu___or1200_rf___rf_b___mem [31:0];
    reg [4:0] or1200_cpu___or1200_rf___rf_b___addr_a_reg;
    reg [31:0] or1200_cpu___or1200_rf___rf_b___k;
    logic or1200_cpu___or1200_operandmuxes___clk;
    logic or1200_cpu___or1200_operandmuxes___rst;
    logic or1200_cpu___or1200_operandmuxes___id_freeze;
    logic or1200_cpu___or1200_operandmuxes___ex_freeze;
    logic [31:0] or1200_cpu___or1200_operandmuxes___rf_dataa;
    logic [31:0] or1200_cpu___or1200_operandmuxes___rf_datab;
    logic [31:0] or1200_cpu___or1200_operandmuxes___ex_forw;
    logic [31:0] or1200_cpu___or1200_operandmuxes___wb_forw;
    logic [31:0] or1200_cpu___or1200_operandmuxes___simm;
    logic [1:0] or1200_cpu___or1200_operandmuxes___sel_a;
    logic [1:0] or1200_cpu___or1200_operandmuxes___sel_b;





    reg or1200_cpu___or1200_operandmuxes___saved_a;
    reg or1200_cpu___or1200_operandmuxes___saved_b;
    logic [31:0] or1200_cpu___or1200_alu___a;
    logic [31:0] or1200_cpu___or1200_alu___b;
    logic [31:0] or1200_cpu___or1200_alu___mult_mac_result;
    logic or1200_cpu___or1200_alu___macrc_op;
    logic [4:0] or1200_cpu___or1200_alu___alu_op;
    logic [3:0] or1200_cpu___or1200_alu___alu_op2;
    logic [3:0] or1200_cpu___or1200_alu___comp_op;
    logic [4:0] or1200_cpu___or1200_alu___cust5_op;
    logic [5:0] or1200_cpu___or1200_alu___cust5_limm;







    logic or1200_cpu___or1200_alu___carry;
    logic or1200_cpu___or1200_alu___flag;

    reg [31:0] or1200_cpu___or1200_alu___shifted_rotated;
    reg [31:0] or1200_cpu___or1200_alu___extended;
    reg or1200_cpu___or1200_alu___flagcomp;
    logic [31:0] or1200_cpu___or1200_alu___comp_a;
    logic [31:0] or1200_cpu___or1200_alu___comp_b;


    logic [31:0] or1200_cpu___or1200_alu___result_sum;
    logic [31:0] or1200_cpu___or1200_alu___result_and;
    logic or1200_cpu___or1200_alu___cy_sum;
    logic or1200_cpu___or1200_alu___cy_sub;
    logic or1200_cpu___or1200_alu___ov_sum;
    logic [31:0] or1200_cpu___or1200_alu___carry_in;
    logic [31:0] or1200_cpu___or1200_alu___b_mux;
    logic or1200_cpu___or1200_fpu___clk;
    logic or1200_cpu___or1200_fpu___rst;
    logic or1200_cpu___or1200_fpu___ex_freeze;
    logic [31:0] or1200_cpu___or1200_fpu___a;
    logic [31:0] or1200_cpu___or1200_fpu___b;
    logic [7:0] or1200_cpu___or1200_fpu___fpu_op;
    logic [31:0] or1200_cpu___or1200_fpu___result;
    logic or1200_cpu___or1200_fpu___done;
    logic or1200_cpu___or1200_fpu___flagforw;
    logic or1200_cpu___or1200_fpu___flag_we;
    logic or1200_cpu___or1200_fpu___sig_fp;
    logic or1200_cpu___or1200_fpu___except_started;
    logic or1200_cpu___or1200_fpu___fpcsr_we;
    logic [11:0] or1200_cpu___or1200_fpu___fpcsr;
    logic or1200_cpu___or1200_fpu___spr_cs;
    logic or1200_cpu___or1200_fpu___spr_write;
    logic [31:0] or1200_cpu___or1200_fpu___spr_addr;
    logic [31:0] or1200_cpu___or1200_fpu___spr_dat_i;
    logic [31:0] or1200_cpu___or1200_fpu___spr_dat_o;

    logic or1200_cpu___or1200_mult_mac___clk;
    logic or1200_cpu___or1200_mult_mac___rst;
    logic or1200_cpu___or1200_mult_mac___ex_freeze;
    logic or1200_cpu___or1200_mult_mac___id_macrc_op;
    logic or1200_cpu___or1200_mult_mac___macrc_op;
    logic [31:0] or1200_cpu___or1200_mult_mac___a;
    logic [31:0] or1200_cpu___or1200_mult_mac___b;

    logic [4:0] or1200_cpu___or1200_mult_mac___alu_op;

    logic or1200_cpu___or1200_mult_mac___mult_mac_stall;


    logic or1200_cpu___or1200_mult_mac___spr_cs;
    logic or1200_cpu___or1200_mult_mac___spr_write;
    logic [31:0] or1200_cpu___or1200_mult_mac___spr_addr;
    logic [31:0] or1200_cpu___or1200_mult_mac___spr_dat_i;
    logic [31:0] or1200_cpu___or1200_mult_mac___spr_dat_o;

    reg or1200_cpu___or1200_mult_mac___ex_freeze_r;
    logic or1200_cpu___or1200_mult_mac___alu_op_mul;
    logic or1200_cpu___or1200_mult_mac___alu_op_smul;
    reg [63:0] or1200_cpu___or1200_mult_mac___mul_prod_r;
    logic or1200_cpu___or1200_mult_mac___alu_op_umul;
    logic [63:0] or1200_cpu___or1200_mult_mac___mul_prod;
    logic or1200_cpu___or1200_mult_mac___mul_stall;
    reg [1:0] or1200_cpu___or1200_mult_mac___mul_stall_count;
    reg [2:0] or1200_cpu___or1200_mult_mac___mac_op_r1;
    reg [2:0] or1200_cpu___or1200_mult_mac___mac_op_r2;
    reg [2:0] or1200_cpu___or1200_mult_mac___mac_op_r3;
    reg or1200_cpu___or1200_mult_mac___mac_stall_r;
    reg [63:0] or1200_cpu___or1200_mult_mac___mac_r;
    logic [31:0] or1200_cpu___or1200_mult_mac___x;
    logic [31:0] or1200_cpu___or1200_mult_mac___y;
    logic or1200_cpu___or1200_mult_mac___spr_maclo_we;
    logic or1200_cpu___or1200_mult_mac___spr_machi_we;
    logic or1200_cpu___or1200_mult_mac___alu_op_div;
    logic or1200_cpu___or1200_mult_mac___alu_op_udiv;
    logic or1200_cpu___or1200_mult_mac___alu_op_sdiv;
    reg or1200_cpu___or1200_mult_mac___div_free;
    logic or1200_cpu___or1200_mult_mac___div_stall;
    reg [63:0] or1200_cpu___or1200_mult_mac___div_quot_r;
    logic [31:0] or1200_cpu___or1200_mult_mac___div_tmp;
    reg [5:0] or1200_cpu___or1200_mult_mac___div_cntr;
    logic or1200_cpu___or1200_mult_mac___div_by_zero;
    logic [31:0] or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___X;
    logic [31:0] or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___Y;
    logic or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___CLK;
    logic or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___RST;
    logic [63:0] or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___P;
    reg [63:0] or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___p0;
    reg [63:0] or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___p1;
    reg [31:0] or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___xi;
    reg [31:0] or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___yi;
    logic or1200_cpu___or1200_sprs___clk;
    logic or1200_cpu___or1200_sprs___rst;
    logic or1200_cpu___or1200_sprs___flagforw;
    logic or1200_cpu___or1200_sprs___flag_we;
    logic or1200_cpu___or1200_sprs___flag;
    logic or1200_cpu___or1200_sprs___cyforw;
    logic or1200_cpu___or1200_sprs___cy_we;
    logic or1200_cpu___or1200_sprs___carry;
    logic or1200_cpu___or1200_sprs___ovforw;
    logic or1200_cpu___or1200_sprs___ov_we;
    logic [31:0] or1200_cpu___or1200_sprs___addrbase;
    logic [15:0] or1200_cpu___or1200_sprs___addrofs;
    logic [31:0] or1200_cpu___or1200_sprs___dat_i;
    logic [2:0] or1200_cpu___or1200_sprs___branch_op;
    logic or1200_cpu___or1200_sprs___ex_spr_read;

    logic [31:0] or1200_cpu___or1200_sprs___epcr;
    logic [31:0] or1200_cpu___or1200_sprs___eear;
    logic [16:0] or1200_cpu___or1200_sprs___esr;
    logic or1200_cpu___or1200_sprs___except_started;

    logic or1200_cpu___or1200_sprs___epcr_we;
    logic or1200_cpu___or1200_sprs___eear_we;
    logic or1200_cpu___or1200_sprs___esr_we;
    logic or1200_cpu___or1200_sprs___pc_we;
    logic or1200_cpu___or1200_sprs___sr_we;
    logic [16:0] or1200_cpu___or1200_sprs___to_sr;
    reg [16:0] or1200_cpu___or1200_sprs___sr;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_cfgr;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_rf;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_npc;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_ppc;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_mac;
    logic or1200_cpu___or1200_sprs___boot_adr_sel_i;
    logic [11:0] or1200_cpu___or1200_sprs___fpcsr;
    logic or1200_cpu___or1200_sprs___fpcsr_we;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_fpu;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_pic;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_tt;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_pm;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_dmmu;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_immu;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_du;
    logic [31:0] or1200_cpu___or1200_sprs___spr_addr;
    logic [31:0] or1200_cpu___or1200_sprs___spr_dat_o;
    logic [31:0] or1200_cpu___or1200_sprs___spr_cs;
    logic or1200_cpu___or1200_sprs___spr_we;
    logic [31:0] or1200_cpu___or1200_sprs___du_addr;
    logic [31:0] or1200_cpu___or1200_sprs___du_dat_du;
    logic or1200_cpu___or1200_sprs___du_read;
    logic or1200_cpu___or1200_sprs___du_write;
    logic [31:0] or1200_cpu___or1200_sprs___du_dat_cpu;
    logic [31:0] or1200_cpu___or1200_sprs___ex_pc;
    logic or1200_cpu___or1200_sprs___ex_void;
    logic or1200_cpu___or1200_sprs___except_illegal;
    logic or1200_cpu___or1200_sprs___sp_epcr_ghost_we;
    logic or1200_cpu___or1200_sprs___sp_eear_ghost_we;
    logic or1200_cpu___or1200_sprs___sp_esr_ghost_we;
    logic [31:0] or1200_cpu___or1200_sprs___sp_epcr_ghost;
    logic [31:0] or1200_cpu___or1200_sprs___sp_eear_ghost;
    logic [16:0] or1200_cpu___or1200_sprs___sp_esr_ghost;
    logic [31:0] or1200_cpu___or1200_sprs___sp_address;
    logic [31:0] or1200_cpu___or1200_sprs___sp_data;
    logic [31:0] or1200_cpu___or1200_sprs___sp_strobe;
    logic [31:0] or1200_cpu___or1200_sprs___sp_assertions_violated;
    logic or1200_cpu___or1200_sprs___sp_assertion_violated;
    logic [31:0] or1200_cpu___or1200_sprs___sp_attack_enable;
    logic or1200_cpu___or1200_sprs___sp_insn_is_exthz;
    logic or1200_cpu___or1200_sprs___id_freeze;

    reg [16:0] or1200_cpu___or1200_sprs___sr_reg;
    reg or1200_cpu___or1200_sprs___sr_reg_bit_eph;
    reg or1200_cpu___or1200_sprs___sr_reg_bit_eph_select;
    logic or1200_cpu___or1200_sprs___sr_reg_bit_eph_muxed;
    logic or1200_cpu___or1200_sprs___cfgr_sel;
    logic or1200_cpu___or1200_sprs___rf_sel;
    logic or1200_cpu___or1200_sprs___npc_sel;
    logic or1200_cpu___or1200_sprs___ppc_sel;
    logic or1200_cpu___or1200_sprs___sr_sel;
    logic or1200_cpu___or1200_sprs___epcr_sel;
    logic or1200_cpu___or1200_sprs___eear_sel;
    logic or1200_cpu___or1200_sprs___esr_sel;
    logic or1200_cpu___or1200_sprs___fpcsr_sel;
    logic [31:0] or1200_cpu___or1200_sprs___sys_data;
    logic or1200_cpu___or1200_sprs___du_access;
    reg [31:0] or1200_cpu___or1200_sprs___unqualified_cs;
    reg [31:0] or1200_cpu___or1200_sprs___sp_reg0;
    logic or1200_cpu___or1200_sprs___sp_reg0_we;
    logic or1200_cpu___or1200_sprs___sp_reg0_sel;
    reg [31:0] or1200_cpu___or1200_sprs___sp_reg1;
    logic or1200_cpu___or1200_sprs___sp_reg1_we;
    logic or1200_cpu___or1200_sprs___sp_reg1_sel;
    reg [31:0] or1200_cpu___or1200_sprs___sp_reg2;
    logic or1200_cpu___or1200_sprs___sp_reg2_we;
    logic or1200_cpu___or1200_sprs___sp_reg2_sel;
    reg [31:0] or1200_cpu___or1200_sprs___sp_reg3;
    logic or1200_cpu___or1200_sprs___sp_reg3_we;
    logic or1200_cpu___or1200_sprs___sp_reg3_sel;
    reg [31:0] or1200_cpu___or1200_sprs___sp_reg4;
    logic or1200_cpu___or1200_sprs___sp_reg4_we;
    logic or1200_cpu___or1200_sprs___sp_reg4_sel;
    logic or1200_cpu___or1200_sprs___sp_reg5_sel;
    reg [31:0] or1200_cpu___or1200_sprs___sp_reg6;
    logic or1200_cpu___or1200_sprs___sp_reg6_we;
    logic or1200_cpu___or1200_sprs___sp_reg6_sel;
    reg [31:0] or1200_cpu___or1200_sprs___sp_reg7;
    logic or1200_cpu___or1200_sprs___sp_reg7_sel;
    logic or1200_cpu___or1200_sprs___sp_esr_ghost_sel;
    logic or1200_cpu___or1200_sprs___sp_epcr_ghost_sel;

    reg or1200_cpu___or1200_sprs___sp_already_attacked;
    logic or1200_cpu___or1200_sprs___sp_eear_ghost_sel;
    logic or1200_cpu___or1200_lsu___clk;
    logic or1200_cpu___or1200_lsu___rst;
    logic [31:0] or1200_cpu___or1200_lsu___id_addrbase;
    logic [31:0] or1200_cpu___or1200_lsu___ex_addrbase;
    logic [31:0] or1200_cpu___or1200_lsu___id_addrofs;
    logic [31:0] or1200_cpu___or1200_lsu___ex_addrofs;
    logic [3:0] or1200_cpu___or1200_lsu___id_lsu_op;
    logic [31:0] or1200_cpu___or1200_lsu___lsu_datain;
    logic [31:0] or1200_cpu___or1200_lsu___lsu_dataout;
    logic or1200_cpu___or1200_lsu___lsu_stall;
    logic or1200_cpu___or1200_lsu___lsu_unstall;
    logic or1200_cpu___or1200_lsu___du_stall;

    logic or1200_cpu___or1200_lsu___except_dtlbmiss;
    logic or1200_cpu___or1200_lsu___except_dmmufault;
    logic or1200_cpu___or1200_lsu___except_dbuserr;
    logic or1200_cpu___or1200_lsu___id_freeze;
    logic or1200_cpu___or1200_lsu___ex_freeze;
    logic or1200_cpu___or1200_lsu___flushpipe;
    logic [31:0] or1200_cpu___or1200_lsu___dcpu_adr_o;
    logic or1200_cpu___or1200_lsu___dcpu_cycstb_o;
    logic or1200_cpu___or1200_lsu___dcpu_we_o;

    logic [3:0] or1200_cpu___or1200_lsu___dcpu_tag_o;
    logic [31:0] or1200_cpu___or1200_lsu___dcpu_dat_o;
    logic [31:0] or1200_cpu___or1200_lsu___dcpu_dat_i;
    logic or1200_cpu___or1200_lsu___dcpu_ack_i;
    logic or1200_cpu___or1200_lsu___dcpu_rty_i;
    logic or1200_cpu___or1200_lsu___dcpu_err_i;
    logic [3:0] or1200_cpu___or1200_lsu___dcpu_tag_i;


    reg [3:0] or1200_cpu___or1200_lsu___ex_lsu_op;
    logic [2:0] or1200_cpu___or1200_lsu___id_precalc_sum;
    reg [2:0] or1200_cpu___or1200_lsu___dcpu_adr_r;
    logic [1:0] or1200_cpu___or1200_lsu_____Vcellinp__or1200_mem2reg__addr;
    logic [1:0] or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr;
    logic [1:0] or1200_cpu___or1200_lsu___or1200_mem2reg___addr;
    logic [3:0] or1200_cpu___or1200_lsu___or1200_mem2reg___lsu_op;
    logic [31:0] or1200_cpu___or1200_lsu___or1200_mem2reg___memdata;


    reg [31:0] or1200_cpu___or1200_lsu___or1200_mem2reg___aligned;
    logic [1:0] or1200_cpu___or1200_lsu___or1200_reg2mem___addr;
    logic [3:0] or1200_cpu___or1200_lsu___or1200_reg2mem___lsu_op;
    logic [31:0] or1200_cpu___or1200_lsu___or1200_reg2mem___regdata;
    logic [31:0] or1200_cpu___or1200_lsu___or1200_reg2mem___memdata;

    reg [7:0] or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hh;
    reg [7:0] or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hl;
    reg [7:0] or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_lh;
    reg [7:0] or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_ll;
    logic or1200_cpu___or1200_wbmux___clk;
    logic or1200_cpu___or1200_wbmux___rst;
    logic or1200_cpu___or1200_wbmux___wb_freeze;
    logic [3:0] or1200_cpu___or1200_wbmux___rfwb_op;
    logic [31:0] or1200_cpu___or1200_wbmux___muxin_a;
    logic [31:0] or1200_cpu___or1200_wbmux___muxin_b;
    logic [31:0] or1200_cpu___or1200_wbmux___muxin_c;
    logic [31:0] or1200_cpu___or1200_wbmux___muxin_d;
    logic [31:0] or1200_cpu___or1200_wbmux___muxin_e;




    logic or1200_cpu___or1200_freeze___clk;
    logic or1200_cpu___or1200_freeze___rst;
    logic [2:0] or1200_cpu___or1200_freeze___multicycle;
    logic [1:0] or1200_cpu___or1200_freeze___wait_on;
    logic or1200_cpu___or1200_freeze___flushpipe;
    logic or1200_cpu___or1200_freeze___extend_flush;
    logic or1200_cpu___or1200_freeze___lsu_stall;
    logic or1200_cpu___or1200_freeze___if_stall;
    logic or1200_cpu___or1200_freeze___lsu_unstall;
    logic or1200_cpu___or1200_freeze___du_stall;
    logic or1200_cpu___or1200_freeze___mac_stall;
    logic or1200_cpu___or1200_freeze___force_dslot_fetch;
    logic or1200_cpu___or1200_freeze___abort_ex;
    logic or1200_cpu___or1200_freeze___genpc_freeze;
    logic or1200_cpu___or1200_freeze___if_freeze;
    logic or1200_cpu___or1200_freeze___id_freeze;
    logic or1200_cpu___or1200_freeze___ex_freeze;
    logic or1200_cpu___or1200_freeze___wb_freeze;
    logic or1200_cpu___or1200_freeze___saving_if_insn;
    logic or1200_cpu___or1200_freeze___fpu_done;
    logic or1200_cpu___or1200_freeze___mtspr_done;
    logic or1200_cpu___or1200_freeze___icpu_ack_i;
    logic or1200_cpu___or1200_freeze___icpu_err_i;
    logic or1200_cpu___or1200_freeze___multicycle_freeze;
    reg [2:0] or1200_cpu___or1200_freeze___multicycle_cnt;
    reg or1200_cpu___or1200_freeze___flushpipe_r;
    reg [1:0] or1200_cpu___or1200_freeze___waiting_on;
    logic or1200_cpu___or1200_except___clk;
    logic or1200_cpu___or1200_except___rst;
    logic or1200_cpu___or1200_except___sig_ibuserr;
    logic or1200_cpu___or1200_except___sig_dbuserr;
    logic or1200_cpu___or1200_except___sig_illegal;
    logic or1200_cpu___or1200_except___sig_align;
    logic or1200_cpu___or1200_except___sig_range;
    logic or1200_cpu___or1200_except___sig_dtlbmiss;
    logic or1200_cpu___or1200_except___sig_dmmufault;
    logic or1200_cpu___or1200_except___sig_int;
    logic or1200_cpu___or1200_except___sig_syscall;
    logic or1200_cpu___or1200_except___sig_trap;
    logic or1200_cpu___or1200_except___sig_itlbmiss;
    logic or1200_cpu___or1200_except___sig_immufault;
    logic or1200_cpu___or1200_except___sig_tick;
    logic or1200_cpu___or1200_except___ex_branch_taken;
    logic or1200_cpu___or1200_except___genpc_freeze;
    logic or1200_cpu___or1200_except___id_freeze;
    logic or1200_cpu___or1200_except___ex_freeze;
    logic or1200_cpu___or1200_except___wb_freeze;
    logic or1200_cpu___or1200_except___if_stall;
    logic [31:0] or1200_cpu___or1200_except___if_pc;
    reg [31:0] or1200_cpu___or1200_except___id_pc;
    reg [31:0] or1200_cpu___or1200_except___ex_pc;
    reg [31:0] or1200_cpu___or1200_except___wb_pc;
    logic or1200_cpu___or1200_except___id_flushpipe;
    logic or1200_cpu___or1200_except___ex_flushpipe;

    logic or1200_cpu___or1200_except___except_flushpipe;
    reg [3:0] or1200_cpu___or1200_except___except_type;
    logic or1200_cpu___or1200_except___except_start;

    logic [13:0] or1200_cpu___or1200_except___except_stop;
    logic [13:0] or1200_cpu___or1200_except___except_trig;
    logic or1200_cpu___or1200_except___ex_void;
    logic or1200_cpu___or1200_except___abort_mvspr;
    logic [2:0] or1200_cpu___or1200_except___branch_op;


    logic [31:0] or1200_cpu___or1200_except___datain;
    logic [13:0] or1200_cpu___or1200_except___du_dsr;
    logic or1200_cpu___or1200_except___epcr_we;
    logic or1200_cpu___or1200_except___eear_we;
    logic or1200_cpu___or1200_except___esr_we;
    logic or1200_cpu___or1200_except___pc_we;


    logic [24:0] or1200_cpu___or1200_except___du_dmr1;
    logic or1200_cpu___or1200_except___du_hwbkpt;
    logic or1200_cpu___or1200_except___du_hwbkpt_ls_r;

    logic or1200_cpu___or1200_except___sr_we;
    logic [16:0] or1200_cpu___or1200_except___to_sr;
    logic [16:0] or1200_cpu___or1200_except___sr;
    logic [31:0] or1200_cpu___or1200_except___lsu_addr;
    logic or1200_cpu___or1200_except___abort_ex;
    logic or1200_cpu___or1200_except___icpu_ack_i;
    logic or1200_cpu___or1200_except___icpu_err_i;
    logic or1200_cpu___or1200_except___dcpu_ack_i;
    logic or1200_cpu___or1200_except___dcpu_err_i;
    logic or1200_cpu___or1200_except___sig_fp;
    logic or1200_cpu___or1200_except___fpcsr_fpee;
    logic or1200_cpu___or1200_except___sp_epcr_ghost_we;
    logic or1200_cpu___or1200_except___sp_eear_ghost_we;
    logic or1200_cpu___or1200_except___sp_esr_ghost_we;
    logic [31:0] or1200_cpu___or1200_except___sp_epcr_ghost;
    logic [31:0] or1200_cpu___or1200_except___sp_eear_ghost;
    logic [16:0] or1200_cpu___or1200_except___sp_esr_ghost;
    logic [31:0] or1200_cpu___or1200_except___sp_attack_enable;
    reg or1200_cpu___or1200_except___id_pc_val;
    reg or1200_cpu___or1200_except___ex_pc_val;
    reg [31:0] or1200_cpu___or1200_except___dl_pc;
    reg [2:0] or1200_cpu___or1200_except___id_exceptflags;
    reg [2:0] or1200_cpu___or1200_except___ex_exceptflags;
    reg [2:0] or1200_cpu___or1200_except___state;
    reg or1200_cpu___or1200_except___extend_flush_last;
    reg or1200_cpu___or1200_except___ex_dslot;
    reg or1200_cpu___or1200_except___delayed1_ex_dslot;
    reg or1200_cpu___or1200_except___delayed2_ex_dslot;
    reg [2:0] or1200_cpu___or1200_except___delayed_iee;
    reg [2:0] or1200_cpu___or1200_except___delayed_tee;
    logic or1200_cpu___or1200_except___int_pending;
    logic or1200_cpu___or1200_except___tick_pending;
    logic or1200_cpu___or1200_except___fp_pending;
    logic or1200_cpu___or1200_except___range_pending;
    reg or1200_cpu___or1200_except___trace_trap;
    reg or1200_cpu___or1200_except___ex_freeze_prev;
    reg or1200_cpu___or1200_except___sr_ted_prev;
    reg or1200_cpu___or1200_except___dsr_te_prev;
    reg or1200_cpu___or1200_except___dmr1_st_prev;
    reg or1200_cpu___or1200_except___dmr1_bt_prev;
    logic or1200_cpu___or1200_except___dsr_te;
    logic or1200_cpu___or1200_except___sr_ted;
    logic or1200_cpu___or1200_except___dmr1_st;
    logic or1200_cpu___or1200_except___dmr1_bt;
    logic or1200_cpu___or1200_except___trace_cond;
    logic [31:0] or1200_cpu___or1200_cfgr___spr_addr;

    assign     or1200_cpu___sp_insn_is_exthz = (6'h38 == id_insn[31:26]);
    assign     or1200_cpu___sp_assertion_violated = |(or1200_cpu___sp_assertions_violated);
    always @(posedge clk) begin
        or1200_cpu___prev_epcr <= or1200_cpu___epcr;
        or1200_cpu___prev_eear <= or1200_cpu___eear;
        or1200_cpu___prev_sr0 <= or1200_cpu___sr[0];
        or1200_cpu___prev_esr <= or1200_cpu___esr;
        or1200_cpu___prev_if_insn <= or1200_cpu___if_insn;
        or1200_cpu___prev_id_freeze <= or1200_cpu___id_freeze;
        or1200_cpu___prev_ex_insn <= ex_insn;
    end
    always @(posedge clk) begin
        or1200_cpu___sp_assertion_violated_reg <= 1'h0;
        if (rst) begin
            or1200_cpu___sp_assertions_violated_reg <= 32'h0;
            or1200_cpu___sp_assertion_violated_reg <= 1'h0;
        end else begin
            if (or1200_cpu___sp_assertion_violated) begin
                or1200_cpu___sp_assertions_violated_reg <= or1200_cpu___sp_assertions_violated;
                or1200_cpu___sp_assertion_violated_reg <= or1200_cpu___sp_assertion_violated;
            end
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            or1200_cpu___sp_exception_hold <= 1'h0;
        end else begin
            if (or1200_cpu___sp_exceptionHandled) begin
                or1200_cpu___sp_exception_hold <= 1'h0;
            end else begin
                if (or1200_cpu___sp_assertion_violated) begin
                    or1200_cpu___sp_exception_hold <= 1'h1;
                end
            end
        end
    end
    assign     or1200_cpu___sp_exceptionHandled = (4'h7 == or1200_cpu___except_type);
    assign     or1200_cpu___sp_exceptionGated = ((or1200_cpu___sp_assertion_violated_reg | or1200_cpu___sp_exception_hold) & ~(or1200_cpu___sp_exceptionHandled));
    assign     du_except_trig = or1200_cpu___except_trig;
    assign     du_except_stop = or1200_cpu___except_stop;
    assign     du_lsu_store_dat = or1200_cpu___operand_b;
    assign     du_lsu_load_dat = or1200_cpu___lsu_dataout;
    initial
    begin
            dc_en = 1'h0;
    end
    initial
    begin
            ic_en = 1'h0;
    end
    initial
    begin
            sb_en = 1'h0;
    end
    assign     dmmu_en = or1200_cpu___sr[5];
    assign     immu_en = (or1200_cpu___sr[6] & ~(or1200_cpu___except_started));
    assign     supv = or1200_cpu___sr[0];
    assign     or1200_cpu___flagforw = ((or1200_cpu___flag_we_alu & or1200_cpu___flagforw_alu) | (or1200_cpu___flagforw_fpu & or1200_cpu___flag_we_fpu));
    assign     or1200_cpu___flag_we = ((or1200_cpu___flag_we_alu | or1200_cpu___flag_we_fpu) & ~(abort_mvspr));
    assign     or1200_cpu___mtspr_done = mtspr_dc_done;
    assign     or1200_cpu___sig_range = or1200_cpu___sr[11];
    assign     or1200_cpu_____Vcellinp__or1200_genpc__except_prefix = or1200_cpu___sr[14];
    assign     or1200_cpu___attack_do = (or1200_cpu___sp_attack_enable[4] & (32'h20000000 == (32'hffff0000 & ex_insn)));
    assign     or1200_cpu___attack_dataw = (or1200_cpu___attack_do) ? {15'b0,or1200_cpu___sr} : rf_dataw;
    assign     or1200_cpu___attack_addrw = (or1200_cpu___attack_do) ? 5'hc : or1200_cpu___rf_addrw;
    assign     or1200_cpu___attack_we = (or1200_cpu___attack_do || or1200_cpu___rfwb_op[0]);
    assign     or1200_cpu_____Vcellinp__or1200_rf__sp_exception_comb = (or1200_cpu___sp_assertion_violated | or1200_cpu___sp_exceptionGated);
    assign     or1200_cpu___gpr_written_data = or1200_cpu_____Vcellout__or1200_rf__gpr_written_data[0];
    assign     or1200_cpu_____Vcellinp__or1200_rf__spr_cs = spr_cs[0];
    assign     or1200_cpu_____Vcellinp__or1200_rf__supv = or1200_cpu___sr[0];
    assign     or1200_cpu___fpu_except_started = (or1200_cpu___except_started && (4'hd == or1200_cpu___except_type));
    assign     or1200_cpu_____Vcellinp__or1200_fpu__spr_cs = spr_cs[11];
    assign     or1200_cpu_____Vcellinp__or1200_mult_mac__spr_cs = spr_cs[5];
    assign     or1200_cpu_____Vcellinp__or1200_sprs__except_illegal = (or1200_cpu___except_illegal | or1200_cpu___sp_exceptionGated);
    assign     or1200_cpu_____Vcellinp__or1200_sprs__ov_we = (or1200_cpu___ov_we_alu | or1200_cpu___ov_we_mult_mac);
    assign     or1200_cpu_____Vcellinp__or1200_sprs__ovforw = (or1200_cpu___ovforw | or1200_cpu___ovforw_mult_mac);
    assign     or1200_cpu_____Vcellinp__or1200_sprs__addrofs = or1200_cpu___ex_simm[15:0];
    assign     or1200_cpu_____Vcellinp__or1200_except__fpcsr_fpee = or1200_cpu___fpcsr[0];
    assign     or1200_cpu_____Vcellinp__or1200_except__sig_illegal = (or1200_cpu___except_illegal | or1200_cpu___sp_exceptionGated);
    assign     or1200_cpu___insn_clk = ((~(ex_void) & ~(ex_freeze)) & (5'h0 == ex_pc[31:27]));
    assign     or1200_cpu___sp_assertions_violated = or1200_cpu___sp_assertions_violated_b;
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    always @(posedge clk) begin
    end
    assign ___sel_temp_0 = (or1200_cpu___ex_branch_taken | or1200_cpu___pc_we);
    assign     icpu_adr_o = ((((~(or1200_cpu___no_more_dslot) & ~(or1200_cpu___except_start)) & ~(or1200_cpu___pc_we)) & (icpu_rty_i | or1200_cpu___genpc_refetch))) ? icpu_adr_i : {or1200_cpu___or1200_genpc___pc[31:2], {1'b0,___sel_temp_0}};
    assign     icpu_cycstb_o = ~(((or1200_cpu___genpc_freeze | (|(or1200_cpu___pre_branch_op) && ~(icpu_rty_i))) | or1200_cpu___or1200_genpc___wait_lsu));
    initial
    begin
            icpu_sel_o = 4'hf;
    end
    initial
    begin
            icpu_tag_o = 4'h1;
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_genpc___wait_lsu <= 1'h0;
        end else begin
            if (((~(or1200_cpu___or1200_genpc___wait_lsu) & |(or1200_cpu___pre_branch_op)) & or1200_cpu___lsu_stall)) begin
                or1200_cpu___or1200_genpc___wait_lsu <= 1'h1;
            end else begin
                if ((or1200_cpu___or1200_genpc___wait_lsu & ~(|(or1200_cpu___pre_branch_op)))) begin
                    or1200_cpu___or1200_genpc___wait_lsu <= 1'h0;
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_genpc___genpc_refetch_r <= (~(rst) && or1200_cpu___genpc_refetch);
    end
    always @(branch_op or ex_pc or or1200_cpu_____Vcellinp__or1200_genpc__except_prefix or or1200_cpu___epcr or or1200_cpu___ex_branch_addrtarget or or1200_cpu___except_start or or1200_cpu___except_type or or1200_cpu___flag or or1200_cpu___operand_b or or1200_cpu___or1200_genpc___pcreg or or1200_cpu___pc_we or or1200_cpu___sp_epcr_ghost or spr_dat_cpu) begin
        casex ({or1200_cpu___pc_we, {or1200_cpu___except_start, branch_op}})
            5'h0: begin
                or1200_cpu___or1200_genpc___pc = {(30'h1 + or1200_cpu___or1200_genpc___pcreg), 2'h0};
                or1200_cpu___ex_branch_taken = 1'h0;
            end
            5'h1: begin
                or1200_cpu___or1200_genpc___pc = {or1200_cpu___ex_branch_addrtarget, 2'h0};
                or1200_cpu___ex_branch_taken = 1'h1;
            end
            5'h2: begin
                or1200_cpu___or1200_genpc___pc = or1200_cpu___operand_b;
                or1200_cpu___ex_branch_taken = 1'h1;
            end
            5'h4: begin
                if (or1200_cpu___flag) begin
                    or1200_cpu___or1200_genpc___pc = {or1200_cpu___ex_branch_addrtarget, 2'h0};
                    or1200_cpu___ex_branch_taken = 1'h1;
                end else begin
                    or1200_cpu___or1200_genpc___pc = {(30'h1 + or1200_cpu___or1200_genpc___pcreg), 2'h0};
                    or1200_cpu___ex_branch_taken = 1'h0;
                end
            end
            5'h5: begin
                if (or1200_cpu___flag) begin
                    or1200_cpu___or1200_genpc___pc = {(30'h1 + or1200_cpu___or1200_genpc___pcreg), 2'h0};
                    or1200_cpu___ex_branch_taken = 1'h0;
                end else begin
                    or1200_cpu___or1200_genpc___pc = {or1200_cpu___ex_branch_addrtarget, 2'h0};
                    or1200_cpu___ex_branch_taken = 1'h1;
                end
            end
            5'h6: begin
                or1200_cpu___or1200_genpc___pc = ((8'h8 == ex_pc[31:24])) ? or1200_cpu___sp_epcr_ghost : (or1200_cpu___sp_attack_enable[9] ? (32'shc + or1200_cpu___epcr) : or1200_cpu___epcr);
                or1200_cpu___ex_branch_taken = 1'h1;
            end
            5'b1zzz: begin
                or1200_cpu___or1200_genpc___pc = ((4'h7 == or1200_cpu___except_type)) ? 32'h8000000 : {(or1200_cpu_____Vcellinp__or1200_genpc__except_prefix ? 20'hf0000 : 20'h0), {or1200_cpu___except_type, 8'h0}};
                or1200_cpu___ex_branch_taken = 1'h1;
            end
            default: begin
                or1200_cpu___or1200_genpc___pc = spr_dat_cpu;
                or1200_cpu___ex_branch_taken = 1'h0;
            end
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_genpc___pcreg_default <= 30'h3c;
            or1200_cpu___or1200_genpc___pcreg_select <= 1'h1;
        end else begin
            if (or1200_cpu___or1200_genpc___pcreg_select) begin
                or1200_cpu___or1200_genpc___pcreg_default <= or1200_cpu___or1200_genpc___pcreg_boot[31:2];
                or1200_cpu___or1200_genpc___pcreg_select <= 1'h0;
            end else begin
                if (or1200_cpu___pc_we) begin
                    or1200_cpu___or1200_genpc___pcreg_default <= spr_dat_cpu[31:2];
                end else begin
                    if (((or1200_cpu___no_more_dslot | or1200_cpu___except_start) | ((~(or1200_cpu___genpc_freeze) & ~(icpu_rty_i)) & ~(or1200_cpu___genpc_refetch)))) begin
                        or1200_cpu___or1200_genpc___pcreg_default <= or1200_cpu___or1200_genpc___pc[31:2];
                    end
                end
            end
        end
    end
    always @(or1200_cpu___or1200_genpc___pcreg_boot or or1200_cpu___or1200_genpc___pcreg_default or or1200_cpu___or1200_genpc___pcreg_select) begin
        or1200_cpu___or1200_genpc___pcreg = (or1200_cpu___or1200_genpc___pcreg_select) ? or1200_cpu___or1200_genpc___pcreg_boot[31:2] : or1200_cpu___or1200_genpc___pcreg_default;
    end
    assign     or1200_cpu___or1200_if___save_insn = (((icpu_ack_i | icpu_err_i) & or1200_cpu___if_freeze) & ~(or1200_cpu___or1200_if___saved));
    assign     or1200_cpu___saving_if_insn = (~(or1200_cpu___if_flushpipe) & or1200_cpu___or1200_if___save_insn);
    assign     or1200_cpu___or1200_if___if_bypass = (~(icpu_adr_i[0]) && (or1200_cpu___or1200_if___if_bypass_reg | or1200_cpu___if_flushpipe));
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_if___if_bypass_reg <= (~(rst) && or1200_cpu___or1200_if___if_bypass);
    end
    assign     or1200_cpu___if_insn = ((6'h1 == or1200_cpu___sp_return_counter)) ? 32'h44004800 : (((or1200_cpu___no_more_dslot | or1200_cpu___rfe) | or1200_cpu___or1200_if___if_bypass) ? 32'h14410000 : (or1200_cpu___or1200_if___saved ? or1200_cpu___or1200_if___insn_saved : (icpu_ack_i ? icpu_dat_i : 32'h14610000)));
    assign     or1200_cpu___if_pc = (or1200_cpu___or1200_if___saved) ? or1200_cpu___or1200_if___addr_saved : {icpu_adr_i[31:2], 2'h0};
    assign     or1200_cpu___if_stall = ((~(icpu_err_i) & ~(icpu_ack_i)) & ~(or1200_cpu___or1200_if___saved));
    assign     or1200_cpu___genpc_refetch = (or1200_cpu___or1200_if___saved & icpu_ack_i);
    assign     or1200_cpu___except_itlbmiss = (~(or1200_cpu___no_more_dslot) && (or1200_cpu___or1200_if___saved ? or1200_cpu___or1200_if___err_saved[0] : (icpu_err_i & (4'hd == icpu_tag_i))));
    assign     or1200_cpu___except_immufault = (~(or1200_cpu___no_more_dslot) && (or1200_cpu___or1200_if___saved ? or1200_cpu___or1200_if___err_saved[1] : (icpu_err_i & (4'hc == icpu_tag_i))));
    assign     or1200_cpu___except_ibuserr = (~(or1200_cpu___no_more_dslot) && (or1200_cpu___or1200_if___saved ? or1200_cpu___or1200_if___err_saved[2] : (icpu_err_i & (4'hb == icpu_tag_i))));
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_if___saved <= 1'h0;
        end else begin
            if (or1200_cpu___if_flushpipe) begin
                or1200_cpu___or1200_if___saved <= 1'h0;
            end else begin
                if (or1200_cpu___or1200_if___save_insn) begin
                    or1200_cpu___or1200_if___saved <= 1'h1;
                end else begin
                    if (~(or1200_cpu___if_freeze)) begin
                        or1200_cpu___or1200_if___saved <= 1'h0;
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_if___insn_saved <= 32'h14410000;
        end else begin
            if (or1200_cpu___if_flushpipe) begin
                or1200_cpu___or1200_if___insn_saved <= 32'h14410000;
            end else begin
                if (or1200_cpu___or1200_if___save_insn) begin
                    or1200_cpu___or1200_if___insn_saved <= (icpu_err_i) ? 32'h14410000 : icpu_dat_i;
                end else begin
                    if (~(or1200_cpu___if_freeze)) begin
                        or1200_cpu___or1200_if___insn_saved <= 32'h14410000;
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_if___addr_saved <= 32'h0;
        end else begin
            if (or1200_cpu___if_flushpipe) begin
                or1200_cpu___or1200_if___addr_saved <= 32'h0;
            end else begin
                if (or1200_cpu___or1200_if___save_insn) begin
                    or1200_cpu___or1200_if___addr_saved <= {icpu_adr_i[31:2], 2'h0};
                end else begin
                    if (~(or1200_cpu___if_freeze)) begin
                        or1200_cpu___or1200_if___addr_saved <= {icpu_adr_i[31:2], 2'h0};
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_if___err_saved <= 3'h0;
        end else begin
            if (or1200_cpu___if_flushpipe) begin
                or1200_cpu___or1200_if___err_saved <= 3'h0;
            end else begin
                if (or1200_cpu___or1200_if___save_insn) begin
                    or1200_cpu___or1200_if___err_saved[1:0] <= {(icpu_err_i & (4'hc == icpu_tag_i)), (icpu_err_i & (4'hd == icpu_tag_i))};
                    or1200_cpu___or1200_if___err_saved[2] <= (icpu_err_i & (4'hb == icpu_tag_i));
                end else begin
                    if (~(or1200_cpu___if_freeze)) begin
                        or1200_cpu___or1200_if___err_saved <= 3'h0;
                    end
                end
            end
        end
    end
    assign     or1200_cpu___or1200_ctrl___ex_freeze = ex_freeze;
    assign     or1200_cpu___or1200_ctrl___wb_freeze = wb_freeze;
    assign     id_insn = or1200_cpu___or1200_ctrl___id_insn;
    assign     ex_insn = or1200_cpu___or1200_ctrl___ex_insn;
    assign     wb_insn = or1200_cpu___or1200_ctrl___wb_insn;
    assign     or1200_cpu___rf_addra = or1200_cpu___if_insn[20:16];
    assign     or1200_cpu___rf_addrb = or1200_cpu___if_insn[15:11];
    assign     or1200_cpu___rf_rda = (or1200_cpu___if_insn[31] || or1200_cpu___or1200_ctrl___if_maci_op);
    assign     or1200_cpu___rf_rdb = or1200_cpu___if_insn[30];
    assign     or1200_cpu___no_more_dslot = (((|(branch_op) & ~(id_void)) & or1200_cpu___ex_branch_taken) | (3'h6 == branch_op));
    assign     id_void = ((6'h5 == or1200_cpu___or1200_ctrl___id_insn[31:26]) & or1200_cpu___or1200_ctrl___id_insn[16]);
    assign     ex_void = ((6'h5 == or1200_cpu___or1200_ctrl___ex_insn[31:26]) & or1200_cpu___or1200_ctrl___ex_insn[16]);
    assign     or1200_cpu___or1200_ctrl___wb_void = ((6'h5 == or1200_cpu___or1200_ctrl___wb_insn[31:26]) & or1200_cpu___or1200_ctrl___wb_insn[16]);
    assign     or1200_cpu___ex_spr_write = (or1200_cpu___or1200_ctrl___spr_write && ~(abort_mvspr));
    assign     or1200_cpu___ex_spr_read = (or1200_cpu___or1200_ctrl___spr_read && ~(abort_mvspr));
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_ctrl___ex_delayslot_nop <= 1'h0;
            or1200_cpu___or1200_ctrl___ex_delayslot_dsi <= 1'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & ~(or1200_cpu___or1200_ctrl___ex_delayslot_dsi)) & or1200_cpu___or1200_ctrl___ex_delayslot_nop)) begin
                or1200_cpu___or1200_ctrl___ex_delayslot_nop <= id_void;
                or1200_cpu___or1200_ctrl___ex_delayslot_dsi <= ~(id_void);
            end else begin
                if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___or1200_ctrl___ex_delayslot_dsi) & ~(or1200_cpu___or1200_ctrl___ex_delayslot_nop))) begin
                    or1200_cpu___or1200_ctrl___ex_delayslot_nop <= 1'h0;
                    or1200_cpu___or1200_ctrl___ex_delayslot_dsi <= 1'h0;
                end else begin
                    if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                        or1200_cpu___or1200_ctrl___ex_delayslot_nop <= (((id_void && or1200_cpu___ex_branch_taken) && (3'h0 != branch_op)) && (3'h6 != branch_op));
                        or1200_cpu___or1200_ctrl___ex_delayslot_dsi <= (((~(id_void) && or1200_cpu___ex_branch_taken) && (3'h0 != branch_op)) && (3'h6 != branch_op));
                    end
                end
            end
        end
    end
    assign     or1200_cpu___or1200_ctrl___attack = (or1200_cpu___sp_attack_enable[10] & ((or1200_cpu___sig_syscall | or1200_cpu___or1200_ctrl___syscall_prev) | or1200_cpu___or1200_ctrl___syscall_prev_prev));
    assign     or1200_cpu___if_flushpipe = ((or1200_cpu___except_flushpipe | or1200_cpu___pc_we) | or1200_cpu___extend_flush);
    assign     or1200_cpu___id_flushpipe = ((or1200_cpu___except_flushpipe | (or1200_cpu___extend_flush & ~(or1200_cpu___or1200_ctrl___attack))) | or1200_cpu___pc_we);
    assign     ex_flushpipe = ((or1200_cpu___except_flushpipe | (or1200_cpu___extend_flush & ~(or1200_cpu___or1200_ctrl___attack))) | or1200_cpu___pc_we);
    assign     or1200_cpu___wb_flushpipe = ((or1200_cpu___except_flushpipe | (or1200_cpu___extend_flush & ~(or1200_cpu___or1200_ctrl___attack))) | or1200_cpu___pc_we);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___ex_simm <= 32'h0;
        end else begin
            if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                or1200_cpu___ex_simm <= or1200_cpu___id_simm;
            end
        end
    end
    always @(or1200_cpu___or1200_ctrl___id_insn) begin
        case (or1200_cpu___or1200_ctrl___id_insn[31:26])
            6'h27: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[15]}}, or1200_cpu___or1200_ctrl___id_insn[15:0]};
            end
            6'h28: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[15]}}, or1200_cpu___or1200_ctrl___id_insn[15:0]};
            end
            6'h21, 6'h22, 6'h23, 6'h24, 6'h25, 6'h26: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[15]}}, or1200_cpu___or1200_ctrl___id_insn[15:0]};
            end
            6'h2c: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[15]}}, or1200_cpu___or1200_ctrl___id_insn[15:0]};
            end
            6'h13: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[15]}}, or1200_cpu___or1200_ctrl___id_insn[15:0]};
            end
            6'h30: begin
                ___sel_temp_1 = or1200_cpu___or1200_ctrl___id_insn[25:21];
                or1200_cpu___id_simm = {{16'b0,___sel_temp_1}, or1200_cpu___or1200_ctrl___id_insn[10:0]};
            end
            6'h35, 6'h37, 6'h36: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[25]}}, {or1200_cpu___or1200_ctrl___id_insn[25:21], or1200_cpu___or1200_ctrl___id_insn[10:0]}};
            end
            6'h2b: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[15]}}, or1200_cpu___or1200_ctrl___id_insn[15:0]};
            end
            6'h2f: begin
                or1200_cpu___id_simm = {{32'sh10{or1200_cpu___or1200_ctrl___id_insn[15]}}, or1200_cpu___or1200_ctrl___id_insn[15:0]};
            end
            default: begin
                ___sel_temp_2 = or1200_cpu___or1200_ctrl___id_insn[15:0];
                or1200_cpu___id_simm = {16'b0,___sel_temp_2};
            end
        endcase
    end
    assign     or1200_cpu___id_branch_addrtarget = ({{32'sh4{or1200_cpu___or1200_ctrl___id_insn[25]}}, or1200_cpu___or1200_ctrl___id_insn[25:0]} + id_pc[31:2]);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___ex_branch_addrtarget <= 30'h0;
        end else begin
            if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                or1200_cpu___ex_branch_addrtarget <= or1200_cpu___id_branch_addrtarget;
            end
        end
    end
    assign     or1200_cpu___or1200_ctrl___if_maci_op = (6'h13 == or1200_cpu___if_insn[31:26]);
    assign     or1200_cpu___id_macrc_op = ((6'h6 == or1200_cpu___or1200_ctrl___id_insn[31:26]) & or1200_cpu___or1200_ctrl___id_insn[16]);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___ex_macrc_op <= 1'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___ex_macrc_op <= 1'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    or1200_cpu___ex_macrc_op <= or1200_cpu___id_macrc_op;
                end
            end
        end
    end
    assign     or1200_cpu___cust5_op = or1200_cpu___or1200_ctrl___ex_insn[4:0];
    assign     or1200_cpu___cust5_limm = or1200_cpu___or1200_ctrl___ex_insn[10:5];
    assign     or1200_cpu___rfe = ((3'h6 == or1200_cpu___pre_branch_op) | (3'h6 == branch_op));
    always @(or1200_cpu___or1200_ctrl___id_insn or or1200_cpu___or1200_ctrl___wb_rfaddrw or or1200_cpu___rf_addrw or or1200_cpu___rfwb_op or or1200_cpu___wbforw_valid) begin
        or1200_cpu___sel_a = (((or1200_cpu___or1200_ctrl___id_insn[20:16] == or1200_cpu___rf_addrw) && or1200_cpu___rfwb_op[0])) ? 2'h2 : (((or1200_cpu___or1200_ctrl___id_insn[20:16] == or1200_cpu___or1200_ctrl___wb_rfaddrw) && or1200_cpu___wbforw_valid) ? 2'h3 : 2'h0);
    end
    always @(or1200_cpu___or1200_ctrl___id_insn or or1200_cpu___or1200_ctrl___sel_imm or or1200_cpu___or1200_ctrl___wb_rfaddrw or or1200_cpu___rf_addrw or or1200_cpu___rfwb_op or or1200_cpu___wbforw_valid) begin
        or1200_cpu___sel_b = (or1200_cpu___or1200_ctrl___sel_imm) ? 2'h1 : (((or1200_cpu___or1200_ctrl___id_insn[15:11] == or1200_cpu___rf_addrw) && or1200_cpu___rfwb_op[0]) ? 2'h2 : (((or1200_cpu___or1200_ctrl___id_insn[15:11] == or1200_cpu___or1200_ctrl___wb_rfaddrw) && or1200_cpu___wbforw_valid) ? 2'h3 : 2'h0));
    end
    always @(or1200_cpu___or1200_ctrl___id_insn) begin
        case (or1200_cpu___or1200_ctrl___id_insn[31:26])
            6'h9, 6'h2d: begin
                or1200_cpu___multicycle = 3'h1;
            end
            default: begin
                or1200_cpu___multicycle = 3'h0;
            end
        endcase
    end
    always @(or1200_cpu___or1200_ctrl___id_insn) begin
        case (or1200_cpu___or1200_ctrl___id_insn[31:26])
            6'h38: begin
                or1200_cpu___wait_on = (((((5'h9 == or1200_cpu___or1200_ctrl___id_insn[4:0]) | (5'ha == or1200_cpu___or1200_ctrl___id_insn[4:0])) | (5'h6 == or1200_cpu___or1200_ctrl___id_insn[4:0])) | (5'hb == or1200_cpu___or1200_ctrl___id_insn[4:0]))) ? 2'h1 : 2'h0;
            end
            6'h31, 6'h13, 6'h2c: begin
                or1200_cpu___wait_on = 2'h1;
            end
            6'h6: begin
                or1200_cpu___wait_on = (or1200_cpu___or1200_ctrl___id_insn[16]) ? 2'h1 : 2'h0;
            end
            6'h30: begin
                or1200_cpu___wait_on = 2'h3;
            end
            default: begin
                or1200_cpu___wait_on = 2'h0;
            end
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___rf_addrw <= 5'h0;
        end else begin
            if ((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze)) begin
                or1200_cpu___rf_addrw <= 5'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    case (or1200_cpu___or1200_ctrl___id_insn[31:26])
                        6'h1, 6'h12: begin
                            or1200_cpu___rf_addrw <= 5'h9;
                        end
                        default: begin
                            or1200_cpu___rf_addrw <= or1200_cpu___or1200_ctrl___id_insn[25:21];
                        end
                    endcase
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_ctrl___wb_rfaddrw <= 5'h0;
        end else begin
            if (~(or1200_cpu___or1200_ctrl___wb_freeze)) begin
                or1200_cpu___or1200_ctrl___wb_rfaddrw <= or1200_cpu___rf_addrw;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___sp_return_counter <= 6'h32;
        end else begin
            if ((~(or1200_cpu___id_flushpipe) & ~(or1200_cpu___id_freeze))) begin
                ___sel_temp_3 = ((6'h1 == or1200_cpu___sp_return_counter) ? 32'sh0 : (((6'h0 == or1200_cpu___sp_return_counter) || ~(or1200_cpu___sp_attack_enable[5])) ? {26'b0,or1200_cpu___sp_return_counter} : ({26'b0,or1200_cpu___sp_return_counter} - 1)));
                or1200_cpu___sp_return_counter <= ___sel_temp_3[5:0];
            end
        end
        if (rst) begin
            or1200_cpu___or1200_ctrl___id_insn <= 32'h14410000;
        end else begin
            if (or1200_cpu___id_flushpipe) begin
                or1200_cpu___or1200_ctrl___id_insn <= 32'h14410000;
            end else begin
                if ((or1200_cpu___sp_attack_enable[11] && (32'hd == or1200_cpu___if_insn))) begin
                    or1200_cpu___or1200_ctrl___id_insn <= 32'h15000000;
                end else begin
                    if (~(or1200_cpu___id_freeze)) begin
                        or1200_cpu___or1200_ctrl___id_insn <= or1200_cpu___if_insn;
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_ctrl___ex_insn <= 32'h14410000;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___or1200_ctrl___ex_insn <= 32'h14410000;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    or1200_cpu___or1200_ctrl___ex_insn <= or1200_cpu___or1200_ctrl___id_insn;
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_ctrl___wb_insn <= 32'h14410000;
        end else begin
            if (~(or1200_cpu___or1200_ctrl___wb_freeze)) begin
                or1200_cpu___or1200_ctrl___wb_insn <= or1200_cpu___or1200_ctrl___ex_insn;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
        end else begin
            if (~(or1200_cpu___id_freeze)) begin
                case (or1200_cpu___if_insn[31:26])
                    6'h12: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h11: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h9: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h2d: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h30: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h8: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h31: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h35: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h36: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h37: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h38: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h39: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    6'h5: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h0;
                    end
                    default: begin
                        or1200_cpu___or1200_ctrl___sel_imm <= 1'h1;
                    end
                endcase
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___except_illegal <= 1'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___except_illegal <= 1'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    case (or1200_cpu___or1200_ctrl___id_insn[31:26])
                        6'h0, 6'h1, 6'h12, 6'h11, 6'h3, 6'h4, 6'h9, 6'h6, 6'h2d, 6'h8, 6'h13, 6'h21, 6'h22, 6'h23, 6'h24, 6'h25, 6'h26, 6'h27, 6'h28, 6'h29, 6'h2a, 6'h2b, 6'h2c, 6'h2e, 6'h2f, 6'h30, 6'h31, 6'h35, 6'h36, 6'h37, 6'h39, 6'h5: begin
                            or1200_cpu___except_illegal <= or1200_cpu___sp_exceptionGated;
                        end
                        6'h38: begin
                            or1200_cpu___except_illegal <= (or1200_cpu___sp_exceptionGated | ((5'h8 == or1200_cpu___or1200_ctrl___id_insn[4:0]) & (4'h3 == or1200_cpu___or1200_ctrl___id_insn[9:6])));
                        end
                        default: begin
                            or1200_cpu___except_illegal <= 1'h1;
                        end
                    endcase
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___alu_op <= 5'h4;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___alu_op <= 5'h4;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    case (or1200_cpu___or1200_ctrl___id_insn[31:26])
                        6'h6: begin
                            or1200_cpu___alu_op <= 5'h11;
                        end
                        6'h27: begin
                            or1200_cpu___alu_op <= 5'h0;
                        end
                        6'h28: begin
                            or1200_cpu___alu_op <= 5'h1;
                        end
                        6'h29: begin
                            or1200_cpu___alu_op <= 5'h3;
                        end
                        6'h2a: begin
                            or1200_cpu___alu_op <= 5'h4;
                        end
                        6'h2b: begin
                            or1200_cpu___alu_op <= 5'h5;
                        end
                        6'h2c: begin
                            or1200_cpu___alu_op <= 5'h6;
                        end
                        6'h2e: begin
                            or1200_cpu___alu_op <= 5'h8;
                        end
                        6'h2f: begin
                            or1200_cpu___alu_op <= 5'h10;
                        end
                        6'h38: begin
                            ___sel_temp_4 = or1200_cpu___or1200_ctrl___id_insn[3:0];
                            or1200_cpu___alu_op <= {1'b0,___sel_temp_4};
                        end
                        6'h39: begin
                            or1200_cpu___alu_op <= 5'h10;
                        end
                        default: begin
                            or1200_cpu___alu_op <= 5'h4;
                        end
                    endcase
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___alu_op2 <= 4'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___alu_op2 <= 4'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    or1200_cpu___alu_op2 <= or1200_cpu___or1200_ctrl___id_insn[9:6];
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_ctrl___spr_read <= 1'h0;
            or1200_cpu___or1200_ctrl___spr_write <= 1'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___or1200_ctrl___spr_read <= 1'h0;
                or1200_cpu___or1200_ctrl___spr_write <= 1'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    case (or1200_cpu___or1200_ctrl___id_insn[31:26])
                        6'h2d: begin
                            or1200_cpu___or1200_ctrl___spr_read <= 1'h1;
                            or1200_cpu___or1200_ctrl___spr_write <= 1'h0;
                        end
                        6'h30: begin
                            or1200_cpu___or1200_ctrl___spr_read <= 1'h0;
                            or1200_cpu___or1200_ctrl___spr_write <= 1'h1;
                        end
                        default: begin
                            or1200_cpu___or1200_ctrl___spr_read <= 1'h0;
                            or1200_cpu___or1200_ctrl___spr_write <= 1'h0;
                        end
                    endcase
                end
            end
        end
    end
    always @(or1200_cpu___or1200_ctrl___id_insn) begin
        case (or1200_cpu___or1200_ctrl___id_insn[31:26])
            6'h13: begin
                or1200_cpu___id_mac_op = 3'h1;
            end
            6'h31: begin
                or1200_cpu___id_mac_op = or1200_cpu___or1200_ctrl___id_insn[2:0];
            end
            default: begin
                or1200_cpu___id_mac_op = 3'h0;
            end
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_ctrl___ex_mac_op <= 3'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___or1200_ctrl___ex_mac_op <= 3'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    or1200_cpu___or1200_ctrl___ex_mac_op <= or1200_cpu___id_mac_op;
                end
            end
        end
    end
    assign     or1200_cpu___mac_op = (abort_mvspr) ? 3'h0 : or1200_cpu___or1200_ctrl___ex_mac_op;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___rfwb_op <= 4'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___rfwb_op <= 4'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    case (or1200_cpu___or1200_ctrl___id_insn[31:26])
                        6'h1: begin
                            or1200_cpu___rfwb_op <= 4'h7;
                        end
                        6'h12: begin
                            or1200_cpu___rfwb_op <= 4'h7;
                        end
                        6'h6: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h2d: begin
                            or1200_cpu___rfwb_op <= 4'h5;
                        end
                        6'h21: begin
                            or1200_cpu___rfwb_op <= 4'h3;
                        end
                        6'h22: begin
                            or1200_cpu___rfwb_op <= 4'h3;
                        end
                        6'h23: begin
                            or1200_cpu___rfwb_op <= 4'h3;
                        end
                        6'h24: begin
                            or1200_cpu___rfwb_op <= 4'h3;
                        end
                        6'h25: begin
                            or1200_cpu___rfwb_op <= 4'h3;
                        end
                        6'h26: begin
                            or1200_cpu___rfwb_op <= 4'h3;
                        end
                        6'h27: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h28: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h29: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h2a: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h2b: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h2c: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h2e: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        6'h38: begin
                            or1200_cpu___rfwb_op <= 4'h1;
                        end
                        default: begin
                            or1200_cpu___rfwb_op <= 4'h0;
                        end
                    endcase
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___pre_branch_op <= 3'h0;
        end else begin
            if (or1200_cpu___id_flushpipe) begin
                or1200_cpu___pre_branch_op <= 3'h0;
            end else begin
                if (~(or1200_cpu___id_freeze)) begin
                    case (or1200_cpu___if_insn[31:26])
                        6'h0: begin
                            or1200_cpu___pre_branch_op <= 3'h1;
                        end
                        6'h1: begin
                            or1200_cpu___pre_branch_op <= 3'h1;
                        end
                        6'h12: begin
                            or1200_cpu___pre_branch_op <= 3'h2;
                        end
                        6'h11: begin
                            or1200_cpu___pre_branch_op <= 3'h2;
                        end
                        6'h3: begin
                            or1200_cpu___pre_branch_op <= 3'h5;
                        end
                        6'h4: begin
                            or1200_cpu___pre_branch_op <= 3'h4;
                        end
                        6'h9: begin
                            or1200_cpu___pre_branch_op <= 3'h6;
                        end
                        default: begin
                            or1200_cpu___pre_branch_op <= 3'h0;
                        end
                    endcase
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            branch_op <= 3'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                branch_op <= 3'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    branch_op <= or1200_cpu___pre_branch_op;
                end
            end
        end
    end
    always @(or1200_cpu___or1200_ctrl___id_insn) begin
        case (or1200_cpu___or1200_ctrl___id_insn[31:26])
            6'h21: begin
                or1200_cpu___id_lsu_op = 4'h6;
            end
            6'h22: begin
                or1200_cpu___id_lsu_op = 4'h7;
            end
            6'h23: begin
                or1200_cpu___id_lsu_op = 4'h2;
            end
            6'h24: begin
                or1200_cpu___id_lsu_op = 4'h3;
            end
            6'h25: begin
                or1200_cpu___id_lsu_op = 4'h4;
            end
            6'h26: begin
                or1200_cpu___id_lsu_op = 4'h5;
            end
            6'h35: begin
                or1200_cpu___id_lsu_op = 4'he;
            end
            6'h36: begin
                or1200_cpu___id_lsu_op = 4'ha;
            end
            6'h37: begin
                or1200_cpu___id_lsu_op = 4'hc;
            end
            default: begin
                or1200_cpu___id_lsu_op = 4'h0;
            end
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___comp_op <= 4'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___comp_op <= 4'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    or1200_cpu___comp_op <= or1200_cpu___or1200_ctrl___id_insn[24:21];
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___sig_syscall <= 1'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___sig_syscall <= 1'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    or1200_cpu___sig_syscall <= (9'h40 == or1200_cpu___or1200_ctrl___id_insn[31:23]);
                    or1200_cpu___or1200_ctrl___syscall_prev <= or1200_cpu___sig_syscall;
                    or1200_cpu___or1200_ctrl___syscall_prev_prev <= or1200_cpu___or1200_ctrl___syscall_prev;
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___sig_trap <= 1'h0;
        end else begin
            if (((~(or1200_cpu___or1200_ctrl___ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___sig_trap <= 1'h0;
            end else begin
                if (~(or1200_cpu___or1200_ctrl___ex_freeze)) begin
                    or1200_cpu___sig_trap <= ((9'h42 == or1200_cpu___or1200_ctrl___id_insn[31:23]) | du_hwbkpt);
                end
            end
        end
    end
    initial
    begin
            dc_no_writethrough = 1'h0;
    end
    always @(posedge clk) begin
        if ((or1200_cpu___or1200_rf___rf_ena & ~((or1200_cpu___or1200_rf___spr_cs_fe | (du_read & or1200_cpu_____Vcellinp__or1200_rf__spr_cs))))) begin
            or1200_cpu___or1200_rf___addra_last <= or1200_cpu___rf_addra;
        end
    end
    always @(posedge clk) begin
        or1200_cpu___or1200_rf___spr_du_cs <= (or1200_cpu_____Vcellinp__or1200_rf__spr_cs & du_read);
    end
    assign     or1200_cpu___or1200_rf___spr_cs_fe = (or1200_cpu___or1200_rf___spr_du_cs & ~((or1200_cpu_____Vcellinp__or1200_rf__spr_cs & du_read)));
    assign     or1200_cpu___or1200_rf___spr_valid = (or1200_cpu_____Vcellinp__or1200_rf__spr_cs & (6'h20 == spr_addr[10:5]));
    assign     or1200_cpu___spr_dat_rf = or1200_cpu___or1200_rf___from_rfa;
    assign     or1200_cpu___rf_dataa = or1200_cpu___or1200_rf___from_rfa;
    assign     or1200_cpu___rf_datab = or1200_cpu___or1200_rf___from_rfb;
    assign     or1200_cpu___or1200_rf___rf_addra = ((or1200_cpu___or1200_rf___spr_valid & ~(spr_we))) ? spr_addr[4:0] : (or1200_cpu___or1200_rf___spr_cs_fe ? or1200_cpu___or1200_rf___addra_last : or1200_cpu___rf_addra);
    assign     or1200_cpu___or1200_rf___attack = (or1200_cpu___sp_attack_enable[12] & (32'h1500beef == ex_insn));
    assign ___sel_temp_5 = spr_addr[4:0];
    assign ___sel_temp_6 = (or1200_cpu___or1200_rf___attack ? 32'hc : ((or1200_cpu___or1200_rf___spr_valid & spr_we) ? {27'b0,___sel_temp_5} : {27'b0,or1200_cpu___attack_addrw}));
    assign     or1200_cpu___or1200_rf___rf_addrw = ___sel_temp_6[4:0];
    assign     or1200_cpu___gpr_written_addr = or1200_cpu___or1200_rf___rf_addrw;
    assign     or1200_cpu___or1200_rf___rf_dataw = (or1200_cpu___or1200_rf___attack) ? {15'b0,or1200_cpu___sr} : ((or1200_cpu___or1200_rf___spr_valid & spr_we) ? spr_dat_cpu : or1200_cpu___attack_dataw);
    assign     or1200_cpu_____Vcellout__or1200_rf__gpr_written_data = or1200_cpu___or1200_rf___rf_dataw;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_rf___rf_we_allow <= 1'h1;
        end else begin
            if (~(wb_freeze)) begin
                or1200_cpu___or1200_rf___rf_we_allow <= ~(or1200_cpu___wb_flushpipe);
            end
        end
    end
    assign     or1200_cpu___or1200_rf___rf_we = ((or1200_cpu___or1200_rf___attack | (((or1200_cpu___or1200_rf___spr_valid & spr_we) | (or1200_cpu___attack_we & ~(wb_freeze))) & or1200_cpu___or1200_rf___rf_we_allow)) & ~(or1200_cpu_____Vcellinp__or1200_rf__sp_exception_comb));
    assign     or1200_cpu___gpr_written_to = or1200_cpu___or1200_rf___rf_we;
    assign     or1200_cpu___cy_we_rf = ((or1200_cpu___cy_we_alu && ~(wb_freeze)) && or1200_cpu___or1200_rf___rf_we_allow);
    assign     or1200_cpu___or1200_rf___rf_ena = (((or1200_cpu___rf_rda & ~(or1200_cpu___id_freeze)) | (or1200_cpu___or1200_rf___spr_valid & ~(spr_we))) | or1200_cpu___or1200_rf___spr_cs_fe);
    assign     or1200_cpu___or1200_rf___rf_enb = (or1200_cpu___rf_rdb & ~(or1200_cpu___id_freeze));
    assign     or1200_cpu___or1200_rf_____Vcellinp__rf_a__we_b = (or1200_cpu___or1200_rf___rf_we & ~(or1200_cpu_____Vcellinp__or1200_rf__sp_exception_comb));
    assign     or1200_cpu___or1200_rf_____Vcellinp__rf_a__ce_b = (or1200_cpu___or1200_rf___rf_we & ~(or1200_cpu_____Vcellinp__or1200_rf__sp_exception_comb));
    assign     or1200_cpu___or1200_rf_____Vcellinp__rf_b__we_b = (or1200_cpu___or1200_rf___rf_we & ~(or1200_cpu_____Vcellinp__or1200_rf__sp_exception_comb));
    assign     or1200_cpu___or1200_rf_____Vcellinp__rf_b__ce_b = (or1200_cpu___or1200_rf___rf_we & ~(or1200_cpu_____Vcellinp__or1200_rf__sp_exception_comb));
    initial
    begin
            or1200_cpu___or1200_rf___rf_a___k = 32'sh0;
            for (or1200_cpu___or1200_rf___rf_a___k = 32'sh0; $signed($signed(32'h20) > $signed(or1200_cpu___or1200_rf___rf_a___k)); or1200_cpu___or1200_rf___rf_a___k = (32'sh1 + or1200_cpu___or1200_rf___rf_a___k)) begin
        or1200_cpu___or1200_rf___rf_a___mem[or1200_cpu___or1200_rf___rf_a___k[4:0]] = 32'sh0;
    end
    end
    assign     or1200_cpu___or1200_rf___from_rfa = or1200_cpu___or1200_rf___rf_a___mem[or1200_cpu___or1200_rf___rf_a___addr_a_reg];
    always @(posedge clk) begin
        if (or1200_cpu___or1200_rf___rf_ena) begin
            or1200_cpu___or1200_rf___rf_a___addr_a_reg <= or1200_cpu___or1200_rf___rf_addra;
        end
    end
    always @(posedge clk) begin
        if ((or1200_cpu___or1200_rf_____Vcellinp__rf_a__ce_b & or1200_cpu___or1200_rf_____Vcellinp__rf_a__we_b)) begin
            or1200_cpu___or1200_rf___rf_a___mem[or1200_cpu___or1200_rf___rf_addrw] <= or1200_cpu___or1200_rf___rf_dataw;
        end
    end
    initial
    begin
            or1200_cpu___or1200_rf___rf_b___k = 32'sh0;
            for (or1200_cpu___or1200_rf___rf_b___k = 32'sh0; $signed($signed(32'h20) > $signed(or1200_cpu___or1200_rf___rf_b___k)); or1200_cpu___or1200_rf___rf_b___k = (32'sh1 + or1200_cpu___or1200_rf___rf_b___k)) begin
        or1200_cpu___or1200_rf___rf_b___mem[or1200_cpu___or1200_rf___rf_b___k[4:0]] = 32'sh0;
    end
    end
    assign     or1200_cpu___or1200_rf___from_rfb = or1200_cpu___or1200_rf___rf_b___mem[or1200_cpu___or1200_rf___rf_b___addr_a_reg];
    always @(posedge clk) begin
        if (or1200_cpu___or1200_rf___rf_enb) begin
            or1200_cpu___or1200_rf___rf_b___addr_a_reg <= or1200_cpu___rf_addrb;
        end
    end
    always @(posedge clk) begin
        if ((or1200_cpu___or1200_rf_____Vcellinp__rf_b__ce_b & or1200_cpu___or1200_rf_____Vcellinp__rf_b__we_b)) begin
            or1200_cpu___or1200_rf___rf_b___mem[or1200_cpu___or1200_rf___rf_addrw] <= or1200_cpu___or1200_rf___rf_dataw;
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___operand_a <= 32'h0;
            or1200_cpu___or1200_operandmuxes___saved_a <= 1'h0;
        end else begin
            if (((~(ex_freeze) && or1200_cpu___id_freeze) && ~(or1200_cpu___or1200_operandmuxes___saved_a))) begin
                or1200_cpu___operand_a <= or1200_cpu___muxed_a;
                or1200_cpu___or1200_operandmuxes___saved_a <= 1'h1;
            end else begin
                if ((~(ex_freeze) && ~(or1200_cpu___or1200_operandmuxes___saved_a))) begin
                    or1200_cpu___operand_a <= or1200_cpu___muxed_a;
                end else begin
                    if ((~(ex_freeze) && ~(or1200_cpu___id_freeze))) begin
                        or1200_cpu___or1200_operandmuxes___saved_a <= 1'h0;
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___operand_b <= 32'h0;
            or1200_cpu___or1200_operandmuxes___saved_b <= 1'h0;
        end else begin
            if (((~(ex_freeze) && or1200_cpu___id_freeze) && ~(or1200_cpu___or1200_operandmuxes___saved_b))) begin
                or1200_cpu___operand_b <= or1200_cpu___muxed_b;
                or1200_cpu___or1200_operandmuxes___saved_b <= 1'h1;
            end else begin
                if ((~(ex_freeze) && ~(or1200_cpu___or1200_operandmuxes___saved_b))) begin
                    or1200_cpu___operand_b <= or1200_cpu___muxed_b;
                end else begin
                    if ((~(ex_freeze) && ~(or1200_cpu___id_freeze))) begin
                        or1200_cpu___or1200_operandmuxes___saved_b <= 1'h0;
                    end
                end
            end
        end
    end
    always @(or1200_cpu___rf_dataa or or1200_cpu___sel_a or or1200_cpu___wb_forw or rf_dataw) begin
        case (or1200_cpu___sel_a)
            2'h2: begin
                or1200_cpu___muxed_a = rf_dataw;
            end
            2'h3: begin
                or1200_cpu___muxed_a = or1200_cpu___wb_forw;
            end
            default: begin
                or1200_cpu___muxed_a = or1200_cpu___rf_dataa;
            end
        endcase
    end
    always @(or1200_cpu___id_simm or or1200_cpu___rf_datab or or1200_cpu___sel_b or or1200_cpu___wb_forw or rf_dataw) begin
        case (or1200_cpu___sel_b)
            2'h1: begin
                or1200_cpu___muxed_b = or1200_cpu___id_simm;
            end
            2'h2: begin
                or1200_cpu___muxed_b = rf_dataw;
            end
            2'h3: begin
                or1200_cpu___muxed_b = or1200_cpu___wb_forw;
            end
            default: begin
                or1200_cpu___muxed_b = or1200_cpu___rf_datab;
            end
        endcase
    end
    assign     or1200_cpu___or1200_alu___comp_a = {(or1200_cpu___operand_a[31] ^ or1200_cpu___comp_op[3]), or1200_cpu___operand_a[30:0]};
    assign     or1200_cpu___or1200_alu___comp_b = {(or1200_cpu___operand_b[31] ^ or1200_cpu___comp_op[3]), or1200_cpu___operand_b[30:0]};
    assign     or1200_cpu___or1200_alu___cy_sub = (or1200_cpu___or1200_alu___comp_a < or1200_cpu___or1200_alu___comp_b);
    assign     or1200_cpu___or1200_alu___carry_in = ((5'h1 == or1200_cpu___alu_op)) ? {31'b0,or1200_cpu___carry} : 32'h0;
    assign     or1200_cpu___or1200_alu___b_mux = ((5'h2 == or1200_cpu___alu_op)) ? (32'sh1 + ~(or1200_cpu___operand_b)) : or1200_cpu___operand_b;
    assign ___sel_temp_7 = (({1'b0,or1200_cpu___operand_a} + {1'b0,or1200_cpu___or1200_alu___b_mux}) + {1'b0,or1200_cpu___or1200_alu___carry_in});
    assign     or1200_cpu___or1200_alu___cy_sum = ___sel_temp_7[32];
    assign     or1200_cpu___or1200_alu___result_sum = ((or1200_cpu___operand_a + or1200_cpu___or1200_alu___b_mux) + or1200_cpu___or1200_alu___carry_in);
    assign     or1200_cpu___or1200_alu___ov_sum = ((((~(or1200_cpu___operand_a[31]) & ~(or1200_cpu___or1200_alu___b_mux[31])) & or1200_cpu___or1200_alu___result_sum[31]) | (((~(or1200_cpu___operand_a[31]) & or1200_cpu___or1200_alu___b_mux[31]) & or1200_cpu___or1200_alu___result_sum[31]) & (5'h2 == or1200_cpu___alu_op))) | ((or1200_cpu___operand_a[31] & or1200_cpu___or1200_alu___b_mux[31]) & ~(or1200_cpu___or1200_alu___result_sum[31])));
    assign     or1200_cpu___or1200_alu___result_and = (or1200_cpu___operand_a & or1200_cpu___operand_b);
    always @(or1200_cpu___alu_op or or1200_cpu___alu_op2 or or1200_cpu___carry or or1200_cpu___ex_macrc_op or or1200_cpu___flag or or1200_cpu___mult_mac_result or or1200_cpu___operand_a or or1200_cpu___operand_b or or1200_cpu___or1200_alu___extended or or1200_cpu___or1200_alu___result_and or or1200_cpu___or1200_alu___result_sum or or1200_cpu___or1200_alu___shifted_rotated) begin
        case (or1200_cpu___alu_op)
            5'hf: begin
                case ({28'b0,or1200_cpu___alu_op2})
                    32'sh0: begin
                        or1200_cpu___alu_dataout = (or1200_cpu___operand_a[0]) ? 32'sh1 : (or1200_cpu___operand_a[1] ? 32'sh2 : (or1200_cpu___operand_a[2] ? 32'sh3 : (or1200_cpu___operand_a[3] ? 32'sh4 : (or1200_cpu___operand_a[4] ? 32'sh5 : (or1200_cpu___operand_a[5] ? 32'sh6 : (or1200_cpu___operand_a[6] ? 32'sh7 : (or1200_cpu___operand_a[7] ? 32'sh8 : (or1200_cpu___operand_a[8] ? 32'sh9 : (or1200_cpu___operand_a[9] ? 32'sha : (or1200_cpu___operand_a[10] ? 32'shb : (or1200_cpu___operand_a[11] ? 32'shc : (or1200_cpu___operand_a[12] ? 32'shd : (or1200_cpu___operand_a[13] ? 32'she : (or1200_cpu___operand_a[14] ? 32'shf : (or1200_cpu___operand_a[15] ? 32'sh10 : (or1200_cpu___operand_a[16] ? 32'sh11 : (or1200_cpu___operand_a[17] ? 32'sh12 : (or1200_cpu___operand_a[18] ? 32'sh13 : (or1200_cpu___operand_a[19] ? 32'sh14 : (or1200_cpu___operand_a[20] ? 32'sh15 : (or1200_cpu___operand_a[21] ? 32'sh16 : (or1200_cpu___operand_a[22] ? 32'sh17 : (or1200_cpu___operand_a[23] ? 32'sh18 : (or1200_cpu___operand_a[24] ? 32'sh19 : (or1200_cpu___operand_a[25] ? 32'sh1a : (or1200_cpu___operand_a[26] ? 32'sh1b : (or1200_cpu___operand_a[27] ? 32'sh1c : (or1200_cpu___operand_a[28] ? 32'sh1d : (or1200_cpu___operand_a[29] ? 32'sh1e : (or1200_cpu___operand_a[30] ? 32'sh1f : (or1200_cpu___operand_a[31] ? 32'sh20 : 32'sh0)))))))))))))))))))))))))))))));
                    end
                    default: begin
                        or1200_cpu___alu_dataout = (or1200_cpu___operand_a[31]) ? 32'sh20 : (or1200_cpu___operand_a[30] ? 32'sh1f : (or1200_cpu___operand_a[29] ? 32'sh1e : (or1200_cpu___operand_a[28] ? 32'sh1d : (or1200_cpu___operand_a[27] ? 32'sh1c : (or1200_cpu___operand_a[26] ? 32'sh1b : (or1200_cpu___operand_a[25] ? 32'sh1a : (or1200_cpu___operand_a[24] ? 32'sh19 : (or1200_cpu___operand_a[23] ? 32'sh18 : (or1200_cpu___operand_a[22] ? 32'sh17 : (or1200_cpu___operand_a[21] ? 32'sh16 : (or1200_cpu___operand_a[20] ? 32'sh15 : (or1200_cpu___operand_a[19] ? 32'sh14 : (or1200_cpu___operand_a[18] ? 32'sh13 : (or1200_cpu___operand_a[17] ? 32'sh12 : (or1200_cpu___operand_a[16] ? 32'sh11 : (or1200_cpu___operand_a[15] ? 32'sh10 : (or1200_cpu___operand_a[14] ? 32'shf : (or1200_cpu___operand_a[13] ? 32'she : (or1200_cpu___operand_a[12] ? 32'shd : (or1200_cpu___operand_a[11] ? 32'shc : (or1200_cpu___operand_a[10] ? 32'shb : (or1200_cpu___operand_a[9] ? 32'sha : (or1200_cpu___operand_a[8] ? 32'sh9 : (or1200_cpu___operand_a[7] ? 32'sh8 : (or1200_cpu___operand_a[6] ? 32'sh7 : (or1200_cpu___operand_a[5] ? 32'sh6 : (or1200_cpu___operand_a[4] ? 32'sh5 : (or1200_cpu___operand_a[3] ? 32'sh4 : (or1200_cpu___operand_a[2] ? 32'sh3 : (or1200_cpu___operand_a[1] ? 32'sh2 : (or1200_cpu___operand_a[0] ? 32'sh1 : 32'sh0)))))))))))))))))))))))))))))));
                    end
                endcase
            end
            5'h8: begin
                or1200_cpu___alu_dataout = or1200_cpu___or1200_alu___shifted_rotated;
            end
            5'h1, 5'h2, 5'h0: begin
                or1200_cpu___alu_dataout = or1200_cpu___or1200_alu___result_sum;
            end
            5'h5: begin
                or1200_cpu___alu_dataout = (or1200_cpu___operand_a ^ or1200_cpu___operand_b);
            end
            5'h4: begin
                or1200_cpu___alu_dataout = (or1200_cpu___operand_a | or1200_cpu___operand_b);
            end
            5'hc: begin
                or1200_cpu___alu_dataout = or1200_cpu___or1200_alu___extended;
            end
            5'hd: begin
                or1200_cpu___alu_dataout = or1200_cpu___or1200_alu___extended;
            end
            5'h11: begin
                or1200_cpu___alu_dataout = (or1200_cpu___ex_macrc_op) ? or1200_cpu___mult_mac_result : (or1200_cpu___operand_b << 32'sh10);
            end
            5'h9, 5'ha, 5'h6, 5'hb: begin
                or1200_cpu___alu_dataout = or1200_cpu___mult_mac_result;
            end
            5'he: begin
                or1200_cpu___alu_dataout = (or1200_cpu___flag) ? or1200_cpu___operand_a : or1200_cpu___operand_b;
            end
            default: begin
                or1200_cpu___alu_dataout = or1200_cpu___or1200_alu___result_and;
            end
        endcase
    end
    always @(or1200_cpu___alu_op or or1200_cpu___or1200_alu___flagcomp or or1200_cpu___or1200_alu___result_and or or1200_cpu___or1200_alu___result_sum) begin
        case (or1200_cpu___alu_op)
            5'h10: begin
                or1200_cpu___flagforw_alu = or1200_cpu___or1200_alu___flagcomp;
                or1200_cpu___flag_we_alu = 1'h1;
            end
            default: begin
                or1200_cpu___flagforw_alu = or1200_cpu___or1200_alu___flagcomp;
                or1200_cpu___flag_we_alu = 1'h0;
            end
        endcase
    end
    always @(or1200_cpu___alu_op or or1200_cpu___or1200_alu___cy_sub or or1200_cpu___or1200_alu___cy_sum) begin
        case (or1200_cpu___alu_op)
            5'h1, 5'h0: begin
                or1200_cpu___cyforw = or1200_cpu___or1200_alu___cy_sum;
                or1200_cpu___cy_we_alu = 1'h1;
            end
            5'h2: begin
                or1200_cpu___cyforw = or1200_cpu___or1200_alu___cy_sub;
                or1200_cpu___cy_we_alu = 1'h1;
            end
            default: begin
                or1200_cpu___cyforw = 1'h0;
                or1200_cpu___cy_we_alu = 1'h0;
            end
        endcase
    end
    always @(or1200_cpu___alu_op or or1200_cpu___or1200_alu___ov_sum) begin
        case (or1200_cpu___alu_op)
            5'h1, 5'h2, 5'h0: begin
                or1200_cpu___ovforw = or1200_cpu___or1200_alu___ov_sum;
                or1200_cpu___ov_we_alu = 1'h1;
            end
            default: begin
                or1200_cpu___ovforw = 1'h0;
                or1200_cpu___ov_we_alu = 1'h0;
            end
        endcase
    end
    always @(or1200_cpu___alu_op2 or or1200_cpu___operand_a or or1200_cpu___operand_b) begin
        case (or1200_cpu___alu_op2)
            4'h0: begin
                or1200_cpu___or1200_alu___shifted_rotated = (or1200_cpu___operand_a << or1200_cpu___operand_b[4:0]);
            end
            4'h1: begin
                or1200_cpu___or1200_alu___shifted_rotated = (or1200_cpu___operand_a >> or1200_cpu___operand_b[4:0]);
            end
            default: begin
                ___sel_temp_8 = or1200_cpu___operand_b[4:0];
                or1200_cpu___or1200_alu___shifted_rotated = (({32'sh20{or1200_cpu___operand_a[31]}} << (6'h20 - {1'b0,___sel_temp_8})) | (or1200_cpu___operand_a >> or1200_cpu___operand_b[4:0]));
            end
        endcase
    end
    always @(or1200_cpu___comp_op or or1200_cpu___or1200_alu___comp_a or or1200_cpu___or1200_alu___comp_b) begin
        case (or1200_cpu___comp_op[2:0])
            3'h0: begin
                or1200_cpu___or1200_alu___flagcomp = (or1200_cpu___or1200_alu___comp_a == or1200_cpu___or1200_alu___comp_b);
            end
            3'h1: begin
                or1200_cpu___or1200_alu___flagcomp = (or1200_cpu___or1200_alu___comp_a != or1200_cpu___or1200_alu___comp_b);
            end
            3'h2: begin
                or1200_cpu___or1200_alu___flagcomp = (or1200_cpu___or1200_alu___comp_a > or1200_cpu___or1200_alu___comp_b);
            end
            3'h3: begin
                or1200_cpu___or1200_alu___flagcomp = (or1200_cpu___or1200_alu___comp_a >= or1200_cpu___or1200_alu___comp_b);
            end
            3'h4: begin
                or1200_cpu___or1200_alu___flagcomp = (or1200_cpu___or1200_alu___comp_a < or1200_cpu___or1200_alu___comp_b);
            end
            3'h5: begin
                or1200_cpu___or1200_alu___flagcomp = (or1200_cpu___or1200_alu___comp_a <= or1200_cpu___or1200_alu___comp_b);
            end
            default: begin
                or1200_cpu___or1200_alu___flagcomp = 1'h0;
            end
        endcase
    end
    always @(or1200_cpu___alu_op or or1200_cpu___alu_op2 or or1200_cpu___operand_a) begin
        case (or1200_cpu___alu_op2)
            4'h0: begin
                or1200_cpu___or1200_alu___extended = {{32'sh10{or1200_cpu___operand_a[15]}}, or1200_cpu___operand_a[15:0]};
            end
            4'h1: begin
                or1200_cpu___or1200_alu___extended = {{32'sh18{or1200_cpu___operand_a[7]}}, or1200_cpu___operand_a[7:0]};
            end
            4'h2: begin
                ___sel_temp_9 = or1200_cpu___operand_a[15:0];
                or1200_cpu___or1200_alu___extended = {16'b0,___sel_temp_9};
            end
            4'h3: begin
                ___sel_temp_10 = or1200_cpu___operand_a[7:0];
                or1200_cpu___or1200_alu___extended = {24'b0,___sel_temp_10};
            end
            default: begin
                or1200_cpu___or1200_alu___extended = or1200_cpu___operand_a;
            end
        endcase
    end
    assign     or1200_cpu___or1200_mult_mac___alu_op_smul = (5'h6 == or1200_cpu___alu_op);
    assign     or1200_cpu___or1200_mult_mac___alu_op_umul = (5'hb == or1200_cpu___alu_op);
    assign     or1200_cpu___or1200_mult_mac___alu_op_mul = (or1200_cpu___or1200_mult_mac___alu_op_smul | or1200_cpu___or1200_mult_mac___alu_op_umul);
    assign     or1200_cpu___or1200_mult_mac___spr_maclo_we = ((or1200_cpu_____Vcellinp__or1200_mult_mac__spr_cs & spr_we) & spr_addr[0]);
    assign     or1200_cpu___or1200_mult_mac___spr_machi_we = ((or1200_cpu_____Vcellinp__or1200_mult_mac__spr_cs & spr_we) & ~(spr_addr[0]));
    assign     or1200_cpu___spr_dat_mac = (spr_addr[0]) ? or1200_cpu___or1200_mult_mac___mac_r[31:0] : or1200_cpu___or1200_mult_mac___mac_r[63:32];
    assign     or1200_cpu___or1200_mult_mac___alu_op_sdiv = (5'h9 == or1200_cpu___alu_op);
    assign     or1200_cpu___or1200_mult_mac___alu_op_udiv = (5'ha == or1200_cpu___alu_op);
    assign     or1200_cpu___or1200_mult_mac___alu_op_div = (or1200_cpu___or1200_mult_mac___alu_op_sdiv | or1200_cpu___or1200_mult_mac___alu_op_udiv);
    assign     or1200_cpu___or1200_mult_mac___x = (((or1200_cpu___or1200_mult_mac___alu_op_sdiv | or1200_cpu___or1200_mult_mac___alu_op_smul) & or1200_cpu___operand_a[31])) ? (32'h1 + ~(or1200_cpu___operand_a)) : (((or1200_cpu___or1200_mult_mac___alu_op_div | or1200_cpu___or1200_mult_mac___alu_op_mul) | |(or1200_cpu___mac_op)) ? or1200_cpu___operand_a : 32'h0);
    assign     or1200_cpu___or1200_mult_mac___y = (((or1200_cpu___or1200_mult_mac___alu_op_sdiv | or1200_cpu___or1200_mult_mac___alu_op_smul) & or1200_cpu___operand_b[31])) ? (32'h1 + ~(or1200_cpu___operand_b)) : (((or1200_cpu___or1200_mult_mac___alu_op_div | or1200_cpu___or1200_mult_mac___alu_op_mul) | |(or1200_cpu___mac_op)) ? or1200_cpu___operand_b : 32'h0);
    assign     or1200_cpu___or1200_mult_mac___div_by_zero = (~(|(or1200_cpu___operand_b)) & or1200_cpu___or1200_mult_mac___alu_op_div);
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___ex_freeze_r <= (rst || ex_freeze);
    end
    always @(*) begin
        case (or1200_cpu___alu_op)
            5'h9: begin
                or1200_cpu___mult_mac_result = ((or1200_cpu___operand_a[31] ^ or1200_cpu___operand_b[31])) ? (32'h1 + ~(or1200_cpu___or1200_mult_mac___div_quot_r[31:0])) : or1200_cpu___or1200_mult_mac___div_quot_r[31:0];
            end
            5'ha: begin
                or1200_cpu___mult_mac_result = or1200_cpu___or1200_mult_mac___div_quot_r[31:0];
            end
            5'h6: begin
                or1200_cpu___mult_mac_result = ((or1200_cpu___operand_a[31] ^ or1200_cpu___operand_b[31])) ? (32'h1 + ~(or1200_cpu___or1200_mult_mac___mul_prod_r[31:0])) : or1200_cpu___or1200_mult_mac___mul_prod_r[31:0];
            end
            5'hb: begin
                or1200_cpu___mult_mac_result = or1200_cpu___or1200_mult_mac___mul_prod_r[31:0];
            end
            default: begin
                or1200_cpu___mult_mac_result = or1200_cpu___or1200_mult_mac___mac_r[31:0];
            end
        endcase
    end
    always @(*) begin
        case (or1200_cpu___alu_op)
            5'h6: begin
                or1200_cpu___ovforw_mult_mac = ((or1200_cpu___or1200_mult_mac___mul_prod_r[31] && ~(((or1200_cpu___operand_a[31] ^ or1200_cpu___operand_b[31]) && ~(|(or1200_cpu___or1200_mult_mac___mul_prod_r[30:0]))))) || |(or1200_cpu___or1200_mult_mac___mul_prod_r[63:32]));
                or1200_cpu___ov_we_mult_mac = 1'h1;
            end
            5'hb: begin
                or1200_cpu___ovforw_mult_mac = |(or1200_cpu___or1200_mult_mac___mul_prod_r[63:32]);
                or1200_cpu___ov_we_mult_mac = 1'h1;
            end
            5'ha, 5'h9: begin
                or1200_cpu___ovforw_mult_mac = (or1200_cpu___or1200_mult_mac___div_by_zero || ((32'h80000000 == or1200_cpu___operand_a) && (32'hffffffff == or1200_cpu___operand_b)));
                or1200_cpu___ov_we_mult_mac = 1'h1;
            end
            default: begin
                or1200_cpu___ovforw_mult_mac = 1'h0;
                or1200_cpu___ov_we_mult_mac = 1'h0;
            end
        endcase
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___mul_prod_r <= (rst) ? 64'h0 : or1200_cpu___or1200_mult_mac___mul_prod;
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___mul_stall_count <= (rst) ? 2'h0 : (|(or1200_cpu___or1200_mult_mac___mul_stall_count) ? {or1200_cpu___or1200_mult_mac___mul_stall_count[0], 1'h0} : {or1200_cpu___or1200_mult_mac___mul_stall_count[0], (or1200_cpu___or1200_mult_mac___alu_op_mul & ~(or1200_cpu___or1200_mult_mac___ex_freeze_r))});
    end
    assign     or1200_cpu___or1200_mult_mac___mul_stall = (|(or1200_cpu___or1200_mult_mac___mul_stall_count) | ((~(|(or1200_cpu___or1200_mult_mac___mul_stall_count)) & or1200_cpu___or1200_mult_mac___alu_op_mul) & ~(or1200_cpu___or1200_mult_mac___ex_freeze_r)));
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___mac_op_r1 <= (rst) ? 3'h0 : (or1200_cpu___or1200_mult_mac___ex_freeze_r ? 3'h0 : or1200_cpu___mac_op);
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___mac_op_r2 <= (rst) ? 3'h0 : or1200_cpu___or1200_mult_mac___mac_op_r1;
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___mac_op_r3 <= (rst) ? 3'h0 : or1200_cpu___or1200_mult_mac___mac_op_r2;
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_mult_mac___mac_r <= 64'h0;
        end else begin
            if (or1200_cpu___or1200_mult_mac___spr_maclo_we) begin
                or1200_cpu___or1200_mult_mac___mac_r[31:0] <= spr_dat_cpu;
            end else begin
                if (or1200_cpu___or1200_mult_mac___spr_machi_we) begin
                    or1200_cpu___or1200_mult_mac___mac_r[63:32] <= spr_dat_cpu;
                end else begin
                    if ((3'h1 == or1200_cpu___or1200_mult_mac___mac_op_r3)) begin
                        or1200_cpu___or1200_mult_mac___mac_r <= (or1200_cpu___or1200_mult_mac___mac_r + or1200_cpu___or1200_mult_mac___mul_prod_r);
                    end else begin
                        if ((3'h2 == or1200_cpu___or1200_mult_mac___mac_op_r3)) begin
                            or1200_cpu___or1200_mult_mac___mac_r <= (or1200_cpu___or1200_mult_mac___mac_r - or1200_cpu___or1200_mult_mac___mul_prod_r);
                        end else begin
                            if ((or1200_cpu___ex_macrc_op && ~(ex_freeze))) begin
                                or1200_cpu___or1200_mult_mac___mac_r <= 64'h0;
                            end
                        end
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___mac_stall_r <= (~(rst) && (((|(or1200_cpu___mac_op) | |(or1200_cpu___or1200_mult_mac___mac_op_r1)) | |(or1200_cpu___or1200_mult_mac___mac_op_r2)) & (or1200_cpu___id_macrc_op | or1200_cpu___or1200_mult_mac___mac_stall_r)));
    end
    assign     or1200_cpu___or1200_mult_mac___div_tmp = (or1200_cpu___or1200_mult_mac___div_quot_r[63:32] - or1200_cpu___or1200_mult_mac___y);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_mult_mac___div_quot_r <= 64'h0;
            or1200_cpu___or1200_mult_mac___div_free <= 1'h1;
            or1200_cpu___or1200_mult_mac___div_cntr <= 6'h0;
        end else begin
            if (or1200_cpu___or1200_mult_mac___div_by_zero) begin
                or1200_cpu___or1200_mult_mac___div_quot_r <= 64'h0;
                or1200_cpu___or1200_mult_mac___div_free <= 1'h1;
                or1200_cpu___or1200_mult_mac___div_cntr <= 6'h0;
            end else begin
                if (|(or1200_cpu___or1200_mult_mac___div_cntr)) begin
                    or1200_cpu___or1200_mult_mac___div_quot_r <= (or1200_cpu___or1200_mult_mac___div_tmp[31]) ? {or1200_cpu___or1200_mult_mac___div_quot_r[62:0], 1'h0} : {or1200_cpu___or1200_mult_mac___div_tmp[30:0], {or1200_cpu___or1200_mult_mac___div_quot_r[31:0], 1'h1}};
                    or1200_cpu___or1200_mult_mac___div_cntr <= (or1200_cpu___or1200_mult_mac___div_cntr - 6'h1);
                end else begin
                    if ((or1200_cpu___or1200_mult_mac___alu_op_div && or1200_cpu___or1200_mult_mac___div_free)) begin
                        or1200_cpu___or1200_mult_mac___div_quot_r <= {{31'b0,or1200_cpu___or1200_mult_mac___x}, 1'h0};
                        or1200_cpu___or1200_mult_mac___div_cntr <= 6'h20;
                        or1200_cpu___or1200_mult_mac___div_free <= 1'h0;
                    end else begin
                        if ((or1200_cpu___or1200_mult_mac___div_free | ~(ex_freeze))) begin
                            or1200_cpu___or1200_mult_mac___div_free <= 1'h1;
                        end
                    end
                end
            end
        end
    end
    assign     or1200_cpu___or1200_mult_mac___div_stall = (|(or1200_cpu___or1200_mult_mac___div_cntr) | (~(or1200_cpu___or1200_mult_mac___ex_freeze_r) & or1200_cpu___or1200_mult_mac___alu_op_div));
    assign     or1200_cpu___mult_mac_stall = ((or1200_cpu___or1200_mult_mac___mac_stall_r | or1200_cpu___or1200_mult_mac___div_stall) | or1200_cpu___or1200_mult_mac___mul_stall);
    always @(or1200_cpu___or1200_mult_mac___x) begin
        or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___xi = or1200_cpu___or1200_mult_mac___x;
    end
    always @(or1200_cpu___or1200_mult_mac___y) begin
        or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___yi = or1200_cpu___or1200_mult_mac___y;
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___p0 <= (rst) ? 64'h0 : $signed($signed({{32{or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___xi[31]}},or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___xi}) * $signed({{32{or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___yi[31]}},or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___yi}));
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___p1 <= (rst) ? 64'h0 : or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___p0;
    end
    assign     or1200_cpu___or1200_mult_mac___mul_prod = or1200_cpu___or1200_mult_mac___or1200_gmultp2_32x32___p1;
    assign     or1200_cpu___or1200_sprs___epcr = or1200_cpu___epcr;
    assign     or1200_cpu___or1200_sprs___eear = or1200_cpu___eear;
    assign     or1200_cpu___or1200_sprs___esr = or1200_cpu___esr;
    assign     or1200_cpu___sr = or1200_cpu___or1200_sprs___sr;
    assign     or1200_cpu___sp_address = or1200_cpu___or1200_sprs___sp_reg1;
    assign     or1200_cpu___sp_data = or1200_cpu___or1200_sprs___sp_reg2;
    assign     or1200_cpu___sp_strobe = or1200_cpu___or1200_sprs___sp_reg3;
    assign     or1200_cpu___sp_attack_enable = or1200_cpu___or1200_sprs___sp_reg6;
    assign     or1200_cpu___or1200_sprs___du_access = (du_read | du_write);
    assign     spr_addr = (or1200_cpu___or1200_sprs___du_access) ? du_addr : (or1200_cpu___operand_a | {16'b0,or1200_cpu_____Vcellinp__or1200_sprs__addrofs});
    assign     spr_dat_cpu = (du_write) ? du_dat_du : or1200_cpu___operand_b;
    assign     du_dat_cpu = (du_read) ? or1200_cpu___sprs_dataout : (du_write ? du_dat_du : or1200_cpu___operand_b);
    assign     spr_we = (du_write | (or1200_cpu___ex_spr_write & ~(or1200_cpu___or1200_sprs___du_access)));
    assign     spr_cs = (or1200_cpu___or1200_sprs___unqualified_cs & {32'sh20{(((du_read | du_write) | or1200_cpu___ex_spr_read) | (or1200_cpu___ex_spr_write & ((or1200_cpu___or1200_sprs___sr[0] | or1200_cpu___sp_attack_enable[0]) | (5'h7 == spr_addr[15:11]))))}});
    always @(spr_addr) begin
        case (spr_addr[15:11])
            5'h0: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h1;
            end
            5'h1: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h2;
            end
            5'h2: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h4;
            end
            5'h3: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h8;
            end
            5'h4: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h10;
            end
            5'h5: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h20;
            end
            5'h6: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h40;
            end
            5'h7: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h80;
            end
            5'h8: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h100;
            end
            5'h9: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h200;
            end
            5'ha: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h400;
            end
            5'hb: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h800;
            end
            5'hc: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h1000;
            end
            5'hd: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h2000;
            end
            5'he: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h4000;
            end
            5'hf: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h8000;
            end
            5'h10: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h10000;
            end
            5'h11: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h20000;
            end
            5'h12: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h40000;
            end
            5'h13: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h80000;
            end
            5'h14: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h100000;
            end
            5'h15: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h200000;
            end
            5'h16: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h400000;
            end
            5'h17: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h800000;
            end
            5'h18: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h1000000;
            end
            5'h19: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h2000000;
            end
            5'h1a: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h4000000;
            end
            5'h1b: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h8000000;
            end
            5'h1c: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h10000000;
            end
            5'h1d: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h20000000;
            end
            5'h1e: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h40000000;
            end
            5'h1f: begin
                or1200_cpu___or1200_sprs___unqualified_cs = 32'h80000000;
            end
        endcase
    end
    assign ___sel_temp_11 = {or1200_cpu___or1200_sprs___sr[15:14], 1'h0};
    assign     or1200_cpu___to_sr[15:12] = (or1200_cpu___except_started) ? {1'b0,___sel_temp_11} : ((3'h6 == branch_op) ? ((8'h8 == ex_pc[31:24]) ? or1200_cpu___sp_esr_ghost[15:12] : or1200_cpu___or1200_sprs___esr[15:12]) : ((spr_we && or1200_cpu___or1200_sprs___sr_sel) ? {1'h1, spr_dat_cpu[14:12]} : or1200_cpu___or1200_sprs___sr[15:12]));
    assign     or1200_cpu___to_sr[16] = (or1200_cpu___except_started || ((3'h6 == branch_op) ? ((8'h8 == ex_pc[31:24]) ? or1200_cpu___sp_esr_ghost[16] : or1200_cpu___or1200_sprs___esr[16]) : ((spr_we && or1200_cpu___or1200_sprs___sr_sel) ? spr_dat_cpu[16] : or1200_cpu___or1200_sprs___sr[16])));
    assign     or1200_cpu___to_sr[11] = (or1200_cpu___except_started) ? or1200_cpu___or1200_sprs___sr[11] : ((3'h6 == branch_op) ? ((8'h8 == ex_pc[31:24]) ? or1200_cpu___sp_esr_ghost[11] : or1200_cpu___or1200_sprs___esr[11]) : (or1200_cpu_____Vcellinp__or1200_sprs__ov_we ? or1200_cpu_____Vcellinp__or1200_sprs__ovforw : ((spr_we && or1200_cpu___or1200_sprs___sr_sel) ? spr_dat_cpu[11] : or1200_cpu___or1200_sprs___sr[11])));
    assign     or1200_cpu___to_sr[10] = (or1200_cpu___except_started) ? or1200_cpu___or1200_sprs___sr[10] : ((3'h6 == branch_op) ? ((8'h8 == ex_pc[31:24]) ? or1200_cpu___sp_esr_ghost[10] : or1200_cpu___or1200_sprs___esr[10]) : (or1200_cpu___cy_we_rf ? or1200_cpu___cyforw : ((spr_we && or1200_cpu___or1200_sprs___sr_sel) ? spr_dat_cpu[10] : or1200_cpu___or1200_sprs___sr[10])));
    assign     or1200_cpu___to_sr[9] = (or1200_cpu___except_started) ? or1200_cpu___or1200_sprs___sr[9] : ((3'h6 == branch_op) ? ((8'h8 == ex_pc[31:24]) ? or1200_cpu___sp_esr_ghost[9] : or1200_cpu___or1200_sprs___esr[9]) : (or1200_cpu___flag_we ? or1200_cpu___flagforw : ((spr_we && or1200_cpu___or1200_sprs___sr_sel) ? spr_dat_cpu[9] : or1200_cpu___or1200_sprs___sr[9])));
    assign ___sel_temp_13 = or1200_cpu___sp_attack_enable[2];
    assign ___sel_temp_12 = {or1200_cpu___or1200_sprs___sr[4:3], 3'h1};
    assign     or1200_cpu___to_sr[8:0] = (or1200_cpu___except_started) ? {or1200_cpu___or1200_sprs___sr[8:7], {2'b0,___sel_temp_12}} : ((3'h6 == branch_op) ? ((8'h8 == ex_pc[31:24]) ? or1200_cpu___sp_esr_ghost[8:0] : (or1200_cpu___or1200_sprs___esr[8:0] | {8'b0,___sel_temp_13})) : ((spr_we && or1200_cpu___or1200_sprs___sr_sel) ? spr_dat_cpu[8:0] : or1200_cpu___or1200_sprs___sr[8:0]));
    assign     or1200_cpu___or1200_sprs___cfgr_sel = (spr_cs[0] && (7'h0 == spr_addr[10:4]));
    assign     or1200_cpu___or1200_sprs___rf_sel = (spr_cs[0] && (6'h20 == spr_addr[10:5]));
    assign     or1200_cpu___or1200_sprs___npc_sel = (spr_cs[0] && (11'h10 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___ppc_sel = (spr_cs[0] && (11'h12 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sr_sel = (spr_cs[0] && (11'h11 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___epcr_sel = (spr_cs[0] && (11'h20 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_epcr_ghost_sel = (spr_cs[0] && (11'h21 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___eear_sel = (spr_cs[0] && (11'h30 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_eear_ghost_sel = (spr_cs[0] && (11'h31 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___esr_sel = (spr_cs[0] && (11'h40 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_esr_ghost_sel = (spr_cs[0] && (11'h41 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___fpcsr_sel = (spr_cs[0] && (11'h14 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg0_sel = (spr_cs[7] && (11'h0 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg4_sel = (spr_cs[7] && (11'h4 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg1_sel = (spr_cs[7] && (11'h1 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg2_sel = (spr_cs[7] && (11'h2 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg3_sel = (spr_cs[7] && (11'h3 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg5_sel = (spr_cs[7] && (11'h5 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg6_sel = (spr_cs[7] && (11'h6 == spr_addr[10:0]));
    assign     or1200_cpu___or1200_sprs___sp_reg7_sel = (spr_cs[7] && (11'h7 == spr_addr[10:0]));
    assign     or1200_cpu___sr_we = (((((spr_we && or1200_cpu___or1200_sprs___sr_sel) | (((~(or1200_cpu_____Vcellinp__or1200_sprs__except_illegal) & ~(or1200_cpu___sp_assertion_violated)) & ~(ex_void)) & (3'h6 == branch_op))) | or1200_cpu___flag_we) | or1200_cpu___cy_we_rf) | or1200_cpu_____Vcellinp__or1200_sprs__ov_we);
    assign     or1200_cpu___pc_we = (du_write && (or1200_cpu___or1200_sprs___npc_sel | or1200_cpu___or1200_sprs___ppc_sel));
    assign     or1200_cpu___epcr_we = (spr_we && or1200_cpu___or1200_sprs___epcr_sel);
    assign     or1200_cpu___eear_we = (spr_we && or1200_cpu___or1200_sprs___eear_sel);
    assign     or1200_cpu___esr_we = (spr_we && or1200_cpu___or1200_sprs___esr_sel);
    assign     or1200_cpu___fpcsr_we = (spr_we && or1200_cpu___or1200_sprs___fpcsr_sel);
    assign     or1200_cpu___sp_epcr_ghost_we = (spr_we && or1200_cpu___or1200_sprs___sp_epcr_ghost_sel);
    assign     or1200_cpu___sp_eear_ghost_we = (spr_we && or1200_cpu___or1200_sprs___sp_eear_ghost_sel);
    assign     or1200_cpu___sp_esr_ghost_we = (spr_we && or1200_cpu___or1200_sprs___sp_esr_ghost_sel);
    assign     or1200_cpu___or1200_sprs___sp_reg0_we = (spr_we & or1200_cpu___or1200_sprs___sp_reg0_sel);
    assign     or1200_cpu___or1200_sprs___sp_reg4_we = (spr_we & or1200_cpu___or1200_sprs___sp_reg4_sel);
    assign     or1200_cpu___or1200_sprs___sp_reg1_we = (spr_we & or1200_cpu___or1200_sprs___sp_reg1_sel);
    assign     or1200_cpu___or1200_sprs___sp_reg2_we = (spr_we & or1200_cpu___or1200_sprs___sp_reg2_sel);
    assign     or1200_cpu___or1200_sprs___sp_reg3_we = (spr_we & or1200_cpu___or1200_sprs___sp_reg3_sel);
    assign     or1200_cpu___or1200_sprs___sp_reg6_we = (spr_we & or1200_cpu___or1200_sprs___sp_reg6_sel);
    assign     or1200_cpu___or1200_sprs___sys_data = ((((((((((((((((((((or1200_cpu___spr_dat_cfgr & {32'sh20{or1200_cpu___or1200_sprs___cfgr_sel}}) | ({15'b0,or1200_cpu___sp_esr_ghost} & {32'sh20{or1200_cpu___or1200_sprs___sp_esr_ghost_sel}})) | (or1200_cpu___sp_epcr_ghost & {32'sh20{or1200_cpu___or1200_sprs___sp_epcr_ghost_sel}})) | (or1200_cpu___sp_eear_ghost & {32'sh20{or1200_cpu___or1200_sprs___sp_eear_ghost_sel}})) | (or1200_cpu___or1200_sprs___sp_reg0 & {32'sh20{or1200_cpu___or1200_sprs___sp_reg0_sel}})) | (or1200_cpu___or1200_sprs___sp_reg4 & {32'sh20{or1200_cpu___or1200_sprs___sp_reg4_sel}})) | (or1200_cpu___or1200_sprs___sp_reg1 & {32'sh20{or1200_cpu___or1200_sprs___sp_reg1_sel}})) | (or1200_cpu___or1200_sprs___sp_reg2 & {32'sh20{or1200_cpu___or1200_sprs___sp_reg2_sel}})) | (or1200_cpu___or1200_sprs___sp_reg3 & {32'sh20{or1200_cpu___or1200_sprs___sp_reg3_sel}})) | (or1200_cpu___sp_assertions_violated_reg & {32'sh20{or1200_cpu___or1200_sprs___sp_reg5_sel}})) | (or1200_cpu___or1200_sprs___sp_reg6 & {32'sh20{or1200_cpu___or1200_sprs___sp_reg6_sel}})) | (or1200_cpu___or1200_sprs___sp_reg7 & {32'sh20{or1200_cpu___or1200_sprs___sp_reg7_sel}})) | (or1200_cpu___spr_dat_rf & {32'sh20{or1200_cpu___or1200_sprs___rf_sel}})) | (spr_dat_npc & {32'sh20{or1200_cpu___or1200_sprs___npc_sel}})) | (or1200_cpu___spr_dat_ppc & {32'sh20{or1200_cpu___or1200_sprs___ppc_sel}})) | ({15'b0,or1200_cpu___or1200_sprs___sr} & {32'sh20{or1200_cpu___or1200_sprs___sr_sel}})) | (or1200_cpu___or1200_sprs___epcr & {32'sh20{or1200_cpu___or1200_sprs___epcr_sel}})) | (or1200_cpu___or1200_sprs___eear & {32'sh20{or1200_cpu___or1200_sprs___eear_sel}})) | ({20'b0,or1200_cpu___fpcsr} & {32'sh20{or1200_cpu___or1200_sprs___fpcsr_sel}})) | ({15'b0,or1200_cpu___or1200_sprs___esr} & {32'sh20{or1200_cpu___or1200_sprs___esr_sel}}));
    assign     or1200_cpu___flag = or1200_cpu___or1200_sprs___sr[9];
    assign     or1200_cpu___carry = or1200_cpu___or1200_sprs___sr[10];
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sp_reg7 <= 32'sh0;
        end else begin
            if (or1200_cpu___sp_assertion_violated) begin
                or1200_cpu___or1200_sprs___sp_reg7 <= or1200_cpu___spr_dat_ppc;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sp_reg0 <= 32'sh0;
        end else begin
            if (or1200_cpu___or1200_sprs___sp_reg0_we) begin
                or1200_cpu___or1200_sprs___sp_reg0 <= spr_dat_cpu;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sp_reg4 <= 32'sh0;
        end else begin
            if (or1200_cpu___or1200_sprs___sp_reg4_we) begin
                or1200_cpu___or1200_sprs___sp_reg4 <= spr_dat_cpu;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sp_reg1 <= 32'h7d0;
        end else begin
            if (or1200_cpu___or1200_sprs___sp_reg1_we) begin
                or1200_cpu___or1200_sprs___sp_reg1 <= spr_dat_cpu;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sp_reg2 <= 32'sh0;
        end else begin
            if (or1200_cpu___or1200_sprs___sp_reg2_we) begin
                or1200_cpu___or1200_sprs___sp_reg2 <= spr_dat_cpu;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sp_reg3 <= 32'sh0;
        end else begin
            if (or1200_cpu___or1200_sprs___sp_reg3_we) begin
                or1200_cpu___or1200_sprs___sp_reg3 <= spr_dat_cpu;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sp_reg6 <= 32'sh0;
        end else begin
            if (or1200_cpu___or1200_sprs___sp_reg6_we) begin
                or1200_cpu___or1200_sprs___sp_reg6 <= spr_dat_cpu;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sr_reg <= 17'h8001;
            or1200_cpu___or1200_sprs___sp_already_attacked <= 1'h0;
        end else begin
            if (or1200_cpu___except_started) begin
                or1200_cpu___or1200_sprs___sr_reg <= or1200_cpu___to_sr;
            end else begin
                if (or1200_cpu___sr_we) begin
                    or1200_cpu___or1200_sprs___sr_reg <= or1200_cpu___to_sr;
                end else begin
                    if ((((or1200_cpu___sp_attack_enable[3] & or1200_cpu___sp_insn_is_exthz) & ~(or1200_cpu___id_freeze)) & ~(or1200_cpu___or1200_sprs___sp_already_attacked))) begin
                        or1200_cpu___or1200_sprs___sr_reg <= or1200_cpu___spr_dat_rf[16:0];
                        or1200_cpu___or1200_sprs___sp_already_attacked <= 1'h1;
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_sprs___sr_reg_bit_eph <= 1'h0;
            or1200_cpu___or1200_sprs___sr_reg_bit_eph_select <= 1'h1;
        end else begin
            if (or1200_cpu___or1200_sprs___sr_reg_bit_eph_select) begin
                or1200_cpu___or1200_sprs___sr_reg_bit_eph <= boot_adr_sel_i;
                or1200_cpu___or1200_sprs___sr_reg_bit_eph_select <= 1'h0;
            end else begin
                if (or1200_cpu___sr_we) begin
                    or1200_cpu___or1200_sprs___sr_reg_bit_eph <= or1200_cpu___to_sr[14];
                end
            end
        end
    end
    assign     or1200_cpu___or1200_sprs___sr_reg_bit_eph_muxed = (or1200_cpu___or1200_sprs___sr_reg_bit_eph_select) ? boot_adr_sel_i : or1200_cpu___or1200_sprs___sr_reg_bit_eph;
    always @(or1200_cpu___or1200_sprs___sp_reg3 or or1200_cpu___or1200_sprs___sr_reg or or1200_cpu___or1200_sprs___sr_reg_bit_eph_muxed) begin
        or1200_cpu___or1200_sprs___sr = {or1200_cpu___or1200_sprs___sr_reg[16:15], {or1200_cpu___or1200_sprs___sr_reg_bit_eph_muxed, or1200_cpu___or1200_sprs___sr_reg[13:0]}};
    end
    always @(or1200_cpu___or1200_sprs___sys_data or or1200_cpu___spr_dat_fpu or or1200_cpu___spr_dat_mac or spr_addr or spr_dat_dmmu or spr_dat_du or spr_dat_immu or spr_dat_pic or spr_dat_pm or spr_dat_tt) begin
        case (spr_addr[15:11])
            5'h7: begin
                or1200_cpu___sprs_dataout = or1200_cpu___or1200_sprs___sys_data;
            end
            5'h0: begin
                or1200_cpu___sprs_dataout = or1200_cpu___or1200_sprs___sys_data;
            end
            5'ha: begin
                or1200_cpu___sprs_dataout = spr_dat_tt;
            end
            5'h9: begin
                or1200_cpu___sprs_dataout = spr_dat_pic;
            end
            5'h8: begin
                or1200_cpu___sprs_dataout = spr_dat_pm;
            end
            5'h1: begin
                or1200_cpu___sprs_dataout = spr_dat_dmmu;
            end
            5'h2: begin
                or1200_cpu___sprs_dataout = spr_dat_immu;
            end
            5'h5: begin
                or1200_cpu___sprs_dataout = or1200_cpu___spr_dat_mac;
            end
            5'hb: begin
                or1200_cpu___sprs_dataout = or1200_cpu___spr_dat_fpu;
            end
            default: begin
                or1200_cpu___sprs_dataout = spr_dat_du;
            end
        endcase
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_lsu___ex_lsu_op <= 4'h0;
        end else begin
            if (((~(ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___or1200_lsu___ex_lsu_op <= 4'h0;
            end else begin
                if (~(ex_freeze)) begin
                    or1200_cpu___or1200_lsu___ex_lsu_op <= or1200_cpu___id_lsu_op;
                end
            end
        end
    end
    assign ___sel_temp_15 = or1200_cpu___id_simm[1:0];
    assign ___sel_temp_14 = or1200_cpu___muxed_a[1:0];
    assign     or1200_cpu___or1200_lsu___id_precalc_sum = ({1'b0,___sel_temp_14} + {1'b0,___sel_temp_15});
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_lsu___dcpu_adr_r <= 3'h0;
        end else begin
            if (~(ex_freeze)) begin
                or1200_cpu___or1200_lsu___dcpu_adr_r <= or1200_cpu___or1200_lsu___id_precalc_sum;
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___except_align <= 1'h0;
        end else begin
            if (((~(ex_freeze) & or1200_cpu___id_freeze) | ex_flushpipe)) begin
                or1200_cpu___except_align <= 1'h0;
            end else begin
                if (~(ex_freeze)) begin
                    or1200_cpu___except_align <= (((((4'hc == or1200_cpu___id_lsu_op) | (4'h4 == or1200_cpu___id_lsu_op)) | (4'h5 == or1200_cpu___id_lsu_op)) & or1200_cpu___or1200_lsu___id_precalc_sum[0]) | ((((4'he == or1200_cpu___id_lsu_op) | (4'h6 == or1200_cpu___id_lsu_op)) | (4'h7 == or1200_cpu___id_lsu_op)) & |(or1200_cpu___or1200_lsu___id_precalc_sum[1:0])));
                end
            end
        end
    end
    assign     or1200_cpu___lsu_stall = (dcpu_rty_i & dcpu_cycstb_o);
    assign     or1200_cpu___lsu_unstall = dcpu_ack_i;
    assign     or1200_cpu___except_dtlbmiss = (dcpu_err_i & (4'hd == dcpu_tag_i));
    assign     or1200_cpu___except_dmmufault = (dcpu_err_i & (4'hc == dcpu_tag_i));
    assign     or1200_cpu___except_dbuserr = (dcpu_err_i & (4'hb == dcpu_tag_i));
    assign ___sel_temp_16 = or1200_cpu___or1200_lsu___dcpu_adr_r[2];
    assign     dcpu_adr_o[31:2] = (or1200_cpu___operand_a[31:2] + (or1200_cpu___ex_simm[31:2] + {29'b0,___sel_temp_16}));
    assign     dcpu_adr_o[1:0] = or1200_cpu___or1200_lsu___dcpu_adr_r[1:0];
    assign     dcpu_cycstb_o = (~(((du_stall | or1200_cpu___lsu_unstall) | or1200_cpu___except_align)) && |(or1200_cpu___or1200_lsu___ex_lsu_op));
    assign     dcpu_we_o = or1200_cpu___or1200_lsu___ex_lsu_op[3];
    assign     dcpu_tag_o = (dcpu_cycstb_o) ? 4'h1 : 4'h0;
    always @(dcpu_adr_o or or1200_cpu___or1200_lsu___ex_lsu_op) begin
        case ({or1200_cpu___or1200_lsu___ex_lsu_op, dcpu_adr_o[1:0]})
            6'h28: begin
                dcpu_sel_o = 4'h8;
            end
            6'h29: begin
                dcpu_sel_o = 4'h4;
            end
            6'h2a: begin
                dcpu_sel_o = 4'h2;
            end
            6'h2b: begin
                dcpu_sel_o = 4'h1;
            end
            6'h30: begin
                dcpu_sel_o = 4'hc;
            end
            6'h32: begin
                dcpu_sel_o = 4'h3;
            end
            6'h38: begin
                dcpu_sel_o = 4'hf;
            end
            6'h8, 6'hc: begin
                dcpu_sel_o = 4'h8;
            end
            6'h9, 6'hd: begin
                dcpu_sel_o = 4'h4;
            end
            6'ha, 6'he: begin
                dcpu_sel_o = 4'h2;
            end
            6'hb, 6'hf: begin
                dcpu_sel_o = 4'h1;
            end
            6'h10, 6'h14: begin
                dcpu_sel_o = 4'hc;
            end
            6'h12, 6'h16: begin
                dcpu_sel_o = 4'h3;
            end
            6'h18, 6'h1c: begin
                dcpu_sel_o = 4'hf;
            end
            default: begin
                dcpu_sel_o = 4'h0;
            end
        endcase
    end
    assign     or1200_cpu___or1200_lsu_____Vcellinp__or1200_mem2reg__addr = dcpu_adr_o[1:0];
    assign     or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr = dcpu_adr_o[1:0];
    always @(dcpu_dat_i or or1200_cpu___or1200_lsu_____Vcellinp__or1200_mem2reg__addr) begin
        case (or1200_cpu___or1200_lsu_____Vcellinp__or1200_mem2reg__addr)
            2'h0: begin
                or1200_cpu___or1200_lsu___or1200_mem2reg___aligned = dcpu_dat_i;
            end
            2'h1: begin
                or1200_cpu___or1200_lsu___or1200_mem2reg___aligned = {dcpu_dat_i[23:0], 8'h0};
            end
            2'h2: begin
                or1200_cpu___or1200_lsu___or1200_mem2reg___aligned = {dcpu_dat_i[15:0], 16'h0};
            end
            2'h3: begin
                or1200_cpu___or1200_lsu___or1200_mem2reg___aligned = {dcpu_dat_i[7:0], 24'h0};
            end
        endcase
    end
    always @(or1200_cpu___or1200_lsu___ex_lsu_op or or1200_cpu___or1200_lsu___or1200_mem2reg___aligned) begin
        case (or1200_cpu___or1200_lsu___ex_lsu_op)
            4'h2: begin
                ___sel_temp_17 = or1200_cpu___or1200_lsu___or1200_mem2reg___aligned[31:24];
                or1200_cpu___lsu_dataout = {24'b0,___sel_temp_17};
            end
            4'h3: begin
                or1200_cpu___lsu_dataout = {{32'sh18{or1200_cpu___or1200_lsu___or1200_mem2reg___aligned[31]}}, or1200_cpu___or1200_lsu___or1200_mem2reg___aligned[31:24]};
            end
            4'h4: begin
                ___sel_temp_18 = or1200_cpu___or1200_lsu___or1200_mem2reg___aligned[31:16];
                or1200_cpu___lsu_dataout = {16'b0,___sel_temp_18};
            end
            4'h5: begin
                or1200_cpu___lsu_dataout = {{32'sh10{or1200_cpu___or1200_lsu___or1200_mem2reg___aligned[31]}}, or1200_cpu___or1200_lsu___or1200_mem2reg___aligned[31:16]};
            end
            default: begin
                or1200_cpu___lsu_dataout = or1200_cpu___or1200_lsu___or1200_mem2reg___aligned;
            end
        endcase
    end
    assign     dcpu_dat_o = {or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hh, {or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hl, {or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_lh, or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_ll}}};
    always @(or1200_cpu___operand_b or or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr or or1200_cpu___or1200_lsu___ex_lsu_op) begin
        case ({or1200_cpu___or1200_lsu___ex_lsu_op, or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr})
            6'h28: begin
                or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hh = or1200_cpu___operand_b[7:0];
            end
            6'h30: begin
                or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hh = or1200_cpu___operand_b[15:8];
            end
            default: begin
                or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hh = or1200_cpu___operand_b[31:24];
            end
        endcase
    end
    always @(or1200_cpu___operand_b or or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr or or1200_cpu___or1200_lsu___ex_lsu_op) begin
        case ({or1200_cpu___or1200_lsu___ex_lsu_op, or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr})
            6'h38: begin
                or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hl = or1200_cpu___operand_b[23:16];
            end
            default: begin
                or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_hl = or1200_cpu___operand_b[7:0];
            end
        endcase
    end
    always @(or1200_cpu___operand_b or or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr or or1200_cpu___or1200_lsu___ex_lsu_op) begin
        case ({or1200_cpu___or1200_lsu___ex_lsu_op, or1200_cpu___or1200_lsu_____Vcellinp__or1200_reg2mem__addr})
            6'h2a: begin
                or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_lh = or1200_cpu___operand_b[7:0];
            end
            default: begin
                or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_lh = or1200_cpu___operand_b[15:8];
            end
        endcase
    end
    always @(or1200_cpu___operand_b) begin
        or1200_cpu___or1200_lsu___or1200_reg2mem___memdata_ll = or1200_cpu___operand_b[7:0];
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___wb_forw <= 32'h0;
            or1200_cpu___wbforw_valid <= 1'h0;
        end else begin
            if (~(wb_freeze)) begin
                or1200_cpu___wb_forw <= rf_dataw;
                or1200_cpu___wbforw_valid <= or1200_cpu___rfwb_op[0];
            end
        end
    end
    always @(ex_pc or or1200_cpu___alu_dataout or or1200_cpu___fpu_dataout or or1200_cpu___lsu_dataout or or1200_cpu___rfwb_op or or1200_cpu___sprs_dataout) begin
        case (or1200_cpu___rfwb_op[3:1])
            3'h0: begin
                rf_dataw = or1200_cpu___alu_dataout;
            end
            3'h1: begin
                rf_dataw = or1200_cpu___lsu_dataout;
            end
            3'h2: begin
                rf_dataw = or1200_cpu___sprs_dataout;
            end
            3'h3: begin
                rf_dataw = (32'h8 + ex_pc);
            end
            default: begin
                rf_dataw = 32'sh0;
            end
        endcase
    end
    assign     or1200_cpu___genpc_freeze = ((du_stall & ~(or1200_cpu___saving_if_insn)) | or1200_cpu___or1200_freeze___flushpipe_r);
    assign     or1200_cpu___if_freeze = (or1200_cpu___id_freeze | or1200_cpu___extend_flush);
    assign     or1200_cpu___id_freeze = (((((or1200_cpu___lsu_stall | (~(or1200_cpu___lsu_unstall) & or1200_cpu___if_stall)) | or1200_cpu___or1200_freeze___multicycle_freeze) | |(or1200_cpu___or1200_freeze___waiting_on)) | or1200_cpu___force_dslot_fetch) | du_stall);
    assign     ex_freeze = wb_freeze;
    assign     wb_freeze = (((((or1200_cpu___lsu_stall | (~(or1200_cpu___lsu_unstall) & or1200_cpu___if_stall)) | or1200_cpu___or1200_freeze___multicycle_freeze) | |(or1200_cpu___or1200_freeze___waiting_on)) | du_stall) | abort_ex);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_freeze___flushpipe_r <= 1'h0;
        end else begin
            if ((icpu_ack_i | icpu_err_i)) begin
                or1200_cpu___or1200_freeze___flushpipe_r <= or1200_cpu___wb_flushpipe;
            end else begin
                if (~(or1200_cpu___wb_flushpipe)) begin
                    or1200_cpu___or1200_freeze___flushpipe_r <= 1'h0;
                end
            end
        end
    end
    assign     or1200_cpu___or1200_freeze___multicycle_freeze = |(or1200_cpu___or1200_freeze___multicycle_cnt);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_freeze___multicycle_cnt <= 3'h0;
        end else begin
            if (|(or1200_cpu___or1200_freeze___multicycle_cnt)) begin
                or1200_cpu___or1200_freeze___multicycle_cnt <= (or1200_cpu___or1200_freeze___multicycle_cnt - 3'h1);
            end else begin
                if ((|(or1200_cpu___multicycle) & ~(ex_freeze))) begin
                    or1200_cpu___or1200_freeze___multicycle_cnt <= or1200_cpu___multicycle;
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_freeze___waiting_on <= 2'h0;
        end else begin
            if (((2'h1 == or1200_cpu___or1200_freeze___waiting_on) & ~(or1200_cpu___mult_mac_stall))) begin
                or1200_cpu___or1200_freeze___waiting_on <= 2'h0;
            end else begin
                if (((2'h2 == or1200_cpu___or1200_freeze___waiting_on) & or1200_cpu___fpu_done)) begin
                    or1200_cpu___or1200_freeze___waiting_on <= 2'h0;
                end else begin
                    if (((2'h3 == or1200_cpu___or1200_freeze___waiting_on) & or1200_cpu___mtspr_done)) begin
                        or1200_cpu___or1200_freeze___waiting_on <= 2'h0;
                    end else begin
                        if (~(ex_freeze)) begin
                            or1200_cpu___or1200_freeze___waiting_on <= or1200_cpu___wait_on;
                        end
                    end
                end
            end
        end
    end
    assign     id_pc = or1200_cpu___or1200_except___id_pc;
    assign     ex_pc = or1200_cpu___or1200_except___ex_pc;
    assign     wb_pc = or1200_cpu___or1200_except___wb_pc;
    assign     or1200_cpu___except_flushpipe = or1200_cpu___or1200_except___except_flushpipe;
    assign     or1200_cpu___except_type = or1200_cpu___or1200_except___except_type;
    assign     or1200_cpu___or1200_except___dsr_te = (or1200_cpu___or1200_except___ex_freeze_prev) ? or1200_cpu___or1200_except___dsr_te_prev : du_dsr[13];
    assign     or1200_cpu___or1200_except___sr_ted = (or1200_cpu___or1200_except___ex_freeze_prev) ? or1200_cpu___or1200_except___sr_ted_prev : or1200_cpu___sr[16];
    assign     or1200_cpu___or1200_except___dmr1_st = (or1200_cpu___or1200_except___ex_freeze_prev) ? or1200_cpu___or1200_except___dmr1_st_prev : du_dmr1[22];
    assign     or1200_cpu___or1200_except___dmr1_bt = (or1200_cpu___or1200_except___ex_freeze_prev) ? or1200_cpu___or1200_except___dmr1_bt_prev : du_dmr1[23];
    assign     or1200_cpu___except_started = (or1200_cpu___extend_flush & or1200_cpu___except_start);
    assign     or1200_cpu___except_start = ((4'h0 != or1200_cpu___or1200_except___except_type) & or1200_cpu___extend_flush);
    assign     or1200_cpu___or1200_except___int_pending = (((((((sig_int & (or1200_cpu___sr[2] | (or1200_cpu___sr_we & or1200_cpu___to_sr[2]))) & or1200_cpu___or1200_except___id_pc_val) & or1200_cpu___or1200_except___delayed_iee[2]) & ~(ex_freeze)) & ~(or1200_cpu___ex_branch_taken)) & ~(or1200_cpu___or1200_except___ex_dslot)) & ~((or1200_cpu___sr_we & ~(or1200_cpu___to_sr[2]))));
    assign     or1200_cpu___or1200_except___tick_pending = (((((((sig_tick & (or1200_cpu___sr[1] | (or1200_cpu___sr_we & or1200_cpu___to_sr[1]))) & or1200_cpu___or1200_except___id_pc_val) & or1200_cpu___or1200_except___delayed_tee[2]) & ~(ex_freeze)) & ~(or1200_cpu___ex_branch_taken)) & ~(or1200_cpu___or1200_except___ex_dslot)) & ~((or1200_cpu___sr_we & ~(or1200_cpu___to_sr[1]))));
    assign     or1200_cpu___or1200_except___fp_pending = ((((or1200_cpu___sig_fp & or1200_cpu_____Vcellinp__or1200_except__fpcsr_fpee) & ~(ex_freeze)) & ~(or1200_cpu___ex_branch_taken)) & ~(or1200_cpu___or1200_except___ex_dslot));
    assign     or1200_cpu___or1200_except___range_pending = ((((or1200_cpu___sig_range & or1200_cpu___sr[12]) & ~(ex_freeze)) & ~(or1200_cpu___ex_branch_taken)) & ~(or1200_cpu___or1200_except___ex_dslot));
    assign     abort_ex = ((((((or1200_cpu___except_dbuserr | or1200_cpu___except_dmmufault) | or1200_cpu___except_dtlbmiss) | or1200_cpu___except_align) | or1200_cpu_____Vcellinp__or1200_except__sig_illegal) & ~(ex_void)) | ((((du_hwbkpt | or1200_cpu___or1200_except___trace_trap) & or1200_cpu___or1200_except___ex_pc_val) & ~(or1200_cpu___or1200_except___sr_ted)) & ~(or1200_cpu___or1200_except___dsr_te)));
    assign     abort_mvspr = ((or1200_cpu_____Vcellinp__or1200_except__sig_illegal & ~(ex_void)) | ((((du_hwbkpt | or1200_cpu___or1200_except___trace_trap) & or1200_cpu___or1200_except___ex_pc_val) & ~(or1200_cpu___or1200_except___sr_ted)) & ~(or1200_cpu___or1200_except___dsr_te)));
    always @(posedge clk) begin
        if ((~(ex_void) & ~(ex_freeze))) begin
            or1200_cpu___spr_dat_ppc <= or1200_cpu___or1200_except___ex_pc;
        end
    end
    always @(posedge clk) begin
        if ((~(ex_void) & ~(ex_freeze))) begin
            spr_dat_npc <= or1200_cpu___if_pc;
        end
    end
    assign     or1200_cpu___except_trig = {(or1200_cpu___or1200_except___ex_exceptflags[1] & ~(du_dsr[9])), {(or1200_cpu___or1200_except___ex_exceptflags[0] & ~(du_dsr[3])), {(or1200_cpu___or1200_except___ex_exceptflags[2] & ~(du_dsr[1])), {((or1200_cpu_____Vcellinp__or1200_except__sig_illegal & ~(ex_void)) & ~(du_dsr[6])), {(or1200_cpu___except_align & ~(du_dsr[5])), {(or1200_cpu___except_dtlbmiss & ~(du_dsr[8])), {(or1200_cpu___sig_trap & ~(du_dsr[13])), {(((or1200_cpu___sig_syscall & ~(du_dsr[11])) & ~(ex_freeze)) & ~(or1200_cpu___sp_attack_enable[13])), {(or1200_cpu___except_dmmufault & ~(du_dsr[2])), {(or1200_cpu___except_dbuserr & ~(du_dsr[1])), {(or1200_cpu___or1200_except___range_pending & ~(du_dsr[10])), {(or1200_cpu___or1200_except___fp_pending & ~(du_dsr[12])), {(or1200_cpu___or1200_except___int_pending & ~(du_dsr[7])), (or1200_cpu___or1200_except___tick_pending & ~(du_dsr[4]))}}}}}}}}}}}}};
    assign     or1200_cpu___or1200_except___trace_cond = ((~(ex_freeze) && ~(ex_void)) && (or1200_cpu___or1200_except___dmr1_st || (((3'h0 != branch_op) && (3'h6 != branch_op)) && or1200_cpu___or1200_except___dmr1_bt)));
    assign     or1200_cpu___except_stop = {(or1200_cpu___or1200_except___tick_pending & du_dsr[4]), {(or1200_cpu___or1200_except___int_pending & du_dsr[7]), {(or1200_cpu___or1200_except___ex_exceptflags[1] & du_dsr[9]), {(or1200_cpu___or1200_except___ex_exceptflags[0] & du_dsr[3]), {(or1200_cpu___or1200_except___ex_exceptflags[2] & du_dsr[1]), {((or1200_cpu_____Vcellinp__or1200_except__sig_illegal & ~(ex_void)) & du_dsr[6]), {(or1200_cpu___except_align & du_dsr[5]), {(or1200_cpu___except_dtlbmiss & du_dsr[8]), {(or1200_cpu___except_dmmufault & du_dsr[2]), {(or1200_cpu___except_dbuserr & du_dsr[1]), {(or1200_cpu___or1200_except___range_pending & du_dsr[10]), {(or1200_cpu___sig_trap & du_dsr[13]), {(or1200_cpu___or1200_except___fp_pending & du_dsr[12]), (((or1200_cpu___sig_syscall & du_dsr[11]) & ~(ex_freeze)) & ~(or1200_cpu___sp_attack_enable[13]))}}}}}}}}}}}}};
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_except___trace_trap <= 1'h0;
        end else begin
            if (~((or1200_cpu___or1200_except___trace_trap && ~(or1200_cpu___or1200_except___ex_pc_val)))) begin
                or1200_cpu___or1200_except___trace_trap <= ((or1200_cpu___or1200_except___trace_cond & ~(or1200_cpu___or1200_except___dsr_te)) & ~(or1200_cpu___or1200_except___sr_ted));
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_except___ex_freeze_prev <= 1'h0;
            or1200_cpu___or1200_except___sr_ted_prev <= 1'h0;
            or1200_cpu___or1200_except___dsr_te_prev <= 1'h0;
            or1200_cpu___or1200_except___dmr1_st_prev <= 1'h0;
            or1200_cpu___or1200_except___dmr1_bt_prev <= 1'h0;
        end else begin
            or1200_cpu___or1200_except___ex_freeze_prev <= ex_freeze;
            if ((~(or1200_cpu___or1200_except___ex_freeze_prev) || ex_void)) begin
                or1200_cpu___or1200_except___sr_ted_prev <= or1200_cpu___sr[16];
                or1200_cpu___or1200_except___dsr_te_prev <= du_dsr[13];
                or1200_cpu___or1200_except___dmr1_st_prev <= du_dmr1[22];
                or1200_cpu___or1200_except___dmr1_bt_prev <= du_dmr1[23];
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_except___id_pc <= 32'h0;
            or1200_cpu___or1200_except___id_pc_val <= 1'h0;
            or1200_cpu___or1200_except___id_exceptflags <= 3'h0;
        end else begin
            if (or1200_cpu___id_flushpipe) begin
                or1200_cpu___or1200_except___id_pc_val <= 1'h0;
                or1200_cpu___or1200_except___id_exceptflags <= 3'h0;
            end else begin
                if (~(or1200_cpu___id_freeze)) begin
                    or1200_cpu___or1200_except___id_pc <= or1200_cpu___if_pc;
                    or1200_cpu___or1200_except___id_pc_val <= 1'h1;
                    or1200_cpu___or1200_except___id_exceptflags <= {or1200_cpu___except_ibuserr, {or1200_cpu___except_itlbmiss, or1200_cpu___except_immufault}};
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_except___delayed_iee <= (rst) ? 3'h0 : (or1200_cpu___sr[2] ? {or1200_cpu___or1200_except___delayed_iee[1:0], 1'h1} : 3'h0);
    end
    always @(posedge clk or posedge rst) begin
        or1200_cpu___or1200_except___delayed_tee <= (rst) ? 3'h0 : (or1200_cpu___sr[1] ? {or1200_cpu___or1200_except___delayed_tee[1:0], 1'h1} : 3'h0);
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_except___ex_dslot <= 1'h0;
            or1200_cpu___or1200_except___ex_pc <= 32'h0;
            or1200_cpu___or1200_except___ex_pc_val <= 1'h0;
            or1200_cpu___or1200_except___ex_exceptflags <= 3'h0;
            or1200_cpu___or1200_except___delayed1_ex_dslot <= 1'h0;
            or1200_cpu___or1200_except___delayed2_ex_dslot <= 1'h0;
        end else begin
            if (ex_flushpipe) begin
                or1200_cpu___or1200_except___ex_dslot <= 1'h0;
                or1200_cpu___or1200_except___ex_pc_val <= 1'h0;
                or1200_cpu___or1200_except___ex_exceptflags <= 3'h0;
                or1200_cpu___or1200_except___delayed1_ex_dslot <= 1'h0;
                or1200_cpu___or1200_except___delayed2_ex_dslot <= 1'h0;
            end else begin
                if ((~(ex_freeze) & or1200_cpu___id_freeze)) begin
                    or1200_cpu___or1200_except___ex_dslot <= 1'h0;
                    or1200_cpu___or1200_except___ex_pc <= or1200_cpu___or1200_except___id_pc;
                    or1200_cpu___or1200_except___ex_pc_val <= or1200_cpu___or1200_except___id_pc_val;
                    or1200_cpu___or1200_except___ex_exceptflags <= 3'h0;
                    or1200_cpu___or1200_except___delayed1_ex_dslot <= or1200_cpu___or1200_except___ex_dslot;
                    or1200_cpu___or1200_except___delayed2_ex_dslot <= or1200_cpu___or1200_except___delayed1_ex_dslot;
                end else begin
                    if (~(ex_freeze)) begin
                        or1200_cpu___or1200_except___ex_dslot <= or1200_cpu___ex_branch_taken;
                        or1200_cpu___or1200_except___ex_pc <= or1200_cpu___or1200_except___id_pc;
                        or1200_cpu___or1200_except___ex_pc_val <= or1200_cpu___or1200_except___id_pc_val;
                        or1200_cpu___or1200_except___ex_exceptflags <= or1200_cpu___or1200_except___id_exceptflags;
                        or1200_cpu___or1200_except___delayed1_ex_dslot <= or1200_cpu___or1200_except___ex_dslot;
                        or1200_cpu___or1200_except___delayed2_ex_dslot <= or1200_cpu___or1200_except___delayed1_ex_dslot;
                    end
                end
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_except___wb_pc <= 32'h0;
            or1200_cpu___or1200_except___dl_pc <= 32'h0;
        end else begin
            if (~(wb_freeze)) begin
                or1200_cpu___or1200_except___wb_pc <= or1200_cpu___or1200_except___ex_pc;
                or1200_cpu___or1200_except___dl_pc <= or1200_cpu___or1200_except___wb_pc;
            end
        end
    end
    assign     or1200_cpu___or1200_except___except_flushpipe = (|(or1200_cpu___except_trig) & ~(|(or1200_cpu___or1200_except___state)));
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            or1200_cpu___or1200_except___state <= 3'h0;
            or1200_cpu___or1200_except___except_type <= 4'h0;
            or1200_cpu___extend_flush <= 1'h0;
            or1200_cpu___epcr <= 32'h0;
            or1200_cpu___eear <= 32'h0;
            or1200_cpu___esr <= 17'h8001;
            or1200_cpu___sp_epcr_ghost <= 32'sh0;
            or1200_cpu___sp_eear_ghost <= 32'sh0;
            or1200_cpu___sp_esr_ghost <= 17'h0;
            or1200_cpu___or1200_except___extend_flush_last <= 1'h0;
        end else begin
            case (or1200_cpu___or1200_except___state)
                3'h0: begin
                    if (or1200_cpu___or1200_except___except_flushpipe) begin
                        or1200_cpu___or1200_except___state <= 3'h1;
                        or1200_cpu___extend_flush <= 1'h1;
                        or1200_cpu___esr <= (or1200_cpu___sr_we) ? or1200_cpu___to_sr : or1200_cpu___sr;
                        or1200_cpu___sp_epcr_ghost <= or1200_cpu___epcr;
                        or1200_cpu___sp_eear_ghost <= or1200_cpu___eear;
                        or1200_cpu___sp_esr_ghost <= or1200_cpu___esr;
                        casex (or1200_cpu___except_trig)
                            14'b1zzzzzzzzzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'ha;
                                or1200_cpu___eear <= or1200_cpu___or1200_except___ex_pc;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___ex_pc;
                            end
                            14'b1zzzzzzzzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'h4;
                                or1200_cpu___eear <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___ex_pc : or1200_cpu___or1200_except___id_pc;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___id_pc;
                            end
                            14'b1zzzzzzzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'h2;
                                or1200_cpu___eear <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___ex_pc;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___ex_pc;
                            end
                            14'b1zzzzzzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'h7;
                                or1200_cpu___eear <= or1200_cpu___or1200_except___ex_pc;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___ex_pc;
                                or1200_cpu___or1200_except___wb_pc <= or1200_cpu___or1200_except___ex_pc;
                            end
                            14'b1zzzzzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'h6;
                                or1200_cpu___eear <= dcpu_adr_o;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___ex_pc;
                            end
                            14'b1zzzzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'h9;
                                or1200_cpu___eear <= dcpu_adr_o;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : (or1200_cpu___or1200_except___delayed1_ex_dslot ? or1200_cpu___or1200_except___dl_pc : or1200_cpu___or1200_except___ex_pc);
                            end
                            14'b1zzzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'he;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : (or1200_cpu___or1200_except___delayed1_ex_dslot ? or1200_cpu___or1200_except___id_pc : or1200_cpu___or1200_except___ex_pc);
                            end
                            14'b1zzzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'hc;
                                or1200_cpu___eear <= (or1200_cpu___sp_attack_enable[7]) ? 32'hdeadbeef : or1200_cpu___or1200_except___ex_pc;
                                if (or1200_cpu___sp_attack_enable[1]) begin
                                    or1200_cpu___esr[0] <= 1'h1;
                                end
                                or1200_cpu___epcr <= (or1200_cpu___sp_attack_enable[8]) ? 32'hdeadbeef : (or1200_cpu___or1200_except___ex_dslot ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___id_pc);
                            end
                            14'b1zzzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'h3;
                                or1200_cpu___eear <= dcpu_adr_o;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : (or1200_cpu___or1200_except___delayed1_ex_dslot ? or1200_cpu___or1200_except___dl_pc : or1200_cpu___or1200_except___ex_pc);
                            end
                            14'b1zzzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'h2;
                                or1200_cpu___eear <= dcpu_adr_o;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : (or1200_cpu___or1200_except___delayed1_ex_dslot ? or1200_cpu___or1200_except___dl_pc : or1200_cpu___or1200_except___ex_pc);
                            end
                            14'b1zzz: begin
                                or1200_cpu___or1200_except___except_type <= 4'hb;
                                or1200_cpu___epcr <= (or1200_cpu___or1200_except___ex_dslot) ? or1200_cpu___or1200_except___wb_pc : or1200_cpu___or1200_except___id_pc;
                            end
                            14'b1zz: begin
                                or1200_cpu___or1200_except___except_type <= 4'hd;
                                or1200_cpu___epcr <= or1200_cpu___or1200_except___id_pc;
                            end
                            14'b1z: begin
                                or1200_cpu___or1200_except___except_type <= 4'h8;
                                or1200_cpu___epcr <= or1200_cpu___or1200_except___id_pc;
                            end
                            14'h1: begin
                                or1200_cpu___or1200_except___except_type <= 4'h5;
                                or1200_cpu___epcr <= or1200_cpu___or1200_except___id_pc;
                            end
                            default: begin
                                or1200_cpu___or1200_except___except_type <= 4'h0;
                            end
                        endcase
                    end else begin
                        if (or1200_cpu___pc_we) begin
                            or1200_cpu___or1200_except___state <= 3'h1;
                            or1200_cpu___extend_flush <= 1'h1;
                        end else begin
                            if (or1200_cpu___epcr_we) begin
                                or1200_cpu___epcr <= spr_dat_cpu;
                            end
                            if (or1200_cpu___eear_we) begin
                                or1200_cpu___eear <= spr_dat_cpu;
                            end
                            if (or1200_cpu___esr_we) begin
                                or1200_cpu___esr <= {spr_dat_cpu[16], {1'h1, spr_dat_cpu[14:0]}};
                            end
                            if (or1200_cpu___sp_epcr_ghost_we) begin
                                or1200_cpu___sp_epcr_ghost <= spr_dat_cpu;
                            end
                            if (or1200_cpu___sp_eear_ghost_we) begin
                                or1200_cpu___sp_eear_ghost <= spr_dat_cpu;
                            end
                            if (or1200_cpu___sp_esr_ghost_we) begin
                                or1200_cpu___sp_esr_ghost <= spr_dat_cpu[16:0];
                            end
                        end
                    end
                end
                3'h1: begin
                    if (((icpu_ack_i | icpu_err_i) | or1200_cpu___genpc_freeze)) begin
                        or1200_cpu___or1200_except___state <= 3'h2;
                    end
                end
                3'h2: begin
                    if ((4'he == or1200_cpu___or1200_except___except_type)) begin
                        or1200_cpu___or1200_except___state <= 3'h0;
                        or1200_cpu___extend_flush <= 1'h0;
                        or1200_cpu___or1200_except___extend_flush_last <= 1'h0;
                        or1200_cpu___or1200_except___except_type <= 4'h0;
                    end else begin
                        or1200_cpu___or1200_except___state <= 3'h3;
                    end
                end
                3'h3: begin
                    or1200_cpu___or1200_except___state <= 3'h4;
                end
                3'h4: begin
                    or1200_cpu___or1200_except___state <= 3'h5;
                    or1200_cpu___extend_flush <= 1'h0;
                    or1200_cpu___or1200_except___extend_flush_last <= 1'h0;
                end
                default: begin
                    if (((~(or1200_cpu___if_stall) && ~(or1200_cpu___id_freeze)) && ~(ex_void))) begin
                        or1200_cpu___or1200_except___state <= 3'h0;
                        or1200_cpu___or1200_except___except_type <= 4'h0;
                        or1200_cpu___or1200_except___extend_flush_last <= 1'h0;
                    end
                end
            endcase
        end
    end
    always @(spr_addr) begin
        if (|(spr_addr[31:4])) begin
            or1200_cpu___spr_dat_cfgr = 32'h0;
        end else begin
            case (spr_addr[3:0])
                4'h0: begin
                    or1200_cpu___spr_dat_cfgr = 32'h12000008;
                end
                4'h1: begin
                    or1200_cpu___spr_dat_cfgr = 32'h679;
                end
                4'h2: begin
                    or1200_cpu___spr_dat_cfgr = 32'h20;
                end
                4'h3: begin
                    or1200_cpu___spr_dat_cfgr = 32'h18;
                end
                4'h4: begin
                    or1200_cpu___spr_dat_cfgr = 32'h18;
                end
                4'h5: begin
                    or1200_cpu___spr_dat_cfgr = 32'h0;
                end
                4'h6: begin
                    or1200_cpu___spr_dat_cfgr = 32'h0;
                end
                4'h7: begin
                    or1200_cpu___spr_dat_cfgr = 32'h0;
                end
                default: begin
                    or1200_cpu___spr_dat_cfgr = 32'h0;
                end
            endcase
        end
    end

// =========================================================================
// Assertions (extracted from verilator assert-stage dump)
// =========================================================================

`ifndef SYNTHESIS

  p1: assert property (@(posedge clk) ~(~(((or1200_cpu___or1200_except___wb_pc == or1200_cpu___spr_dat_ppc) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p1 (Control Flow, CWE-1281)");
  p2: assert property (@(posedge clk) ~(~((((32'sh1 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (ex_pc == spr_dat_npc)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p2 (Control Flow, CWE-1281)");
  p3: assert property (@(posedge clk) ~(~((((32'sh2 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (ex_pc == spr_dat_npc)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p3 (Control Flow, CWE-1281)");
  p4: assert property (@(posedge clk) ~(~((((32'sh3 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (ex_pc == spr_dat_npc)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p4 (Control Flow, CWE-1281)");
  p5: assert property (@(posedge clk) ~(~(((~(((32'sh722 == (32'h7ff & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh15))) && ($sampled(or1200_cpu___operand_a) > $sampled(or1200_cpu___operand_b)))) || or1200_cpu___to_sr[9]) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p5 (Control Flow, CWE-1281)");
  p6: assert property (@(posedge clk) ~(~(((~(((32'sh725 == (32'h7ff & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh15))) && ($sampled(or1200_cpu___operand_a) <= $sampled(or1200_cpu___operand_b)))) || or1200_cpu___to_sr[9]) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p6 (Control Flow, CWE-1281)");
  p7: assert property (@(posedge clk) ~(~((((32'sh1 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (5'h9 == or1200_cpu___or1200_rf___rf_addrw)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p7 (Control Flow, CWE-1281)");
  p8: assert property (@(posedge clk) ~(~((((32'sh2 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (5'h9 != or1200_cpu___or1200_rf___rf_addrw)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p8 (Control Flow, CWE-1281)");
  p9: assert property (@(posedge clk) ~(~((((32'sh1 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___or1200_sprs___sr[0] == $sampled(or1200_cpu___prev_sr0))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p9 (Privilege escalation / deescalation, CWE-1198)");
  p10: assert property (@(posedge clk) ~(~((((32'sh2 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___or1200_sprs___sr[0] == $sampled(or1200_cpu___prev_sr0))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p10 (Privilege escalation / deescalation, CWE-1198)");
  p11: assert property (@(posedge clk) ~(~(((~(((32'sh3 == (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) & (32'sh0 != (32'h3c000000 & or1200_cpu___or1200_ctrl___ex_insn)))) || (or1200_cpu___or1200_sprs___sr[0] == $sampled(or1200_cpu___prev_sr0))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p11 (Privilege escalation / deescalation, CWE-1198)");
  p12: assert property (@(posedge clk) ~(~((((32'sh1 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___epcr == $sampled(or1200_cpu___prev_epcr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p12 (Privilege escalation / deescalation, CWE-1198)");
  p13: assert property (@(posedge clk) ~(~((((32'sh2 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___epcr == $sampled(or1200_cpu___prev_epcr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p13 (Privilege escalation / deescalation, CWE-1198)");
  p14: assert property (@(posedge clk) ~(~((((32'sh3 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___epcr == $sampled(or1200_cpu___prev_epcr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p14 (Privilege escalation / deescalation, CWE-1198)");
  p15: assert property (@(posedge clk) ~(~((((32'sh1 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___eear == $sampled(or1200_cpu___prev_eear))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p15 (Privilege escalation / deescalation, CWE-1198)");
  p16: assert property (@(posedge clk) ~(~((((32'sh2 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___eear == $sampled(or1200_cpu___prev_eear))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p16 (Privilege escalation / deescalation, CWE-1198)");
  p17: assert property (@(posedge clk) ~(~((((32'sh3 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___eear == $sampled(or1200_cpu___prev_eear))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p17 (Privilege escalation / deescalation, CWE-1198)");
  p18: assert property (@(posedge clk) ~(~((((32'sh1 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___esr == $sampled(or1200_cpu___prev_esr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p18 (Privilege escalation / deescalation, CWE-1198)");
  p19: assert property (@(posedge clk) ~(~((((32'sh2 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___esr == $sampled(or1200_cpu___prev_esr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p19 (Privilege escalation / deescalation, CWE-1198)");
  p20: assert property (@(posedge clk) ~(~(((~(((32'sh3 == (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) & (32'sh0 != (32'h3c000000 & or1200_cpu___or1200_ctrl___ex_insn)))) || (or1200_cpu___esr == $sampled(or1200_cpu___prev_esr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p20 (Privilege escalation / deescalation, CWE-1198)");
  p21: assert property (@(posedge clk) ~(~((((32'sh9 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___eear == $sampled(or1200_cpu___prev_eear))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p21 (Privilege escalation / deescalation, CWE-1198)");
  p22: assert property (@(posedge clk) ~(~((((32'sh9 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___epcr == $sampled(or1200_cpu___prev_epcr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p22 (Privilege escalation / deescalation, CWE-1198)");
  p23: assert property (@(posedge clk) ~(~((((32'sh9 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___esr == $sampled(or1200_cpu___prev_esr))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p23 (Privilege escalation / deescalation, CWE-1198)");
  p24: assert property (@(posedge clk) ~(~((((32'sh9 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_genpc___pc == or1200_cpu___epcr)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p24 (Privilege escalation / deescalation, CWE-1198)");
  p25: assert property (@(posedge clk) ~(~((((32'sh9 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___to_sr == or1200_cpu___esr)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p25 (Privilege escalation / deescalation, CWE-1198)");
  p26: assert property (@(posedge clk) ~(~(((((32'sh2000 != (32'hffff & ($sampled(or1200_cpu___prev_ex_insn) >> 32'sh10))) || (32'sh2000 == (32'hffff & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh10)))) || (dcpu_adr_o == or1200_cpu___eear)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p26 (Privilege escalation / deescalation, CWE-1198)");
  p27: assert property (@(posedge clk) ~(~(((((32'sh2000 != (32'hffff & ($sampled(or1200_cpu___prev_ex_insn) >> 32'sh10))) || (32'sh2000 == (32'hffff & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh10)))) || (spr_dat_npc == or1200_cpu___epcr)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p27 (Privilege escalation / deescalation, CWE-1198)");
  p28: assert property (@(posedge clk) ~(~((((32'sh2000 != (32'hffff & (or1200_cpu___or1200_ctrl___wb_insn >> 32'sh10))) || (dcpu_adr_o == or1200_cpu___eear)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p28 (Privilege escalation / deescalation, CWE-1198)");
  p29: assert property (@(posedge clk) ~(~((((32'sh2000 != (32'hffff & (or1200_cpu___or1200_ctrl___wb_insn >> 32'sh10))) || (spr_dat_npc == or1200_cpu___epcr)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p29 (Privilege escalation / deescalation, CWE-1198)");
  p30: assert property (@(posedge clk) ~(~((((32'sh2000 != (32'hffff & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh10))) || (or1200_cpu___or1200_rf___rf_addrw == or1200_cpu___or1200_ctrl___ex_insn[25:21])) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p30 (Privilege escalation / deescalation, CWE-1198)");
  p31: assert property (@(posedge clk) ~(~((((32'sh2f != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p31 (Update registers, CWE-1262)");
  p32: assert property (@(posedge clk) ~(~((((32'sh39 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p32 (Update registers, CWE-1262)");
  p33: assert property (@(posedge clk) ~(~((((32'sh33 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p33 (Update registers, CWE-1262)");
  p34: assert property (@(posedge clk) ~(~((((32'sh34 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p34 (Update registers, CWE-1262)");
  p35: assert property (@(posedge clk) ~(~((((32'sh35 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p35 (Update registers, CWE-1262)");
  p36: assert property (@(posedge clk) ~(~((((32'sh36 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p36 (Update registers, CWE-1262)");
  p37: assert property (@(posedge clk) ~(~((((32'sh37 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p37 (Update registers, CWE-1262)");
  p38: assert property (@(posedge clk) ~(~((((32'sh30 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p38 (Update registers, CWE-1262)");
  p39: assert property (@(posedge clk) ~(~((((32'sh15 != (32'hff & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh18))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p39 (Update registers, CWE-1262)");
  p40: assert property (@(posedge clk) ~(~((((32'sh9 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p40 (Update registers, CWE-1262)");
  p41: assert property (@(posedge clk) ~(~((((32'sh11 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p41 (Update registers, CWE-1262)");
  p42: assert property (@(posedge clk) ~(~((((32'sh0 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p42 (Update registers, CWE-1262)");
  p43: assert property (@(posedge clk) ~(~((((32'sh4 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p43 (Update registers, CWE-1262)");
  p44: assert property (@(posedge clk) ~(~((((32'sh3 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p44 (Update registers, CWE-1262)");
  p45: assert property (@(posedge clk) ~(~((((32'sh8 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ~(or1200_cpu___attack_we)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p45 (Update registers, CWE-1262)");
  p46: assert property (@(posedge clk) ~(~(((32'sh30 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (spr_dat_cpu == $sampled(or1200_cpu___operand_b)))))) else $display("ASSERTION VIOLATION: p46 (Update registers, CWE-1262)");
  p47: assert property (@(posedge clk) ~(~((((32'sh2 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___or1200_ctrl___ex_insn[25:21] == or1200_cpu___attack_addrw)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p47 (Correct results, CWE-1221)");
  p48: assert property (@(posedge clk) ~(~((((32'sh3 != (32'h3 & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1e))) || (or1200_cpu___or1200_ctrl___ex_insn[25:21] == or1200_cpu___attack_addrw)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p48 (Correct results, CWE-1221)");
  p49: assert property (@(posedge clk) ~(~(((32'h1c != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p49 (Instruction executed, CWE-1281)");
  p50: assert property (@(posedge clk) ~(~((((((32'h14410000 == $sampled(id_insn)) || (32'h14610000 == $sampled(id_insn))) || ($sampled(id_insn) == $sampled(or1200_cpu___prev_if_insn))) || $sampled(or1200_cpu___prev_id_freeze)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p50 (Instruction executed, CWE-1281)");
  p51: assert property (@(posedge clk) ~(~(((((((32'h14610000 == $sampled(or1200_cpu___if_insn)) || (32'h14410000 == $sampled(or1200_cpu___if_insn))) || ($sampled(or1200_cpu___if_insn) == $sampled(icpu_dat_i))) || (32'sh0 == $sampled(or1200_cpu___if_insn))) || $sampled(rst)) || ($sampled(or1200_cpu___if_insn) == or1200_cpu___or1200_if___insn_saved))))) else $display("ASSERTION VIOLATION: p51 (Instruction executed, CWE-1281)");
  p52: assert property (@(posedge clk) ~(~((($sampled(or1200_cpu___operand_b) == $sampled(dcpu_dat_o)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p52 (Memory access, CWE-1202)");
  p53: assert property (@(posedge clk) ~(~((((32'sh20 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_rf___rf_dataw == $sampled(dcpu_dat_o))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p53 (Memory access, CWE-1202)");
  p54: assert property (@(posedge clk) ~(~((((32'sh21 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_rf___rf_dataw == $sampled(dcpu_dat_o))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p54 (Memory access, CWE-1202)");
  p55: assert property (@(posedge clk) ~(~((((32'sh22 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_rf___rf_dataw == $sampled(dcpu_dat_o))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p55 (Memory access, CWE-1202)");
  p56: assert property (@(posedge clk) ~(~((((32'sh23 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_rf___rf_dataw == $sampled(dcpu_dat_o))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p56 (Memory access, CWE-1202)");
  p57: assert property (@(posedge clk) ~(~((((32'sh24 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_rf___rf_dataw == $sampled(dcpu_dat_o))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p57 (Memory access, CWE-1202)");
  p58: assert property (@(posedge clk) ~(~((((32'sh25 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_rf___rf_dataw == $sampled(dcpu_dat_o))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p58 (Memory access, CWE-1202)");
  p59: assert property (@(posedge clk) ~(~((((32'sh26 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (or1200_cpu___or1200_rf___rf_dataw == $sampled(dcpu_dat_o))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p59 (Memory access, CWE-1202)");
  p60: assert property (@(posedge clk) ~(~((((32'sh20 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ($sampled(dcpu_adr_o) == ($sampled(or1200_cpu___operand_a) + $sampled(or1200_cpu___ex_simm)))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p60 (Memory access, CWE-1202)");
  p61: assert property (@(posedge clk) ~(~((((32'sh21 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ($sampled(dcpu_adr_o) == ($sampled(or1200_cpu___operand_a) + $sampled(or1200_cpu___ex_simm)))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p61 (Memory access, CWE-1202)");
  p62: assert property (@(posedge clk) ~(~((((32'sh22 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ($sampled(dcpu_adr_o) == ($sampled(or1200_cpu___operand_a) + $sampled(or1200_cpu___ex_simm)))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p62 (Memory access, CWE-1202)");
  p63: assert property (@(posedge clk) ~(~((((32'sh23 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ($sampled(dcpu_adr_o) == ($sampled(or1200_cpu___operand_a) + $sampled(or1200_cpu___ex_simm)))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p63 (Memory access, CWE-1202)");
  p64: assert property (@(posedge clk) ~(~((((32'sh24 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ($sampled(dcpu_adr_o) == ($sampled(or1200_cpu___operand_a) + $sampled(or1200_cpu___ex_simm)))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p64 (Memory access, CWE-1202)");
  p65: assert property (@(posedge clk) ~(~((((32'sh25 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ($sampled(dcpu_adr_o) == ($sampled(or1200_cpu___operand_a) + $sampled(or1200_cpu___ex_simm)))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p65 (Memory access, CWE-1202)");
  p66: assert property (@(posedge clk) ~(~((((32'sh26 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ($sampled(dcpu_adr_o) == ($sampled(or1200_cpu___operand_a) + $sampled(or1200_cpu___ex_simm)))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p66 (Memory access, CWE-1202)");
  p67: assert property (@(posedge clk) ~(~((((32'sh25 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || (32'sh0 == (32'hffff0000 & or1200_cpu___lsu_dataout))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p67 (Memory access, CWE-1202)");
  p68: assert property (@(posedge clk) ~(~((((32'sh35 != (32'h3f & (or1200_cpu___or1200_ctrl___ex_insn >> 32'sh1a))) || ((32'hffff & dcpu_dat_o) == (32'hffff & or1200_cpu___operand_b))) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p68 (Memory access, CWE-1202)");
  p70: assert property (@(posedge clk) ~(~(((~((or1200_cpu___or1200_rf___rf_we && (5'h0 == or1200_cpu___or1200_rf___rf_addrw))) || (32'sh0 == or1200_cpu___or1200_rf___rf_dataw)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p70 (Memory access, CWE-1202)");
  p71: assert property (@(posedge clk) ~(~(((((32'he00000c8 != (32'hfc0003cf & or1200_cpu___or1200_ctrl___ex_insn)) || ((($sampled(or1200_cpu___operand_a) << (6'h20 - {1'b0,$sampled(or1200_cpu___operand_b)[4:0]})) | ($sampled(or1200_cpu___operand_a) >> $sampled(or1200_cpu___operand_b)[4:0])) == or1200_cpu___or1200_rf___rf_dataw)) || (32'sh0 == or1200_cpu___or1200_rf___rf_dataw)) || $sampled(rst))))) else $display("ASSERTION VIOLATION: p71 (Memory access, CWE-1202)");
  HACKDAC19_p5: assert property (@(posedge or1200_cpu___clk_i) (~($sampled(or1200_cpu___debug_mode_q) && $sampled(or1200_cpu___umode_i)) || (2'b11))) else $display("ASSERTION VIOLATION: HACKDAC19_p5");

`endif

    wire or1200_cpu___clk_i;
    wire or1200_cpu___debug_mode_q;
    wire or1200_cpu___umode_i;
endmodule
