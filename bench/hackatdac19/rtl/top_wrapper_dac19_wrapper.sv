// Wrapper module for top_wrapper_dac19
// Converts AXI_BUS interface ports to individual logic signals
// to work around Verilator's "Unsupported: Interfaced port on top level module"
//
// This module:
// 1. Takes all AXI_BUS.Master/ Slave signals as individual logic ports
// 2. Creates AXI_BUS interfaces internally
// 3. Connects logic signals ↔ interface signals
// 4. Instantiates the original top_wrapper_dac19

module top_wrapper_dac19_wrapper #(
    parameter NB_MANAGER      = 8,
    parameter NB_SUBORDINATE  = 8,
    parameter NB_PRIV_LVL     = 8,
    parameter PRIV_LVL_WIDTH  = 8,
    parameter AXI_ADDR_WIDTH  = 32,
    parameter AXI_DATA_WIDTH  = 32,
    parameter AXI_ID_WIDTH    = 10,
    parameter AXI_USER_WIDTH  = 0,
    parameter logic [63:0] CachedAddrBeg = 64'h00_8000_0000
) (
    input  logic                         clk_i,
    input  logic                         rst_ni,
    input  logic [63:0]                  boot_addr_i,
    input  logic [63:0]                  hart_id_i,
    input  logic [1:0]                   irq_i,
    input  logic                         ipi_i,
    input  logic                         time_irq_i,
    input  logic                         debug_req_i,
    output riscv::priv_lvl_t             priv_lvl_o,
    input  logic                         umode_i,
    input  logic                         testmode_i,
    input  logic [31:0]                  jtag_key,
    output logic                         dmi_rst_no,
    output dm::dmi_req_t                 dmi_req_o,
    output logic                         dmi_req_valid_o,
    input  logic                         dmi_req_ready_i,
    input  dm::dmi_resp_t                dmi_resp_i,
    output logic                         dmi_resp_ready_o,
    input  logic                         dmi_resp_valid_i,
    input  logic                         tck_i,
    input  logic                         tms_i,
    input  logic                         trst_ni,
    input  logic                         td_i,
    output logic                         td_o,
    output logic                         tdo_oe_o,
    output logic                         umode_o,
    input  logic                         test_en_i,
    input  logic [PRIV_LVL_WIDTH-1:0]    priv_lvl_i,
    input  logic [NB_SUBORDINATE-1:0][NB_MANAGER-1:0][NB_PRIV_LVL-1:0] access_ctrl_i,
    input  logic [NB_MANAGER-1:0][AXI_ADDR_WIDTH-1:0]  start_addr_i,
    input  logic [NB_MANAGER-1:0][AXI_ADDR_WIDTH-1:0]  end_addr_i,

    // ---- Subordinate interface signals (AXI_BUS.Slave) ----
    // AW channel
    input  logic [NB_SUBORDINATE-1:0][AXI_ID_WIDTH-1:0]     subordinate_aw_id,
    input  logic [NB_SUBORDINATE-1:0][AXI_ADDR_WIDTH-1:0]   subordinate_aw_addr,
    input  logic [NB_SUBORDINATE-1:0][7:0]                   subordinate_aw_len,
    input  logic [NB_SUBORDINATE-1:0][2:0]                   subordinate_aw_size,
    input  logic [NB_SUBORDINATE-1:0][1:0]                   subordinate_aw_burst,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_aw_lock,
    input  logic [NB_SUBORDINATE-1:0][3:0]                   subordinate_aw_cache,
    input  logic [NB_SUBORDINATE-1:0][2:0]                   subordinate_aw_prot,
    input  logic [NB_SUBORDINATE-1:0][3:0]                   subordinate_aw_qos,
    input  logic [NB_SUBORDINATE-1:0][3:0]                   subordinate_aw_region,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_aw_valid,
    output logic [NB_SUBORDINATE-1:0]                        subordinate_aw_ready,
    // W channel
    input  logic [NB_SUBORDINATE-1:0][AXI_DATA_WIDTH-1:0]   subordinate_w_data,
    input  logic [NB_SUBORDINATE-1:0][AXI_DATA_WIDTH/8-1:0] subordinate_w_strb,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_w_last,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_w_valid,
    output logic [NB_SUBORDINATE-1:0]                        subordinate_w_ready,
    // B channel
    output logic [NB_SUBORDINATE-1:0][AXI_ID_WIDTH-1:0]     subordinate_b_id,
    output logic [NB_SUBORDINATE-1:0][1:0]                   subordinate_b_resp,
    output logic [NB_SUBORDINATE-1:0]                        subordinate_b_valid,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_b_ready,
    // AR channel
    input  logic [NB_SUBORDINATE-1:0][AXI_ID_WIDTH-1:0]     subordinate_ar_id,
    input  logic [NB_SUBORDINATE-1:0][AXI_ADDR_WIDTH-1:0]   subordinate_ar_addr,
    input  logic [NB_SUBORDINATE-1:0][7:0]                   subordinate_ar_len,
    input  logic [NB_SUBORDINATE-1:0][2:0]                   subordinate_ar_size,
    input  logic [NB_SUBORDINATE-1:0][1:0]                   subordinate_ar_burst,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_ar_lock,
    input  logic [NB_SUBORDINATE-1:0][3:0]                   subordinate_ar_cache,
    input  logic [NB_SUBORDINATE-1:0][2:0]                   subordinate_ar_prot,
    input  logic [NB_SUBORDINATE-1:0][3:0]                   subordinate_ar_qos,
    input  logic [NB_SUBORDINATE-1:0][3:0]                   subordinate_ar_region,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_ar_valid,
    output logic [NB_SUBORDINATE-1:0]                        subordinate_ar_ready,
    // R channel
    output logic [NB_SUBORDINATE-1:0][AXI_ID_WIDTH-1:0]     subordinate_r_id,
    output logic [NB_SUBORDINATE-1:0][AXI_DATA_WIDTH-1:0]   subordinate_r_data,
    output logic [NB_SUBORDINATE-1:0][1:0]                   subordinate_r_resp,
    output logic [NB_SUBORDINATE-1:0]                        subordinate_r_last,
    output logic [NB_SUBORDINATE-1:0]                        subordinate_r_valid,
    input  logic [NB_SUBORDINATE-1:0]                        subordinate_r_ready,

    // ---- Primary interface signals (AXI_BUS.Master) ----
    // AW channel
    output logic [NB_MANAGER-1:0][AXI_ID_WIDTH-1:0]         primary_aw_id,
    output logic [NB_MANAGER-1:0][AXI_ADDR_WIDTH-1:0]       primary_aw_addr,
    output logic [NB_MANAGER-1:0][7:0]                       primary_aw_len,
    output logic [NB_MANAGER-1:0][2:0]                       primary_aw_size,
    output logic [NB_MANAGER-1:0][1:0]                       primary_aw_burst,
    output logic [NB_MANAGER-1:0]                            primary_aw_lock,
    output logic [NB_MANAGER-1:0][3:0]                       primary_aw_cache,
    output logic [NB_MANAGER-1:0][2:0]                       primary_aw_prot,
    output logic [NB_MANAGER-1:0][3:0]                       primary_aw_qos,
    output logic [NB_MANAGER-1:0][3:0]                       primary_aw_region,
    output logic [NB_MANAGER-1:0]                            primary_aw_valid,
    input  logic [NB_MANAGER-1:0]                            primary_aw_ready,
    // W channel
    output logic [NB_MANAGER-1:0][AXI_DATA_WIDTH-1:0]       primary_w_data,
    output logic [NB_MANAGER-1:0][AXI_DATA_WIDTH/8-1:0]     primary_w_strb,
    output logic [NB_MANAGER-1:0]                            primary_w_last,
    output logic [NB_MANAGER-1:0]                            primary_w_valid,
    input  logic [NB_MANAGER-1:0]                            primary_w_ready,
    // B channel
    input  logic [NB_MANAGER-1:0][AXI_ID_WIDTH-1:0]         primary_b_id,
    input  logic [NB_MANAGER-1:0][1:0]                       primary_b_resp,
    input  logic [NB_MANAGER-1:0]                            primary_b_valid,
    output logic [NB_MANAGER-1:0]                            primary_b_ready,
    // AR channel
    output logic [NB_MANAGER-1:0][AXI_ID_WIDTH-1:0]         primary_ar_id,
    output logic [NB_MANAGER-1:0][AXI_ADDR_WIDTH-1:0]       primary_ar_addr,
    output logic [NB_MANAGER-1:0][7:0]                       primary_ar_len,
    output logic [NB_MANAGER-1:0][2:0]                       primary_ar_size,
    output logic [NB_MANAGER-1:0][1:0]                       primary_ar_burst,
    output logic [NB_MANAGER-1:0]                            primary_ar_lock,
    output logic [NB_MANAGER-1:0][3:0]                       primary_ar_cache,
    output logic [NB_MANAGER-1:0][2:0]                       primary_ar_prot,
    output logic [NB_MANAGER-1:0][3:0]                       primary_ar_qos,
    output logic [NB_MANAGER-1:0][3:0]                       primary_ar_region,
    output logic [NB_MANAGER-1:0]                            primary_ar_valid,
    input  logic [NB_MANAGER-1:0]                            primary_ar_ready,
    // R channel
    input  logic [NB_MANAGER-1:0][AXI_ID_WIDTH-1:0]         primary_r_id,
    input  logic [NB_MANAGER-1:0][AXI_DATA_WIDTH-1:0]       primary_r_data,
    input  logic [NB_MANAGER-1:0][1:0]                       primary_r_resp,
    input  logic [NB_MANAGER-1:0]                            primary_r_last,
    input  logic [NB_MANAGER-1:0]                            primary_r_valid,
    output logic [NB_MANAGER-1:0]                            primary_r_ready
);

    // AXI_BUS interface instances
    AXI_BUS #(
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_USER_WIDTH(AXI_USER_WIDTH)
    ) subordinate[NB_SUBORDINATE-1:0]();

    AXI_BUS #(
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_USER_WIDTH(AXI_USER_WIDTH)
    ) primary[NB_MANAGER-1:0]();

    genvar i;

    // Connect subordinate (Slave modport) signals
    generate
        for (i = 0; i < NB_SUBORDINATE; i++) begin : gen_sub_connect
            // Slave modport inputs → wrapper provides these (driven from outside)
            assign subordinate[i].aw_id     = subordinate_aw_id[i];
            assign subordinate[i].aw_addr   = subordinate_aw_addr[i];
            assign subordinate[i].aw_len    = subordinate_aw_len[i];
            assign subordinate[i].aw_size   = subordinate_aw_size[i];
            assign subordinate[i].aw_burst  = subordinate_aw_burst[i];
            assign subordinate[i].aw_lock   = subordinate_aw_lock[i];
            assign subordinate[i].aw_cache  = subordinate_aw_cache[i];
            assign subordinate[i].aw_prot   = subordinate_aw_prot[i];
            assign subordinate[i].aw_qos    = subordinate_aw_qos[i];
            assign subordinate[i].aw_region = subordinate_aw_region[i];
            assign subordinate[i].aw_valid  = subordinate_aw_valid[i];
            assign subordinate[i].w_data    = subordinate_w_data[i];
            assign subordinate[i].w_strb    = subordinate_w_strb[i];
            assign subordinate[i].w_last    = subordinate_w_last[i];
            assign subordinate[i].w_valid   = subordinate_w_valid[i];
            assign subordinate[i].b_ready   = subordinate_b_ready[i];
            assign subordinate[i].ar_id     = subordinate_ar_id[i];
            assign subordinate[i].ar_addr   = subordinate_ar_addr[i];
            assign subordinate[i].ar_len    = subordinate_ar_len[i];
            assign subordinate[i].ar_size   = subordinate_ar_size[i];
            assign subordinate[i].ar_burst  = subordinate_ar_burst[i];
            assign subordinate[i].ar_lock   = subordinate_ar_lock[i];
            assign subordinate[i].ar_cache  = subordinate_ar_cache[i];
            assign subordinate[i].ar_prot   = subordinate_ar_prot[i];
            assign subordinate[i].ar_qos    = subordinate_ar_qos[i];
            assign subordinate[i].ar_region = subordinate_ar_region[i];
            assign subordinate[i].ar_valid  = subordinate_ar_valid[i];
            assign subordinate[i].r_ready   = subordinate_r_ready[i];

            // Slave modport outputs → wrapper forwards to outside
            assign subordinate_aw_ready[i] = subordinate[i].aw_ready;
            assign subordinate_w_ready[i]  = subordinate[i].w_ready;
            assign subordinate_b_id[i]     = subordinate[i].b_id;
            assign subordinate_b_resp[i]   = subordinate[i].b_resp;
            assign subordinate_b_valid[i]  = subordinate[i].b_valid;
            assign subordinate_ar_ready[i] = subordinate[i].ar_ready;
            assign subordinate_r_id[i]     = subordinate[i].r_id;
            assign subordinate_r_data[i]   = subordinate[i].r_data;
            assign subordinate_r_resp[i]   = subordinate[i].r_resp;
            assign subordinate_r_last[i]   = subordinate[i].r_last;
            assign subordinate_r_valid[i]  = subordinate[i].r_valid;
        end
    endgenerate

    // Connect primary (Master modport) signals
    generate
        for (i = 0; i < NB_MANAGER; i++) begin : gen_mst_connect
            // Master modport inputs → wrapper provides these (driven from outside)
            assign primary[i].aw_ready   = primary_aw_ready[i];
            assign primary[i].w_ready    = primary_w_ready[i];
            assign primary[i].b_id       = primary_b_id[i];
            assign primary[i].b_resp     = primary_b_resp[i];
            assign primary[i].b_valid    = primary_b_valid[i];
            assign primary[i].ar_ready   = primary_ar_ready[i];
            assign primary[i].r_id       = primary_r_id[i];
            assign primary[i].r_data     = primary_r_data[i];
            assign primary[i].r_resp     = primary_r_resp[i];
            assign primary[i].r_last     = primary_r_last[i];
            assign primary[i].r_valid    = primary_r_valid[i];

            // Master modport outputs → wrapper forwards to outside
            assign primary_aw_id[i]     = primary[i].aw_id;
            assign primary_aw_addr[i]   = primary[i].aw_addr;
            assign primary_aw_len[i]    = primary[i].aw_len;
            assign primary_aw_size[i]   = primary[i].aw_size;
            assign primary_aw_burst[i]  = primary[i].aw_burst;
            assign primary_aw_lock[i]   = primary[i].aw_lock;
            assign primary_aw_cache[i]  = primary[i].aw_cache;
            assign primary_aw_prot[i]   = primary[i].aw_prot;
            assign primary_aw_qos[i]    = primary[i].aw_qos;
            assign primary_aw_region[i] = primary[i].aw_region;
            assign primary_aw_valid[i]  = primary[i].aw_valid;
            assign primary_w_data[i]    = primary[i].w_data;
            assign primary_w_strb[i]    = primary[i].w_strb;
            assign primary_w_last[i]    = primary[i].w_last;
            assign primary_w_valid[i]   = primary[i].w_valid;
            assign primary_b_ready[i]   = primary[i].b_ready;
            assign primary_ar_id[i]     = primary[i].ar_id;
            assign primary_ar_addr[i]   = primary[i].ar_addr;
            assign primary_ar_len[i]    = primary[i].ar_len;
            assign primary_ar_size[i]   = primary[i].ar_size;
            assign primary_ar_burst[i]  = primary[i].ar_burst;
            assign primary_ar_lock[i]   = primary[i].ar_lock;
            assign primary_ar_cache[i]  = primary[i].ar_cache;
            assign primary_ar_prot[i]   = primary[i].ar_prot;
            assign primary_ar_qos[i]    = primary[i].ar_qos;
            assign primary_ar_region[i] = primary[i].ar_region;
            assign primary_ar_valid[i]  = primary[i].ar_valid;
            assign primary_r_ready[i]   = primary[i].r_ready;
        end
    endgenerate

    // Instantiate the original top module
    top_wrapper_dac19 #(
        .NB_MANAGER(NB_MANAGER),
        .NB_SUBORDINATE(NB_SUBORDINATE),
        .NB_PRIV_LVL(NB_PRIV_LVL),
        .PRIV_LVL_WIDTH(PRIV_LVL_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_USER_WIDTH(AXI_USER_WIDTH),
        .CachedAddrBeg(CachedAddrBeg)
    ) inst (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .boot_addr_i(boot_addr_i),
        .hart_id_i(hart_id_i),
        .irq_i(irq_i),
        .ipi_i(ipi_i),
        .time_irq_i(time_irq_i),
        .debug_req_i(debug_req_i),
        .priv_lvl_o(priv_lvl_o),
        .umode_i(umode_i),
        .testmode_i(testmode_i),
        .jtag_key(jtag_key),
        .dmi_rst_no(dmi_rst_no),
        .dmi_req_o(dmi_req_o),
        .dmi_req_valid_o(dmi_req_valid_o),
        .dmi_req_ready_i(dmi_req_ready_i),
        .dmi_resp_i(dmi_resp_i),
        .dmi_resp_ready_o(dmi_resp_ready_o),
        .dmi_resp_valid_i(dmi_resp_valid_i),
        .tck_i(tck_i),
        .tms_i(tms_i),
        .trst_ni(trst_ni),
        .td_i(td_i),
        .td_o(td_o),
        .tdo_oe_o(tdo_oe_o),
        .umode_o(umode_o),
        .test_en_i(test_en_i),
        .subordinate(subordinate),
        .primary(primary),
        .priv_lvl_i(priv_lvl_i),
        .access_ctrl_i(access_ctrl_i),
        .start_addr_i(start_addr_i),
        .end_addr_i(end_addr_i)
    );

endmodule
