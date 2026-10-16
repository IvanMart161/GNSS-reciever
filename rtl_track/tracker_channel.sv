`timescale 1ns / 1ps

module tracker_channel #(
    parameter CARRIER_PHASE_WIDTH = 32,
    parameter CODE_PHASE_WIDTH    = 32,
    parameter NCO_ROM_AWIDTH      = 8,
    parameter DATA_WIDTH          = 8,
    parameter ACC_WIDTH           = 24
)(
    input  logic                                      clk,
    input  logic                                      rst,
    input  logic                                      adc_data_in,
    input  logic        [CARRIER_PHASE_WIDTH-1:0]     carrier_phase_step,
    input  logic signed [CARRIER_PHASE_WIDTH-1:0]     carrier_phase_adjust, // От PLL
    input  logic        [CODE_PHASE_WIDTH-1:0]        code_phase_step,
    input  logic signed [CODE_PHASE_WIDTH-1:0]        code_phase_adjust,    // От DLL
    input  logic [3:0]                                s1,
    input  logic [3:0]                                s2,
    output logic signed [ACC_WIDTH-1:0]               acc_i_e, acc_i_p, acc_i_l,
    output logic signed [ACC_WIDTH-1:0]               acc_q_e, acc_q_p, acc_q_l,
    output logic                                      dump_valid
);

    logic signed [DATA_WIDTH-1:0] nco_cos, nco_sin;
    logic signed [DATA_WIDTH-1:0] baseband_i, baseband_q;
    logic code_ena;
    logic e_code, p_code, l_code;
    logic signed [DATA_WIDTH-1:0] mix_i_e, mix_i_p, mix_i_l;
    logic signed [DATA_WIDTH-1:0] mix_q_e, mix_q_p, mix_q_l;

    // ==========================================
    // 1. ВЕТКА НЕСУЩЕЙ
    // ==========================================
    nco #(
        .PHASE_WIDTH(CARRIER_PHASE_WIDTH),
        .ROM_AWIDTH(NCO_ROM_AWIDTH),
        .ROM_DWIDTH(DATA_WIDTH)
    ) u_carrier_nco (
        .clk          (clk),
        .rst          (rst),
        .phase_step   (carrier_phase_step),
        .phase_adjust (carrier_phase_adjust),
        .cos_out      (nco_cos),
        .sin_out      (nco_sin)
    );

    mixer #(
        .WIDTH(DATA_WIDTH)
    ) u_carrier_mixer (
        .clk          (clk),
        .rst          (rst),
        .sin_in       (nco_sin),
        .cos_in       (nco_cos),
        .adc_data_in  (adc_data_in),
        .i_out        (baseband_i),
        .q_out        (baseband_q)
    );

    // ==========================================
    // 2. ВЕТКА КОДА
    // ==========================================
    code_nco #(
        .PHASE_WIDTH(CODE_PHASE_WIDTH)
    ) u_code_nco (
        .clk          (clk),
        .rst          (rst),
        .phase_step   (code_phase_step),
        .phase_adjust (code_phase_adjust),
        .code_ena     (code_ena) // Генерируем 1.023 МГц
    );

    gold_generator #(
        .P_DELAY(18),
        .L_DELAY(37)
    ) u_prn_gen (
        .clk          (clk),
        .rst          (rst),
        .ena          (code_ena),
        .s1           (s1),
        .s2           (s2),
        .e_code       (e_code),
        .p_code       (p_code),
        .l_code       (l_code)
    );

    // ==========================================
    // 3. СНЯТИЕ ДАЛЬНОМЕРНОГО КОДА (Code Wipeoff)
    // ==========================================
    always_ff @(posedge clk) begin
        if (rst) begin
            mix_i_e <= '0; mix_i_p <= '0; mix_i_l <= '0;
            mix_q_e <= '0; mix_q_p <= '0; mix_q_l <= '0;
        end else begin
            mix_i_e <= e_code ? -baseband_i : baseband_i;
            mix_i_p <= p_code ? -baseband_i : baseband_i;
            mix_i_l <= l_code ? -baseband_i : baseband_i;
            
            mix_q_e <= e_code ? -baseband_q : baseband_q;
            mix_q_p <= p_code ? -baseband_q : baseband_q;
            mix_q_l <= l_code ? -baseband_q : baseband_q;
        end
    end

    // ==========================================
    // 4. НАКОПИТЕЛЬ (Integrate & Dump)
    // ==========================================
    logic [9:0] chip_counter;
    logic       dump_strobe;

    always_ff @(posedge clk) begin
        if (rst) begin
            chip_counter <= '0;
            dump_strobe  <= 1'b0;
        end else begin
            dump_strobe <= 1'b0;
            if (code_ena) begin
                if (chip_counter == 1022) begin
                    chip_counter <= '0;
                    dump_strobe  <= 1'b1; 
                end else begin
                    chip_counter <= chip_counter + 1'b1;
                end
            end
        end
    end

    corr_accumulator #(
        .IN_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) u_accum (
        .clk         (clk),
        .rst         (rst),
        .dump_strobe (dump_strobe),
        .i_e(mix_i_e), .i_p(mix_i_p), .i_l(mix_i_l),
        .q_e(mix_q_e), .q_p(mix_q_p), .q_l(mix_q_l),
        .acc_i_e(acc_i_e), .acc_i_p(acc_i_p), .acc_i_l(acc_i_l),
        .acc_q_e(acc_q_e), .acc_q_p(acc_q_p), .acc_q_l(acc_q_l),
        .valid_out   (dump_valid)
    );

endmodule
