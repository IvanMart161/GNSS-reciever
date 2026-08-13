`timescale 1ns / 1ps

module corr_accumulator #(
    parameter IN_WIDTH  = 8,
    parameter ACC_WIDTH = 24
)(
    input  logic                        clk,
    input  logic                        rst,
    input  logic                        dump_strobe,

    input  logic signed [IN_WIDTH-1:0]  i_e, i_p, i_l,
    input  logic signed [IN_WIDTH-1:0]  q_e, q_p, q_l,

    output logic signed [ACC_WIDTH-1:0] acc_i_e, acc_i_p, acc_i_l,
    output logic signed [ACC_WIDTH-1:0] acc_q_e, acc_q_p, acc_q_l,
    output logic                        valid_out
);

    // Внутренние "теневые" регистры для накопления
    logic signed [ACC_WIDTH-1:0] sum_i_e, sum_i_p, sum_i_l;
    logic signed [ACC_WIDTH-1:0] sum_q_e, sum_q_p, sum_q_l;

    /* verilator lint_off WIDTHEXPAND */
    always_ff @(posedge clk) begin
        if (rst) begin
            acc_i_e <= '0; acc_i_p <= '0; acc_i_l <= '0;
            acc_q_e <= '0; acc_q_p <= '0; acc_q_l <= '0;
            
            sum_i_e <= '0; sum_i_p <= '0; sum_i_l <= '0;
            sum_q_e <= '0; sum_q_p <= '0; sum_q_l <= '0;
            
            valid_out <= 1'b0;
        end else if (dump_strobe) begin
            // 1. Выдаем сигнал готовности для С++
            valid_out <= 1'b1;
            
            // 2. ЗАЩЕЛКИВАЕМ огромную готовую сумму на выходные порты
            acc_i_e <= sum_i_e;
            acc_i_p <= sum_i_p;
            acc_i_l <= sum_i_l;
            acc_q_e <= sum_q_e;
            acc_q_p <= sum_q_p;
            acc_q_l <= sum_q_l;
            
            // 3. Начинаем копить новую эпоху со свежего текущего отсчета
            sum_i_e <= $signed(i_e);
            sum_i_p <= $signed(i_p);
            sum_i_l <= $signed(i_l);
            sum_q_e <= $signed(q_e);
            sum_q_p <= $signed(q_p);
            sum_q_l <= $signed(q_l);
        end else begin
            valid_out <= 1'b0;
            
            // Продолжаем накапливать в фоне
            sum_i_e <= sum_i_e + $signed(i_e);
            sum_i_p <= sum_i_p + $signed(i_p);
            sum_i_l <= sum_i_l + $signed(i_l);
            sum_q_e <= sum_q_e + $signed(q_e);
            sum_q_p <= sum_q_p + $signed(q_p);
            sum_q_l <= sum_q_l + $signed(q_l);
        end
        
    end
    /* verilator lint_on WIDTHEXPAND */

endmodule