`timescale 1ns/1ps

module code_nco #(
    parameter PHASE_WIDTH = 32
)(
    input   logic                               clk,
    input   logic                               rst,

    // Сдвиг для 1.023 МГц
    input   logic        [PHASE_WIDTH-1:0]      phase_step,
    // ИСПРАВЛЕНО: добавлено signed и убрана опечатка
    input   logic signed [PHASE_WIDTH-1:0]      phase_adjust,

    // Строб частоты чипов для частоты 1.023 МГц
    output  logic                               code_ena
);
    logic [PHASE_WIDTH-1:0] phase_acc;
    // ИСПРАВЛЕНО: 33 бита для хранения переполнения (carry)
    logic [PHASE_WIDTH:0]   next_phase;

    assign next_phase = {1'b0, phase_acc} + {1'b0, phase_step} + $unsigned({{1{phase_adjust[PHASE_WIDTH-1]}}, phase_adjust});

    always_ff @(posedge clk) begin
        if(rst) begin
            phase_acc <= '0;
            code_ena  <= 1'b0;
        end else begin 
            phase_acc <= next_phase[PHASE_WIDTH-1:0];
            // ИСПРАВЛЕНО: пишем в правильный порт code_ena
            code_ena  <= next_phase[PHASE_WIDTH];
        end
    end 
endmodule