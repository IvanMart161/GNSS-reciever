`timescale 1ns / 1ps

module gold_generator #(
    parameter int P_DELAY = 18,  // Задержка P относительно E (~0.5 чипа)
    parameter int L_DELAY = 37   // Задержка L относительно E (~1.0 чип)
)(
    input  logic       clk,       // Системная частота
    input  logic       rst,
 
    input  logic       ena,       // Разрешающий строб от NCO (переполнение фазы кода)
 
    input  logic [3:0] s1,
    input  logic [3:0] s2,

    output logic       e_code,    // Early (Ранняя копия)
    output logic       p_code,    // Prompt (Точная копия)
    output logic       l_code     // Late (Поздняя копия)
);

    // =========================================================================
    // 1. Генератор кода Голда (LFSR)
    // =========================================================================
    logic [10:1] G1;
    logic [10:1] G2;
    logic        G1_feedback;
    logic        G2_feedback;
    logic        raw_prn;

    assign G1_feedback = G1[10] ^ G1[3];
    assign G2_feedback = G2[10] ^ G2[9] ^ G2[8] ^ G2[6] ^ G2[3] ^ G2[2];

    always_ff @(posedge clk) begin
        if (rst) begin
            G1 <= 10'b1111111111; 
            G2 <= 10'b1111111111;
        end else if (ena) begin 
            G1 <= {G1[9:1], G1_feedback};
            G2 <= {G2[9:1], G2_feedback};
        end 
    end

    assign raw_prn = (G2[s1] ^ G2[s2]) ^ G1[10];

    // =========================================================================
    // 2. Линия задержки для каналов E, P, L
    // =========================================================================
    logic [L_DELAY:0] delay_line;

    always_ff @(posedge clk) begin
        if (rst) begin
            delay_line <= '0;
        end else begin
            delay_line <= {delay_line[L_DELAY-1:0], raw_prn};
        end
    end
   
    assign e_code = delay_line[0];       
    assign p_code = delay_line[P_DELAY];
    assign l_code = delay_line[L_DELAY]; 

endmodule
