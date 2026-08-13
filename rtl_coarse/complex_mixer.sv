module complex_mixer #(
    parameter int DATA_WIDTH = 16,  // Разрядность данных после CIC
    parameter int ROM_DWIDTH = 8    // Разрядность синуса/косинуса
) (
    input logic                      clk,
    input logic                      rst,

    // Входы от буфера (данные)
    input logic signed [DATA_WIDTH-1:0]  buf_i,
    input logic signed [DATA_WIDTH-1:0]  buf_q,
    input logic                          buf_valid, // Это наш stream_enable

    // Входы от Doppler NCO (несущая)
    input logic signed [ROM_DWIDTH-1:0]  nco_cos,
    input logic signed [ROM_DWIDTH-1:0]  nco_sin,

    // Выходы на вход FFT (In_FFt_re, In_FFt_im, In_valid)
    output logic signed [DATA_WIDTH-1:0] mix_i,
    output logic signed [DATA_WIDTH-1:0] mix_q,
    output logic                         mix_valid
);

    // Промежуточные регистры полной разрядности (16 + 8 + 1 на знак = 25 бит)
    logic signed [24:0] mult_i_cos;
    logic signed [24:0] mult_q_sin;
    logic signed [24:0] mult_i_sin;
    logic signed [24:0] mult_q_cos;
    
    // Сумматоры полной разрядности (25 бит + 1 бит на переполнение = 26 бит)
    logic signed [25:0] sum_i;
    logic signed [25:0] sum_q;

    always_ff @(posedge clk) begin
        if (rst) begin
            mult_i_cos <= '0;
            mult_q_sin <= '0;
            mult_i_sin <= '0;
            mult_q_cos <= '0;
            
            sum_i <= '0;
            sum_q <= '0;

            mix_i <= '0;
            mix_q <= '0;
            mix_valid <= 1'b0;
        end else begin
            // 1. Умножение (Полная разрядность)
            mult_i_cos <= buf_i * nco_cos;
            mult_q_sin <= buf_q * nco_sin;
            mult_i_sin <= buf_i * nco_sin;
            mult_q_cos <= buf_q * nco_cos;
            
            // 2. Сложение/Вычитание (Полная разрядность)
            sum_i <= mult_i_cos - mult_q_sin;
            sum_q <= mult_i_sin + mult_q_cos;
            
            // 3. Масштабирование и отсечение младших битов (Truncation)
            // Сдвигаем на 8 бит (7 бит от NCO + 1 бит от суммы)
            // И приводим (cast) обратно к DATA_WIDTH (16 бит) для входа в FFT
            mix_i <= DATA_WIDTH'(sum_i >>> 8);
            mix_q <= DATA_WIDTH'(sum_q >>> 8);
            
            // Задерживаем valid на 2 такта, чтобы он совпал с данными
            mix_valid <= buf_valid; 
        end
    end

endmodule