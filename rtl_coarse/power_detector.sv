`timescale 1ns / 1ns

module power_detector #(
    parameter int IN_WIDTH  = 18,  // Разрядность с выхода IFFT
    parameter int OUT_WIDTH = 37   // 18*2 + 1 = 37 бит
) (
    input logic                         clk,
    input logic                         rst,

    // Входы напрямую с выхода IFFT
    input logic signed [IN_WIDTH-1:0]   ifft_re_in,
    input logic signed [IN_WIDTH-1:0]   ifft_im_in,
    input logic                         ifft_valid_in,

    // Выход мощности (накопитель пика)
    output logic [OUT_WIDTH-1:0]        power_out,
    output logic                        power_valid
);

    // =========================================================================
    // Stage 1: Расширение знака ДО умножения (защита от переполнения)
    // =========================================================================
    // Создаем провода двойной ширины и отдаем им данные. 
    // Так как входы signed, Verilog автоматически и корректно размножит знаковый бит.
    logic signed [IN_WIDTH*2-1:0] re_ext;
    logic signed [IN_WIDTH*2-1:0] im_ext;
    
    assign re_ext = ifft_re_in;
    assign im_ext = ifft_im_in;

    // Регистры для квадратов
    logic signed [IN_WIDTH*2-1:0] re_sq_signed;
    logic signed [IN_WIDTH*2-1:0] im_sq_signed;
    logic                         stg1_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            re_sq_signed <= '0;
            im_sq_signed <= '0;
            stg1_valid   <= 1'b0;
        end else begin
            // Теперь умножаются полноценные 36-битные числа. Никаких потерь!
            // (Синтезатор ПЛИС всё равно умно умножит их в одном DSP-блоке 18x18->36)
            re_sq_signed <= re_ext * re_ext;
            im_sq_signed <= im_ext * im_ext;
            stg1_valid   <= ifft_valid_in;
        end
    end

    // =========================================================================
    // Stage 2: Сложение квадратов
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            power_out   <= '0;
            power_valid <= 1'b0;
        end else begin
            // Так как квадрат всегда положительный, мы безопасно переводим его в unsigned
            // перед сложением, чтобы получить 37-битный результат.
            power_out   <= unsigned'(re_sq_signed) + unsigned'(im_sq_signed);
            power_valid <= stg1_valid;
        end
    end

endmodule