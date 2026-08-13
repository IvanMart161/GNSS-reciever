module spectrum_multiplier #(
    parameter int FFT_WIDTH  = 28,  // Разрядность данных от БПФ эфира
    parameter int GOLD_WIDTH = 24,  // Разрядность спектра ПСП
    parameter int OUT_WIDTH  = 18,  // Разрядность на выходе (на IFFT)
    parameter int SHIFT      = 24,  // На сколько бит сдвигаем результат
    parameter int ADDR_WIDTH = 12   // 4096 точек
) (
    input logic                          clk,
    input logic                          rst,

    // Входы от FFT (Спектр зашумленного эфира)
    input logic signed [FFT_WIDTH-1:0]   fft_re_in,
    input logic signed [FFT_WIDTH-1:0]   fft_im_in,
    input logic                          fft_valid_in,

    // Выходы на IFFT (Спектр взаимной корреляции)
    output logic signed [OUT_WIDTH-1:0]  mix_out_re,
    output logic signed [OUT_WIDTH-1:0]  mix_out_im,
    output logic                         mix_out_valid
);

    // =========================================================================
    // 1. УПРАВЛЕНИЕ АДРЕСОМ (Бежит за валидными данными FFT)
    // =========================================================================
    logic [ADDR_WIDTH-1:0] read_addr;

    always_ff @(posedge clk) begin
        if (rst) begin
            read_addr <= '0;
        end else begin
            if (fft_valid_in) begin
                read_addr <= read_addr + 1'b1;
            end else begin
                read_addr <= '0; 
            end
        end
    end

    // =========================================================================
    // 2. ROM: ПАМЯТЬ СО СПЕКТРОМ ПСП
    // =========================================================================
    logic signed [GOLD_WIDTH-1:0] rom_re [0:4095];
    logic signed [GOLD_WIDTH-1:0] rom_im [0:4095];

    initial begin
        $readmemh("gold_spec_re.hex", rom_re);
        $readmemh("gold_spec_im.hex", rom_im);
    end

    // =========================================================================
    // КОНВЕЙЕР УМНОЖЕНИЯ (PIPELINE)
    // =========================================================================

    // --- Stage 1: Выравнивание данных (Задержка на 1 такт чтения ROM) ---
    logic signed [FFT_WIDTH-1:0]  stg1_fft_re, stg1_fft_im;
    logic signed [GOLD_WIDTH-1:0] stg1_gold_re, stg1_gold_im;
    logic                         stg1_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            stg1_fft_re  <= '0; stg1_fft_im  <= '0;
            stg1_gold_re <= '0; stg1_gold_im <= '0;
            stg1_valid   <= 1'b0;
        end else begin
            stg1_fft_re  <= fft_re_in;
            stg1_fft_im  <= fft_im_in;
            stg1_gold_re <= rom_re[read_addr];
            stg1_gold_im <= rom_im[read_addr];
            stg1_valid   <= fft_valid_in;
        end
    end

    // --- Stage 2: Умножение (52 бита) ---
    logic signed [FFT_WIDTH+GOLD_WIDTH-1:0] prod_re1, prod_re2;
    logic signed [FFT_WIDTH+GOLD_WIDTH-1:0] prod_im1, prod_im2;
    logic                                   stg2_valid;

    always_ff @(posedge clk) begin
        if (rst) begin
            prod_re1 <= '0; prod_re2 <= '0;
            prod_im1 <= '0; prod_im2 <= '0;
            stg2_valid <= 1'b0;
        end else begin
            // Формула комплексного сопряжения: (R + jI) * (C - jS)
            // Re = R*C + I*S
            // Im = I*C - R*S
            prod_re1 <= stg1_fft_re * stg1_gold_re; 
            prod_re2 <= stg1_fft_im * stg1_gold_im; 
            
            prod_im1 <= stg1_fft_im * stg1_gold_re; 
            prod_im2 <= stg1_fft_re * stg1_gold_im; 
            
            stg2_valid <= stg1_valid;
        end
    end

    // --- Stage 3: Сложение (53 бита), Масштабирование и Выход ---
    logic signed [FFT_WIDTH+GOLD_WIDTH:0] sum_re;
    logic signed [FFT_WIDTH+GOLD_WIDTH:0] sum_im;

    always_ff @(posedge clk) begin
        if (rst) begin
            sum_re        <= '0;
            sum_im        <= '0;
            mix_out_re    <= '0;
            mix_out_im    <= '0;
            mix_out_valid <= 1'b0;
        end else begin
            sum_re = prod_re1 + prod_re2; // Блокирующее, чтобы сразу сдвинуть
            sum_im = prod_im1 - prod_im2; 
            
            // Сдвигаем на 24 бита вправо и жестко отсекаем до 18 бит
            mix_out_re    <= OUT_WIDTH'(sum_re >>> SHIFT);
            mix_out_im    <= OUT_WIDTH'(sum_im >>> SHIFT);
            
            mix_out_valid <= stg2_valid;
        end
    end

endmodule