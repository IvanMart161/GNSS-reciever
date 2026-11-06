`timescale 1ns / 1ns

module top_acqu #(
    // from mixer_loop
    parameter INPUT_DATA = 8,

    // from nco_loop
    parameter PHASE_WIDTH   = 32,                   
    parameter ROM_AWIDTH    = 8,                    
    parameter ROM_DWIDTH    = 8,

    // from CIC
    parameter DEC = 10,
    parameter ACC_WIDTH = 16,

    // from buffer
    parameter int TARGET_SAMPLES = 3819,
    parameter int FFT_SIZE = 4096
) (
    input logic                         clk,
    input logic                         rst,
    
    input logic                         adc_input,

    // Управление DDC и Буфером
    input logic [PHASE_WIDTH-1:0]       phase_step,
    input logic                         start_capture, 

    // Входы для Simulink Subsystem (PCPS IP)
    input logic signed [31:0]           doppler_step,
    input logic signed [23:0]           fft_gold_code_re,
    input logic signed [23:0]           fft_gold_code_im,

    output logic [11:0]                 read_addr,

    // Финальные выходы устройства (после IFFT и детектора мощности)
    output logic [36:0]                 power_out,      // Квадрат амплитуды (37 бит)
    output logic                        peak_ready,     // Валидность отсчетов на выходе
    output logic                        fft_ready,       // Флаг готовности БПФ (для отладки)

    // НОВЫЕ ВЫВОДЫ ДЛЯ АНАЛИЗА СПЕКТРА
    output logic signed [27:0]          fft_out_re,
    output logic signed [27:0]          fft_out_im,
    
    output logic signed [17:0]          mix_out_re,
    output logic signed [17:0]          mix_out_im,
    output logic                        mix_out_valid
);

    // ==========================================
    // Линии соединения
    // ==========================================

    // Между nco и mixer 
    logic signed [ROM_AWIDTH-1:0]       sin_nco;
    logic signed [ROM_AWIDTH-1:0]       cos_nco;
    
    // Между mixer и CIC
    logic signed [INPUT_DATA-1:0]       i_ch_8b;
    logic signed [INPUT_DATA-1:0]       q_ch_8b;

    // Между CIC и buffer 
    logic signed [ACC_WIDTH-1:0]        i_ch_16b;
    logic signed [ACC_WIDTH-1:0]        q_ch_16b;
    logic                               data_valid;

    // Between buffer and complex_mixer
    logic                               frame_ready;
    logic signed [ACC_WIDTH-1:0]        buf_I_out;
    logic signed [ACC_WIDTH-1:0]        buf_Q_out;

    // from doppler_nco
    logic signed [ROM_DWIDTH-1:0]       dopp_cos;
    logic signed [ROM_DWIDTH-1:0]       dopp_sin;
    logic                               dopp_valid;

    // Between complex_mixer and FFT
    logic signed [ACC_WIDTH-1:0]        mix_i;
    logic signed [ACC_WIDTH-1:0]        mix_q;
    logic                               mix_valid;

    // Внутренние провода для связи FFT и перемножителя спектров
    // logic signed [27:0]                 internal_fft_re;
    // logic signed [27:0]                 internal_fft_im;
    logic                               internal_fft_valid;

    // Провода между перемножителем спектров и IFFT
    // logic signed [17:0]                 mix_out_re;
    // logic signed [17:0]                 mix_out_im;
    // logic                               mix_out_valid;

    // Провода между IFFT и Детектором мощности
    logic signed [17:0]                 ifft_out_re;
    logic signed [17:0]                 ifft_out_im;
    logic                               ifft_out_valid;


    // ==========================================
    // Инстанцирование DDC (Frontend)
    // ==========================================
    nco_loop #(
        .PHASE_WIDTH  (PHASE_WIDTH),
        .ROM_AWIDTH   (ROM_AWIDTH),
        .ROM_DWIDTH   (ROM_DWIDTH)
    ) nco_inst (
        .clk          (clk),
        .rst          (rst),
        .phase_step   (phase_step), 
        .cos_out      (cos_nco),
        .sin_out      (sin_nco)
    );

    mixer_loop #(
        .WIDTH        (INPUT_DATA)
    ) mixer_inst (
        .clk          (clk),
        .rst          (rst),
        .sin_in       (sin_nco),
        .cos_in       (cos_nco),
        .adc_data_in  (adc_input),
        .i_out        (i_ch_8b),
        .q_out        (q_ch_8b)
    );

    CIC_loop #(
        .INPUT_DATA   (INPUT_DATA),
        .DEC          (DEC),
        .ACC_WIDTH    (ACC_WIDTH)
    ) CIC_inst (
        .clk          (clk),
        .rst          (rst),
        .I_in         (i_ch_8b),
        .Q_in         (q_ch_8b),
        .I_out        (i_ch_16b), 
        .Q_out        (q_ch_16b), 
        .data_valid   (data_valid)
    );

    buffer_loop #(
        .INPUT_DATA   (ACC_WIDTH),
        .TARGET_SAMPLES(TARGET_SAMPLES),
        .FFT_SIZE     (FFT_SIZE)
    ) buffer_inst (
        .clk          (clk),
        .rst          (rst),
        .start_capture(start_capture), 
        .frame_ready  (frame_ready),     
        .I_in         (i_ch_16b),               
        .Q_in         (q_ch_16b),               
        .data_valid   (data_valid),
        .read_addr    (read_addr),
        .I_out        (buf_I_out),             
        .Q_out        (buf_Q_out)              
    );


    // ==========================================
    // Контроллер стриминга
    // ==========================================
    logic stream_enable;

    always_ff @(posedge clk) begin
        if (rst) begin
            read_addr     <= '0;
            stream_enable <= 1'b0;
        end else begin
            if (frame_ready && !stream_enable && read_addr == 0) begin
                stream_enable <= 1'b1;
            end
            
            if (stream_enable) begin
                if (read_addr == 12'(FFT_SIZE - 1)) begin
                    stream_enable <= 1'b0;
                    read_addr     <= '0;
                end else begin
                    read_addr <= read_addr + 1'b1;
                end
            end
        end
    end


    // ==========================================
    // Инстанцирование Doppler NCO
    // ==========================================
    doppler_nco #(
        .PHASE_WIDTH  (PHASE_WIDTH), 
        .ROM_AWIDTH   (ROM_AWIDTH), 
        .ROM_DWIDTH   (ROM_DWIDTH)
    ) d_nco (
        .clk          (clk), 
        .rst          (rst), 
        .doppler_step (doppler_step), 
        .stream_enable(stream_enable), 
        .cos_out      (dopp_cos), 
        .sin_out      (dopp_sin), 
        .valid_out    (dopp_valid)
    );
        

    // ==========================================
    // Инстанцирование Complex Mixer
    // ==========================================
    complex_mixer #(
        .DATA_WIDTH   (ACC_WIDTH), 
        .ROM_DWIDTH   (ROM_DWIDTH)
    ) c_mix (
        .clk          (clk), 
        .rst          (rst), 
        .buf_i        (buf_I_out), 
        .buf_q        (buf_Q_out), 
        .buf_valid    (stream_enable), 
        .nco_cos      (dopp_cos), 
        .nco_sin      (dopp_sin), 
        .mix_i        (mix_i), 
        .mix_q        (mix_q), 
        .mix_valid    (mix_valid)
    );


    // ==========================================
    // Инстанцирование FFT (Simulink IP)
    // ==========================================
    FFT fft_inst (
        .clk          (clk),
        .reset        (rst),
        .clk_enable   (1'b1),
        .In_FFt_re    (mix_i),
        .In_FFt_im    (mix_q),
        .In_vaild     (mix_valid),
        .rst          (rst),
        .ce_out       (),
        // .Out_FFt_re   (internal_fft_re),
        // .Out_FFt_im   (internal_fft_im),
        // .Out_FFT      (internal_fft_valid)

        .Out_FFt_re   (fft_out_re),      // Подключаем напрямую к выходу
        .Out_FFt_im   (fft_out_im),      // Подключаем напрямую к выходу
        .Out_FFT      (fft_ready)        // Используем твой готовый флаг
    );

    // Для отладки наружу выводим флаг валидности спектра FFT
//    assign fft_ready = internal_fft_valid;


    // ==========================================
    // Смеситель спектров (Эфир x ПСП)
    // ==========================================
    // spectrum_multiplier #(
    //     .FFT_WIDTH  (28),
    //     .GOLD_WIDTH (24),
    //     .OUT_WIDTH  (18),
    //     .SHIFT      (24),
    //     .ADDR_WIDTH (12)
    // ) spec_mix_inst (
    //     .clk            (clk),
    //     .rst            (rst),
    //     .fft_re_in      (internal_fft_re),
    //     .fft_im_in      (internal_fft_im),
    //     .fft_valid_in   (internal_fft_valid),
    //     .mix_out_re     (mix_out_re),
    //     .mix_out_im     (mix_out_im),
    //     .mix_out_valid  (mix_out_valid)
    // );

    spectrum_multiplier #(
        .FFT_WIDTH  (28),
        .GOLD_WIDTH (24),
        .OUT_WIDTH  (18),
        .SHIFT      (24),
        .ADDR_WIDTH (12)
    ) spec_mix_inst (
        .clk            (clk),
        .rst            (rst),
        .fft_re_in      (fft_out_re),    
        .fft_im_in      (fft_out_im),
        .fft_valid_in   (fft_ready),
        .mix_out_re     (mix_out_re),   
        .mix_out_im     (mix_out_im),
        .mix_out_valid  (mix_out_valid)
    );


    // ==========================================
    // Инстанцирование IFFT (Новый Simulink IP)
    // ==========================================
    // Подключаем порты по схеме из Code Interface Report
    IFFT ifft_inst (
        .clk          (clk),
        .reset        (rst),
        .clk_enable   (1'b1),
        
        .In1_re       (mix_out_re),
        .In1_im       (mix_out_im),
        .In2          (mix_out_valid),
        .In3          (rst),

        .ce_out       (),
        .Out1_re      (ifft_out_re),
        .Out1_im      (ifft_out_im),
        .Out2         (ifft_out_valid)
    );


    // ==========================================
    // Детектор Мощности (I^2 + Q^2)
    // ==========================================
    power_detector #(
        .IN_WIDTH     (18),
        .OUT_WIDTH    (37)
    ) pow_det (
        .clk          (clk),
        .rst          (rst),
        
        .ifft_re_in   (ifft_out_re),
        .ifft_im_in   (ifft_out_im),
        .ifft_valid_in(ifft_out_valid),
        
        .power_out    (power_out),
        .power_valid  (peak_ready)
    );

endmodule
