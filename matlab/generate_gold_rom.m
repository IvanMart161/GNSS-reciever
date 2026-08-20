%% =========================================================================
% Генерация обычного спектра ПСП (Gold Code) для ROM-таблицы (24 бита)
% =========================================================================
clear; clc; close all;

f_chip = 1.023e6;             % Скорость C/A кода (1.023 МГц)
fs_adc = 38.192e6;            % Исходная частота АЦП
decimation = 10;              % Децимация CIC-фильтра
fs_fft = fs_adc / decimation; % Частота перед FFT (3.8192 МГц)
fft_size = 4096;              % Размер окна БПФ

disp(['Частота дискретизации ПСП: ', num2str(fs_fft/1e6), ' МГц']);

% Генерация C/A кода (PRN 1)
G1 = ones(1, 10); G2 = ones(1, 10);
PRN_code = zeros(1, 1023);
for i = 1:1023
    PRN_code(i) = mod(G1(10) + mod(G2(2) + G2(6), 2), 2);
    G1 = [mod(G1(3) + G1(10), 2), G1(1:9)];
    G2 = [mod(G2(2) + G2(3) + G2(6) + G2(8) + G2(9) + G2(10), 2), G2(1:9)];
end
PRN_bpsk = 1 - 2 * PRN_code; % BPSK: 0 -> 1, 1 -> -1

% Растягиваем код на частоту дискретизации FFT 
t = (0 : fft_size - 1) / fs_fft;
chip_indices = mod(floor(t * f_chip), 1023) + 1;
baseband_signal = PRN_bpsk(chip_indices);

% Выполняем ПРЯМОЙ (не сопряженный) FFT
gold_fft = fft(baseband_signal);

% Масштабирование до 24 бит с безопасным коэффициентом 100
scale_factor = 100;
gold_re_scaled = round(real(gold_fft) * scale_factor);
gold_im_scaled = round(imag(gold_fft) * scale_factor);

% Анонимная функция для перевода в 24-битный HEX (дополнительный код)
to_hex_24 = @(x) dec2hex(mod(x + 2^24, 2^24), 6);

% Запись в HEX-файлы для SystemVerilog
fileID_re = fopen('../bin/gold_spec_re.hex', 'w');
fileID_im = fopen('../bin/gold_spec_im.hex', 'w');

for i = 1:fft_size
    fprintf(fileID_re, '%s\n', to_hex_24(gold_re_scaled(i)));
    fprintf(fileID_im, '%s\n', to_hex_24(gold_im_scaled(i)));
end

fclose(fileID_re);
fclose(fileID_im);

disp('✅ Файлы gold_spec_re.hex и gold_spec_im.hex успешно сгенерированы!');

%% =========================================================================
% Визуализация сгенерированных данных
% =========================================================================
figure('Name', 'Анализ опорного спектра ПСП (Gold Code PRN 1)', 'Position', [100, 100, 1000, 800]);

% --- График 1: Временная область ---
subplot(3,1,1);
plot(t(1:300)*1e6, baseband_signal(1:300), 'b', 'LineWidth', 1.5);
title('Опорный C/A код во времени (фрагмент 1-й эпохи)');
xlabel('Время (\muс)');
ylabel('Амплитуда');
ylim([-1.5 1.5]);
grid on;

% --- График 2: Амплитудный спектр в дБ ---
subplot(3,1,2);
freqs = ( -fft_size/2 : fft_size/2 - 1 ) * (fs_fft / fft_size) / 1e6;
gold_fft_shifted = fftshift(gold_fft);
magnitude_db = 20 * log10(abs(gold_fft_shifted));

plot(freqs, magnitude_db, 'r', 'LineWidth', 1.5);
hold on;
xline(1.023, '--k', 'Ширина C/A (1.023 МГц)', 'LabelVerticalAlignment', 'bottom');
xline(-1.023, '--k');
title('Идеальный спектр C/A кода (без шума)');
xlabel('Частота (МГц относительно центра)');
ylabel('Амплитуда (дБ)');
grid on;

% --- График 3: Реальная и Мнимая части для ROM ---
subplot(3,1,3);
plot(gold_re_scaled, 'Color', [0 0.5 0], 'DisplayName', 'Real (24 bit)');
hold on;
plot(gold_im_scaled, 'Color', [0.85 0.33 0.1], 'DisplayName', 'Imag (24 bit)');
title('Данные для записи в ROM ПЛИС');
xlabel('Индекс бина FFT (0 - 4095)');
ylabel('Целочисленное значение');
legend('show', 'Location', 'northeast');
grid on;