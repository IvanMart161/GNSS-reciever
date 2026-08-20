%% =========================================================================
% Генерация GPS сигнала на ПЧ 9.548 МГц с шумом и динамикой
% =========================================================================
clear; clc; close all;

fs = 38.192e6;         % Частота дискретизации АЦП (38.192 МГц)
f_if = 9.548e6;        % Промежуточная частота (9.548 МГц)
f_doppler = 521;       % Начальная частота Доплера несущей (Гц)
f_dot = -50;           % Ускоререние Доплера (Гц/с)
f_chip = 1.023e6;      % Номинальная скорость C/A кода (Гц)
f_L1 = 1575.42e6;      % Частота несущей GPS L1 (Гц)
T_sim = 8;             % Длительность (8 c)
t = 0 : 1/fs : T_sim - 1/fs;
N = length(t);

G1 = ones(1, 10); G2 = ones(1, 10);
PRN_code = zeros(1, 1023);
for i = 1:1023
    PRN_code(i) = mod(G1(10) + mod(G2(2) + G2(6), 2), 2);
    G1 = [mod(G1(3) + G1(10), 2), G1(1:9)];
    G2 = [mod(G2(2) + G2(3) + G2(6) + G2(8) + G2(9) + G2(10), 2), G2(1:9)];
end
PRN_bpsk = 1 - 2 * PRN_code; % BPSK: 0->1, 1->-1


delay_seconds = 0 / f_chip; 

t_eff = max(t - delay_seconds, 0); 
code_phase = t_eff * f_chip + ...
             (f_chip / f_L1) * (f_doppler * t_eff + 0.5 * f_dot * t_eff.^2);

chip_indices = mod(floor(code_phase), 1023) + 1;
baseband_signal = PRN_bpsk(chip_indices);


%% =========================================================================
% ФОРМИРОВАНИЕ НАВИГАЦИОННОГО СУБКАДРА (50 бит/с)
% =========================================================================

% 1. Мусорные биты (50 бит)
sync_preamble = repmat([1, 0], 1, 50);

% 2. Генерация 10 слов по 30 бит
word1_val = bitshift(139, 22); 
word1_bits = dec2bin(word1_val, 30) - '0';

tow_val = 100000;  
subframe_id = 2;   
word2_val = bitshift(tow_val, 13) + bitshift(subframe_id, 8);
word2_bits = dec2bin(word2_val, 30) - '0';

payload_bits = [];
for i = 3:10
    val = bitshift(i, 6); 
    payload_bits = [payload_bits, dec2bin(val, 30) - '0'];
end

% Склеиваем все вместе (30 бит преамбулы + 300 бит субкадра)
full_bin_msg = [sync_preamble, word1_bits, word2_bits, payload_bits];

% Модуляция BPSK (0 -> +1, 1 -> -1)
nav_data_bpsk = 1 - 2 * full_bin_msg; 

% Растягиваем каждый бит на 20 мс
samples_per_bit = round(20e-3 * fs);
expanded_data = repelem(nav_data_bpsk, samples_per_bit);

% Выравниваем массив под вектор времени t
if length(expanded_data) >= N
    data_mod = expanded_data(1:N);
else
    data_mod = [expanded_data, ones(1, N - length(expanded_data))];
end


%% Перенос на ПЧ с динамическим Доплером

carrier_phase = 2 * pi * (f_if .* t + f_doppler .* t + 0.5 * f_dot .* t.^2);
carrier = cos(carrier_phase);

clean_if_signal = baseband_signal .* data_mod .* carrier;

%% Задаем ОСШ
SNR_db = -20; 

signal_power = mean(clean_if_signal.^2);
SNR_linear = 10^(SNR_db / 10);
noise_power = signal_power / SNR_linear;
noise = sqrt(noise_power) * randn(size(clean_if_signal));

if_signal = clean_if_signal + noise;

%% =========================================================================
% Моделирование ПАВ-фильтра (SAW Filter)
% =========================================================================
f_pass_low  = f_if - 2.0e6; % 7.548 МГц
f_pass_high = f_if + 2.0e6; % 11.548 МГц

[b_saw, a_saw] = butter(4, [f_pass_low, f_pass_high] / (fs/2), 'bandpass');
if_signal = filter(b_saw, a_saw, if_signal);


%% =========================================================================
% Жесткое квантование сигнала через 1-битный АЦП (0 и 1)
% =========================================================================
adc_1bit = uint8(if_signal > 0);

disp('Сохранение 1-битного сигнала АЦП в gps_if_signal.bin...');
fid = fopen('../bin/gps_if_signal.bin', 'wb');
if fid == -1
    error('Не удалось создать файл gps_if_signal.bin');
end
fwrite(fid, adc_1bit, 'uint8');
fclose(fid);
disp('Готово!');