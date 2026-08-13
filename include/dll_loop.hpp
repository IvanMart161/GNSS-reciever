// #pragma once

// #define _USE_MATH_DEFINES 
// #include <cmath> 
// #include <algorithm>
// // gnss_tracking.hpp здесь подключать не обязательно, чтобы не было циклической зависимости

// class DelayLockedLoop { 
// private: 
//     float Kp;
//     float Ki;
//     float code_integrator;
    
//     // [NEW] Отношение скорости C/A кода к частоте несущей L1 (1.023 МГц / 1575.42 МГц)
//     static constexpr float CARRIER_TO_CODE_RATIO = 1.0f / 1540.0f;

// public: 
//     // bw — шумовая полоса (например, 1.0 - 2.0 Гц)
//     // damp — коэффициент демпфирования (обычно 0.707)
//     // per — период обновления (1 мс = 0.001 с) 
//     DelayLockedLoop(float bw, float damp, float per) { 
//         // [NEW] Используем переданные в конструктор имена переменных bw и damp
//         float wn = bw * 8.0f * damp / (4.0f * damp * damp + 1.0f);  

//         Kp = 2.0f * damp * wn;
//         Ki = wn * wn * per;

//         code_integrator = 0.0f;
//     }

//     // Обновление петли 
//     // I_E, Q_E - ранний код (Early)
//     // I_L, Q_L - поздний код (Late)
//     // carrier_doppler_hz - текущая оценка частоты смещения от PLL/FLL
//     float update(float I_E, float Q_E, float I_L, float Q_L, float carrier_doppler_hz) { 
//         // Вычисление мощности
//         float E_power = I_E * I_E + Q_E * Q_E;
//         float L_power = I_L * I_L + Q_L * Q_L;

//         // Дискриминатор NEMLP
//         float sum_power = E_power + L_power;
//         float tau_err = 0.0f;
        
//         // Защита от деления на ноль (на случай полного отсутствия сигнала)
//         if (sum_power > 1e-6f) {
//             tau_err = (E_power - L_power) / sum_power;
//         }

//         // Петлевой фильтр 2-го порядка (PI-регулятор)
//         code_integrator += tau_err * Ki;
//         float code_filter_out = tau_err * Kp + code_integrator;

//         // Помощь от несущей (Carrier Aiding)
//         // Масштабируем доплеровский сдвиг несущей в сдвиг частоты кода
//         float carrier_aiding_hz = carrier_doppler_hz * CARRIER_TO_CODE_RATIO;

//         // Итоговая поправка для Code NCO (чипов в секунду)
//         return code_filter_out + carrier_aiding_hz;
//     }

//     void reset() {
//         code_integrator = 0.0f;
//     }

//     float get_integrator_state() const { return code_integrator; }

// }; // [NEW] Обязательная точка с запятой в конце класса!

#pragma once

#define _USE_MATH_DEFINES 
#include <cmath> 
#include <algorithm>

class DelayLockedLoop { 
private: 
    float Kp;
    float Ki;
    float code_integrator;
    
    // Сохраняем параметры для динамического пересчета
    float damp;
    float per;
    
    static constexpr float CARRIER_TO_CODE_RATIO = 1.0f / 1540.0f;

public: 
    DelayLockedLoop(float initial_bw, float damp_val, float per_val) 
        : damp(damp_val), per(per_val), code_integrator(0.0f) { 
        // Первичная инициализация
        set_bandwidth(initial_bw);
    }

    // [NEW] Функция переключения шумовой полосы "на лету"
    void set_bandwidth(float bw) {
        float wn = bw * 8.0f * damp / (4.0f * damp * damp + 1.0f);  
        Kp = 2.0f * damp * wn;
        Ki = wn * wn * per;
    }

    float update(float I_E, float Q_E, float I_L, float Q_L, float carrier_doppler_hz) { 
        float E_power = I_E * I_E + Q_E * Q_E;
        float L_power = I_L * I_L + Q_L * Q_L;

        float sum_power = E_power + L_power;
        float tau_err = 0.0f;
        
        if (sum_power > 1e-6f) {
            tau_err = (E_power - L_power) / sum_power;
        }

        // Интегратор накапливает ошибку плавно, без скачков при смене Ki
        code_integrator += tau_err * Ki;
        float code_filter_out = tau_err * Kp + code_integrator;

        float carrier_aiding_hz = carrier_doppler_hz * CARRIER_TO_CODE_RATIO;
        return code_filter_out + carrier_aiding_hz;
    }

    void reset() {
        code_integrator = 0.0f;
    }

    float get_integrator_state() const { return code_integrator; }
};