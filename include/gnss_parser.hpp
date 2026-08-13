#pragma once

#include <iostream>
#include <cstdint>
#include <vector>

class NavDataParser {
private:
    uint32_t word_counter; 
    uint32_t current_subframe; 
    
    struct SubframeData {
        uint32_t tow_seconds;
        uint32_t subframe_id;
        std::vector<uint32_t> payload_words; 
    } current_data;

public:
    NavDataParser() : word_counter(0), current_subframe(0) {
        current_data.payload_words.reserve(8);
    }

    void process_word(uint32_t word) {
        word_counter++;

        // СЛОВО 1: TLM
        if (word_counter == 1) {
            current_data.payload_words.clear();
            return;
        }

        // СЛОВО 2: HOW (Извлекаем время)
        if (word_counter == 2) {
            uint32_t tow_count = (word >> 13) & 0x1FFFF;
            current_data.tow_seconds = tow_count * 1.5; 

            current_data.subframe_id = (word >> 8) & 0x07;
            current_subframe = current_data.subframe_id;

            std::cout << "\n[PARSER] --- Начало нового субкадра --- \n";
            std::cout << "[PARSER] Время TOW: " << current_data.tow_seconds << " сек\n";
            std::cout << "[PARSER] Subframe ID: " << current_subframe << "\n";
            return;
        }

        // СЛОВА 3-10: Полезная нагрузка
        if (word_counter >= 3 && word_counter <= 10) {
            uint32_t data_only = (word >> 6) & 0xFFFFFF;
            current_data.payload_words.push_back(data_only);

            if (word_counter == 10) {
                std::cout << "[PARSER] Субкадр " << current_subframe << " успешно прочитан (собрано 8 слов данных).\n";
                word_counter = 0; 
            }
        }
    }
};