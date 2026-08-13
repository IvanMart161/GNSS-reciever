cmake_minimum_required(VERSION 3.10)
project(GNSS_Receiver_Pipeline)

# Находим Verilator в системе
find_package(Verilator REQUIRED)

# Находим Eigen3
find_package(Eigen3 REQUIRED)

# Устанавливаем стандарт C++ (важно для современных фич в тестбенчах)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================================================================
# 1. ТАРГЕТ: Coarse Acquisition (Поиск)
# =========================================================================
set(COARSE_RTL_SRCS
    rtl_coarse/top_acqu.sv
    rtl_coarse/buffer_loop.sv
    rtl_coarse/CIC_loop.sv
    rtl_coarse/mixer_loop.sv
    rtl_coarse/nco_loop.sv
    rtl_coarse/doppler_nco.sv
    rtl_coarse/complex_mixer.sv
    rtl_coarse/FFT/FFT.sv
    rtl_coarse/IFFT/IFFT.sv
    rtl_coarse/power_detector.sv
    rtl_coarse/spectrum_multiplier.sv
)

add_executable(coarse_sim sim/tb_coarse.cpp)

target_include_directories(coarse_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

verilate(coarse_sim
    SOURCES ${COARSE_RTL_SRCS}
    TOP_MODULE top_acqu
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/rtl_coarse/FFT 
                 ${CMAKE_CURRENT_SOURCE_DIR}/rtl_coarse/IFFT
                 ${CMAKE_CURRENT_SOURCE_DIR}/rtl_coarse
    VERILATOR_ARGS "--trace"
                   "-Wno-fatal"
                   "-Wno-UNUSEDSIGNAL"
                   "-Wno-SYNCASYNCNET"
                   "-Wno-PINCONNECTEMPTY"
                   "-Wno-TIMESCALEMOD"
    PREFIX Vtop_acqu
)

# =========================================================================
# 2. ТАРГЕТ: Tracking Channel (Слежение)
# =========================================================================
set(TRACKING_RTL_SRCS
    rtl_track/tracker_channel.sv
    rtl_track/code_nco.sv
    rtl_track/gold_generator.sv
    rtl_track/mixer.sv
    rtl_track/nco.sv
    rtl_track/corr_accumulator.sv
)

# # [NEW] Список заголовочных файлов C++
set(TRACKING_CPP_HDRS
    include/gnss_tracking.hpp
    include/gnss_ekf.hpp
    include/dll_loop.hpp # [NEW] Наша новая петля DLL
)

# # [NEW] Явно передаем и .cpp, и .hpp в add_executable
add_executable(tracking_sim 
    sim/tb_track.cpp
    ${TRACKING_CPP_HDRS} # [NEW] Добавили хедеры
)

# Указываем папку include/, чтобы компилятор знал, где искать #include "..."
target_include_directories(tracking_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Подключение Eigen[cite: 6]
target_link_libraries(tracking_sim PRIVATE Eigen3::Eigen)

verilate(tracking_sim
    SOURCES ${TRACKING_RTL_SRCS}
    TOP_MODULE tracker_channel
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/rtl_track
    VERILATOR_ARGS "--trace"
                   "-Wno-fatal"
                   "-Wno-UNUSEDSIGNAL"
                   "-Wno-SYNCASYNCNET"
                   "-Wno-PINCONNECTEMPTY"
                   "-Wno-TIMESCALEMOD"
    PREFIX Vtracker_channel
)

# =========================================================================
# ГИГАЧАД-АВТОМАТИЗАЦИЯ: Полный пайплайн
# =========================================================================
add_custom_target(run
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/sim/run_pipeline.py
    DEPENDS coarse_sim tracking_sim
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT ">>> Запуск автоматизированного пайплайна GNSS (Coarse -> Tracking -> Graphs)..."
)