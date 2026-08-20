import os
import sys
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def run_subprocess(cmd):
    """Вспомогательная функция для запуска C++ бинарников (моделей Verilator)"""
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"\n[ОШИБКА] Процесс {' '.join(cmd)} завершился с кодом {e.returncode}")
        sys.exit(1)
    except FileNotFoundError:
        print(f"\n[ОШИБКА] Не найден исполняемый файл: {cmd[0]}")
        print("Убедись, что ты запустил сборку через 'make' в папке build/")
        sys.exit(1)


def main():
    print("===================================================")
    print("      GNSS RECEIVER AUTOMATED PIPELINE             ")
    print("===================================================")

    # Папка для всех сгенерированных картинок
    FIG_DIR = "figures"
    os.makedirs(FIG_DIR, exist_ok=True)

    # =========================================================================
    # СТАДИЯ 1: Coarse Acquisition (Грубый поиск)
    # =========================================================================
    print("\n>>> СТАДИЯ 1: Запуск Coarse Acquisition...")
    run_subprocess(["build/coarse_sim"])

    # =========================================================================
    # СТАДИЯ 2: Анализ результатов поиска
    # =========================================================================
    print("\n>>> СТАДИЯ 2: Анализ результатов поиска и построение графиков...")
    pow_path = "bin/power_2d.bin"
    spec_path = "bin/spectra.bin"

    if not os.path.exists(pow_path) or not os.path.exists(spec_path):
        print("Ошибка: Бинарные файлы не найдены! Проверь работу coarse_sim.")
        sys.exit(1)

    print("Чтение бинарных файлов...")

    # --- 1. Читаем матрицу мощности ---
    power_1d = np.fromfile(pow_path, dtype=np.uint64)
    FRAME_SIZE = 4096

    dopplers = np.arange(-5000, 5001, 500)
    num_doppler = len(dopplers)

    # Формируем 2D матрицу (21 x 4096)
    power_2d = power_1d.reshape((num_doppler, FRAME_SIZE)).astype(np.float64)

    # --- 2. Читаем спектры ---
    spectra_1d = np.fromfile(spec_path, dtype=np.int32)
    fft_re = spectra_1d[0:FRAME_SIZE]
    fft_im = spectra_1d[FRAME_SIZE : 2 * FRAME_SIZE]
    mix_re = spectra_1d[2 * FRAME_SIZE : 3 * FRAME_SIZE]
    mix_im = spectra_1d[3 * FRAME_SIZE : 4 * FRAME_SIZE]

    fft_complex = fft_re + 1j * fft_im
    mix_complex = mix_re + 1j * mix_im

    # Поиск координат глобального пика[cite: 5]
    best_doppler_idx, best_code_idx = np.unravel_index(
        np.argmax(power_2d), power_2d.shape
    )
    best_doppler = dopplers[best_doppler_idx]
    best_power = power_2d[best_doppler_idx, best_code_idx]
    print(
        f"Сигнал найден! Пик на Доплере {best_doppler} Гц, сдвиг {best_code_idx} (Мощность: {best_power:.2e})"
    )

    # Параметры спектров[cite: 5]
    FS_FFT_MHZ = 3.8192

    def calc_magnitude_db(complex_arr):
        mag = np.abs(complex_arr)
        mag_shifted = np.fft.fftshift(mag)
        return 20 * np.log10(np.maximum(mag_shifted, 1e-5))

    fft_db = calc_magnitude_db(fft_complex)
    mix_db = calc_magnitude_db(mix_complex)
    freqs = np.fft.fftshift(np.fft.fftfreq(FRAME_SIZE, d=1 / FS_FFT_MHZ))

    # =========================================================
    # FIGURE 1: Спектры (После FFT и После ПСП)[cite: 5]
    # =========================================================
    fig1, (ax1_1, ax1_2) = plt.subplots(1, 2, figsize=(14, 5))
    fig1.canvas.manager.set_window_title("Figure 1: Spectra Analysis")

    ax1_1.plot(freqs, fft_db, color="blue", linewidth=1)
    ax1_1.set_title("Спектр после FFT (Срез: 0 Гц)")
    ax1_1.set_ylabel("Амплитуда (дБ)")
    ax1_1.set_xlabel("Частота (МГц)")
    ax1_1.grid(True, linestyle="--", alpha=0.6)
    ax1_1.axvline(
        x=1.023, color="k", linestyle="--", alpha=0.5, label="Ширина лепестка C/A"
    )
    ax1_1.axvline(x=-1.023, color="k", linestyle="--", alpha=0.5)
    ax1_1.legend()

    ax1_2.plot(freqs, mix_db, color="green", linewidth=1)
    ax1_2.set_title("Спектр после умножения на ПСП (Срез: 0 Гц)")
    ax1_2.set_xlabel("Частота (МГц)")
    ax1_2.set_ylabel("Амплитуда (дБ)")
    ax1_2.grid(True, linestyle="--", alpha=0.6)
    ax1_2.axvline(x=1.023, color="k", linestyle="--", alpha=0.5)
    ax1_2.axvline(x=-1.023, color="k", linestyle="--", alpha=0.5)

    fig1.tight_layout()

    # =========================================================
    # FIGURE 2: Срез корреляционного пика по Доплеру[cite: 5]
    # =========================================================
    fig2 = plt.figure(figsize=(10, 5))
    fig2.canvas.manager.set_window_title("Figure 2: Doppler Slice Correlation Peak")

    ax2 = fig2.add_subplot(1, 1, 1)
    ax2.plot(
        power_2d[best_doppler_idx, :],
        color="blue",
        linewidth=1.5,
        label=f"Доплер: {best_doppler} Гц",
    )
    ax2.set_title(
        f"Срез корреляционной функции при оптимальном Доплере ({best_doppler} Гц)"
    )
    ax2.set_xlabel("Сдвиг кода (в отсчетах)")
    ax2.set_ylabel("Мощность (I² + Q²)")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend()

    fig2.tight_layout()

    # =========================================================
    # FIGURE 3: 3D Поверхность и 2D Тепловая карта[cite: 5]
    # =========================================================
    fig3 = plt.figure(figsize=(15, 6))
    fig3.canvas.manager.set_window_title("Figure 3: 3D & 2D Acquisition Matrix")

    ax3_1 = fig3.add_subplot(1, 2, 1, projection="3d")
    X, Y = np.meshgrid(np.arange(FRAME_SIZE), dopplers)
    surf = ax3_1.plot_surface(X, Y, power_2d, cmap="jet", edgecolor="none", alpha=0.9)
    ax3_1.set_title("3D Матрица быстрого поиска (PCPS)")
    ax3_1.set_xlabel("Сдвиг кода (отсчеты)")
    ax3_1.set_ylabel("Доплер (Гц)")
    ax3_1.set_zlabel("Мощность")
    ax3_1.view_init(elev=35, azim=-55)

    ax3_2 = fig3.add_subplot(1, 2, 2)
    im = ax3_2.imshow(
        power_2d,
        aspect="auto",
        extent=[0, FRAME_SIZE, dopplers[0], dopplers[-1]],
        origin="lower",
        cmap="jet",
    )
    ax3_2.set_title("2D Тепловая карта (Вид сверху)")
    ax3_2.set_xlabel("Сдвиг кода (отсчеты)")
    ax3_2.set_ylabel("Доплер (Гц)")
    fig3.colorbar(im, ax=ax3_2, fraction=0.046, pad=0.04)
    ax3_2.axhline(y=best_doppler, color="white", linestyle="--", alpha=0.8)
    ax3_2.axvline(x=best_code_idx, color="white", linestyle="--", alpha=0.8)

    fig3.tight_layout()

    # =========================================================
    # FIGURE 4: Зависимость максимального пика от частоты Доплера[cite: 5]
    # =========================================================
    fig4 = plt.figure(figsize=(10, 5))
    fig4.canvas.manager.set_window_title("Figure 4: Peak Power vs Doppler")

    ax4 = fig4.add_subplot(1, 1, 1)
    peak_vs_doppler = np.max(power_2d, axis=1)

    ax4.plot(
        dopplers,
        peak_vs_doppler,
        color="red",
        marker="o",
        linestyle="-",
        linewidth=2,
        markersize=6,
    )
    ax4.set_title("Зависимость величины корреляционного пика от частоты Доплера")
    ax4.set_xlabel("Доплеровский сдвиг (Гц)")
    ax4.set_ylabel("Максимальная мощность пика (I² + Q²)")
    ax4.grid(True, linestyle="--", alpha=0.6)
    ax4.axvline(
        x=best_doppler,
        color="blue",
        linestyle="--",
        alpha=0.7,
        label=f"Оптимальный Доплер: {best_doppler} Гц",
    )
    ax4.legend()

    fig4.tight_layout()

    print(
        "Все картинки поиска сохранены (fig1_spectra.png, fig2_doppler_slice.png, fig3_acquisition_3d_2d.png, fig4_peak_vs_doppler.png)"
    )

    # Сохраняем первые 4 графика в папку FIG_DIR
    fig1.savefig(os.path.join(FIG_DIR, "fig1_spectra.png"), dpi=300)
    fig2.savefig(os.path.join(FIG_DIR, "fig2_doppler_slice.png"), dpi=300)
    fig3.savefig(os.path.join(FIG_DIR, "fig3_acquisition_3d_2d.png"), dpi=300)
    fig4.savefig(os.path.join(FIG_DIR, "fig4_peak_vs_doppler.png"), dpi=300)

    # =========================================================================
    # СТАДИЯ 3: Tracking Channel (Слежение)[cite: 5]
    # =========================================================================
    print(f"\n>>> СТАДИЯ 3: Запуск Tracking Loop (Доплер: {best_doppler} Гц)...")
    run_subprocess(["build/tracking_sim", str(best_doppler), str(best_code_idx)])

    # =========================================================================
    # СТАДИЯ 4: Чтение результатов Tracking и добавление графиков[cite: 5]
    # =========================================================================
    print("\n>>> СТАДИЯ 4: Чтение данных слежения и финальная отрисовка...")
    track_bin_path = "bin/tracking_results.bin"

    if not os.path.exists(track_bin_path):
        print(f"[ОШИБКА] Файл {track_bin_path} не найден! Трекинг не отработал.")
        sys.exit(1)

    # [NEW] Обновляем структуру для чтения новых полей DLL
    tracking_dtype = np.dtype(
        [
            ("time_ms", np.uint32),
            ("i_p", np.int32),
            ("q_p", np.int32),
            ("state", np.int32),
            ("fine_tune_hz", np.float32),
            ("nco_adjust_reg", np.int32),
            ("fine_tune_code_hz", np.float32),  # [NEW] Поправка кода в Гц
            ("code_adjust_reg", np.int32),  # [NEW] Значение регистра NCO кода
        ]
    )

    tracking_data = np.fromfile(track_bin_path, dtype=tracking_dtype)

    if len(tracking_data) > 0:
        time_ms = tracking_data["time_ms"]
        i_p = tracking_data["i_p"]
        q_p = tracking_data["q_p"]
        fine_tune_hz = tracking_data["fine_tune_hz"]
        fine_tune_code_hz = tracking_data[
            "fine_tune_code_hz"
        ]  # [NEW] Вытаскиваем массив DLL

        # =========================================================
        # FIGURE 5: Tracking Loop (I_P, Q_P и подстройка частоты)[cite: 5]
        # =========================================================
        fig5, (ax5_1, ax5_2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        fig5.canvas.manager.set_window_title("Figure 5: GNSS Tracking Loop Results")

        # Выходы корреляторов
        ax5_1.plot(time_ms, i_p, label="I_P (In-Phase)", color="blue", linewidth=1.5)
        ax5_1.plot(
            time_ms,
            q_p,
            label="Q_P (Quadrature)",
            color="red",
            alpha=0.7,
            linewidth=1.5,
        )
        ax5_1.set_title(f"Корреляторы I_P и Q_P (Начальный Доплер: {best_doppler} Гц)")
        ax5_1.set_ylabel("Амплитуда корреляции")
        ax5_1.legend(loc="upper right")
        ax5_1.grid(True, linestyle="--", alpha=0.6)

        # Подстройка частоты
        ax5_2.plot(
            time_ms,
            fine_tune_hz,
            label="FLL/PLL Fine Tune (Hz)",
            color="green",
            linewidth=1.5,
        )
        ax5_2.set_title("Подстройка частоты несущей во времени (PLL/FLL)")
        ax5_2.set_xlabel("Время симуляции (мс)")
        ax5_2.set_ylabel("Поправка частоты (Гц)")

        states = tracking_data["state"]
        lock_idx = np.where(states == 1)[0]
        track_idx = np.where(states == 2)[0]

        if len(lock_idx) > 0 and lock_idx[0] > 0:
            ax5_2.axvline(
                x=time_ms[lock_idx[0]],
                color="orange",
                linestyle=":",
                label="FLL Locked",
            )
        if len(track_idx) > 0 and track_idx[0] > 0:
            ax5_2.axvline(
                x=time_ms[track_idx[0]],
                color="purple",
                linestyle=":",
                label="PLL Track",
            )

        ax5_2.legend(loc="upper right")
        ax5_2.grid(True, linestyle="--", alpha=0.6)

        fig5.tight_layout()
        fig5.savefig(os.path.join(FIG_DIR, "fig5_tracking.png"), dpi=300)

        # [NEW] =========================================================
        # [NEW] FIGURE 7: Переходный процесс кодовой петли (DLL)
        # [NEW] =========================================================
        fig7 = plt.figure(figsize=(10, 5))
        fig7.canvas.manager.set_window_title("Figure 7: DLL Code Tracking Transient")

        ax7 = fig7.add_subplot(1, 1, 1)
        ax7.plot(
            time_ms,
            fine_tune_code_hz,
            color="magenta",
            linewidth=1.5,
            label="DLL Fine Tune (Hz)",
        )

        ax7.set_title("Переходный процесс кодовой петли (Delay-Locked Loop)")
        ax7.set_xlabel("Время симуляции (мс)")
        ax7.set_ylabel("Поправка частоты кода (Гц)")
        ax7.grid(True, linestyle="--", alpha=0.6)

        # Добавим те же маркеры состояний, чтобы видеть, как DLL реагирует на смену режимов несущей
        if len(lock_idx) > 0 and lock_idx[0] > 0:
            ax7.axvline(
                x=time_ms[lock_idx[0]],
                color="orange",
                linestyle=":",
                label="FLL Locked",
            )
        if len(track_idx) > 0 and track_idx[0] > 0:
            ax7.axvline(
                x=time_ms[track_idx[0]],
                color="purple",
                linestyle=":",
                label="PLL Track",
            )

        ax7.legend(loc="upper right")
        fig7.tight_layout()

        fig7.savefig(os.path.join(FIG_DIR, "fig7_dll_tracking.png"), dpi=300)
        print("Сохранен график кодовой петли (fig7_dll_tracking.png)")

    else:
        print("[ВНИМАНИЕ] Файл слежения пуст. Графики трекинга не будут построены.")

    # =========================================================
    # FIGURE 6: Потактовое накопление коррелятора (Пила)[cite: 5]
    # =========================================================
    # tact_bin_path = "bin/correlation_tact.bin"
    # if os.path.exists(tact_bin_path):
    #     print(f"Чтение потактовых данных из {tact_bin_path}...")
    #     tact_dtype = np.dtype([
    #         ('tact',  np.uint32),
    #         ('i_sum', np.int32),
    #         ('q_sum', np.int32)
    #     ])
    #
    #     tact_data = np.fromfile(tact_bin_path, dtype=tact_dtype)
    #
    #     if len(tact_data) > 0:
    #         fig6 = plt.figure(figsize=(14, 8))
    #         fig6.canvas.manager.set_window_title('Figure 6: Tact-by-Tact Correlation Accumulation')
    #         ax6 = fig6.add_subplot(1, 1, 1)
    #
    #         # Переводим такты во время в миллисекундах (Частота выборки 38.192 МГц)
    #         time_ms_tact = tact_data['tact'] / 38192.0
    #
    #         # Строим графики как в Matlab
    #         ax6.plot(time_ms_tact, tact_data['i_sum'], label='I_P (Синфазный)', color='lime', linewidth=1)
    #         ax6.plot(time_ms_tact, tact_data['q_sum'], label='Q_P (Квадратурный)', color='blue', linewidth=1)
    #
    #         ax6.set_title("Функция АКФ (Внутреннее состояние аккумулятора)")
    #         ax6.set_xlabel("Время, мс")
    #         ax6.set_ylabel("Значение в аккумуляторе")
    #         ax6.legend(loc='upper left')
    #         ax6.grid(True, linestyle='--', alpha=0.6)
    #
    #         # Добавляем вертикальные линии сброса каждую 1 мс
    #         max_ms = int(np.ceil(time_ms_tact[-1]))
    #         for ms in range(1, max_ms + 1):
    #             ax6.axvline(x=ms, color='black', linestyle='--', alpha=0.7, linewidth=2)
    #             ax6.text(ms, ax6.get_ylim()[0], f' Сброс {ms} мс', rotation=90, verticalalignment='bottom')
    #
    #         fig6.tight_layout()
    #
    #         # [NEW] БЕЗОПАСНОЕ СОХРАНЕНИЕ: Сохраняем график здесь, внутри условия существования tact_data
    #         fig6.savefig(os.path.join(FIG_DIR, "fig6_tact_correlation.png"), dpi=300)
    #         print("Сохранен график потактового накопления (fig6_tact_correlation.png)")
    #     else:
    #         print("[ВНИМАНИЕ] Файл потактового дампа пуст.")
    # else:
    #     print(f"[ВНИМАНИЕ] Файл {tact_bin_path} не найден.")
    #
    # print(f"\nВсе графики успешно сохранены в папку '{FIG_DIR}/'!")

    # =========================================================================
    # ОТОБРАЖЕНИЕ ВСЕХ ГРАФИКОВ[cite: 5]
    # =========================================================================
    print("\nПайплайн завершен. Открываю все окна с графиками...")
    print("Закрой все окна графиков, чтобы завершить скрипт.")
    plt.show()


if __name__ == "__main__":
    main()
