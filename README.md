# GNSS Receiver Pipeline

This project provides a complete, end-to-end simulation environment for a GPS L1 C/A receiver. It bridges the gap between high-speed software signal generation and hardware-level digital signal processing. Designed for high performance, the pipeline handles everything from raw intermediate frequency (IF) binary generation to telemetry decoding and visual data analysis.
Key Features

    C++ Signal Generator: A high-speed baseband generator simulating Doppler shift, acceleration, thermal noise, and full 300-bit navigation subframes.

    RTL Coarse Acquisition: A hardware-accurate SystemVerilog module utilizing FFT-based search and CFAR thresholding for rapid satellite detection.

    RTL Tracking Loop: Closed-loop PLL/DLL channels implemented in SystemVerilog to accurately track code phase and carrier frequency dynamics.

    Automated Verification: A robust CMake and Verilator toolchain for rapid compiling, cycle-accurate RTL testing, and pipeline automation.

    Python Visualization: Automatic rendering of 2D/3D correlation peaks, Doppler spectra, and tracking loop states.

# Prerequisites

    C++ Compiler: Must support C++17. Apple Clang is natively supported and highly recommended for macOS (M1/M2/M3 Silicon).

    CMake: Version 3.10 or newer for build automation.

    Verilator: Required for compiling SystemVerilog testbenches into executable C++ models.

    Python 3: Requires the numpy and matplotlib packages for generating analysis graphs.

    Eigen3: Required for advanced matrix computations within the tracking testbenches.

# Build and Run Instructions

The project uses a unified Makefile wrapper over CMake to simplify execution commands.

    Configure and Generate: Run make gen to compile the C++ signal generator and instantly create the simulated binary test signal.

    Execute Pipeline: Run make run to launch the full Verilator simulation, process the IF signal through acquisition and tracking, and plot the final results.
