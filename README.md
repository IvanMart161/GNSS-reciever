# Hardware-Software GNSS Receiver 

```mermaid
<p align="center">
  <img src="docs/Schem.png" alt="Functional Architecture" width="100%">
</p>

## About the Project

This project presents a hardware-software GNSS receiver engineered to operate under the extreme dynamic environments typical of Low Earth Orbit (LEO) spacecraft.

The core distinguishing feature and primary engineering value of this project lies in the application of an advanced HW/SW co-design methodology. Instead of the classical, bottlenecked approach of isolating CAD logic simulation from abstract mathematical scripting, the entire end-to-end processing pipeline is unified within a single, high-speed ecosystem. High-performance SystemVerilog modules are compiled via Verilator into cycle-accurate C++ models and seamlessly integrated with modular C++ DSP libraries (.hpp). This architecture allows for simulating raw silicon behavior within a closed control loop at incredible speeds.

## Complete Processing Pipeline

The processing chain covers the entire lifecycle of a navigation signal:

- High-Dynamics Signal Generation: A custom C++ IF signal generator (9.548 MHz IF, 38.192 MHz Fs) synthesizes navigation BPSK modulation and thermal noise (SNR -25 dB). It injects a radial Doppler acceleration of -50 Hz/s to accurately simulate the geometry of a high-speed satellite pass.

- Hardware-Accelerated Acquisition: The SystemVerilog layer implements a Parallel Code Phase Search (PCPS) based on fast Fourier transforms (FFT/IFFT). To guarantee robust signal detection in high-noise environments, an adaptive CFAR threshold module based on the statistical Neyman-Pearson criterion is integrated.

- Closed-Loop Tracking & EKF: All clock-critical components (Carrier/Code NCOs, Gold PRN generators, Early/Prompt/Late quadrature correlators) operate strictly at the SystemVerilog hardware level. Their control is delegated to a multi-stage C++ controller that smoothly transitions the receiver from a Frequency Lock Loop (FLL) to a Phase Lock Loop (PLL), augmented by an Extended Kalman Filter (EKF) to compensate for massive dynamic frequency and phase drift.

- Software Demodulator Stack: The software backend performs bit synchronization using a histogram method, automatically resolves BPSK constellation phase ambiguity, acquires the frame preamble, and parses the navigation data word-by-word (TOW, subframe ID, and ephemeris).

## Conclusion

The result is a complete, end-to-end digital platform that proves the receiver's viability from the logic gate level up to clean navigation frame extraction under space-grade dynamic stress.
