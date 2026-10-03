# ESP32 OFDM Communication System

This repository documents an OFDM-based digital communication system implemented on the ESP32 platform. The implementation is designed as a learning project and uses simplified, software-based DSP blocks that can be run in the Arduino environment.

## 1. Project objectives

- Understand OFDM signal generation and reception
- Implement IFFT/FFT based processing in software
- Simulate a wireless channel using multipath and AWGN
- Use pilot subcarriers for channel estimation
- Evaluate system performance using BER
- Demonstrate adaptive modulation using QPSK, 16-QAM, and 64-QAM

## 2. System overview

The system follows the architecture below:

```text
TRANSMITTER
  ESP32
    |
    v
  Random bit generator
    |
    v
  Adaptive modulation (QPSK / 16-QAM / 64-QAM)
    |
    v
  OFDM subcarrier mapping
    |
    v
  64-point IFFT
    |
    v
  Cyclic prefix insertion
    |
    v
  Channel (multipath + AWGN)
    |
    v
RECEIVER
  ESP32
    |
    v
  Remove cyclic prefix
    |
    v
  64-point FFT
    |
    v
  Pilot-based channel estimation
    |
    v
  Equalizer
    |
    v
  Demodulator
    |
    v
  BER calculation
```

## 3. Mathematical background

OFDM transmits data by splitting it over multiple orthogonal subcarriers. The time-domain signal is generated using the inverse FFT:

x[n] = (1/N) sum_{k=0}^{N-1} X[k] e^{j2πkn/N}

The receiver applies FFT to recover the frequency-domain symbols:

X[k] = sum_{n=0}^{N-1} x[n] e^{-j2πkn/N}

A cyclic prefix is inserted to mitigate inter-symbol interference caused by multipath propagation.

## 4. Channel model

The simulated channel includes:

- multipath fading
- additive white Gaussian noise (AWGN)
- frequency-selective distortion

The received signal is written as:

y[n] = h[n] * x[n] + w[n]

where h[n] is the channel impulse response and w[n] is noise.

## 5. Adaptive modulation

The transmitter chooses an appropriate modulation method based on a simple SNR estimate:

- low SNR → QPSK
- medium SNR → 16-QAM
- high SNR → 64-QAM

This demonstrates the concept used in real wireless systems for efficient spectrum utilization.

## 6. Pilot-based channel estimation

Pilot carriers are inserted into the OFDM symbol. At the receiver, the estimated channel response is determined as:

H[k] = Y_pilot[k] / X_pilot[k]

Then the data carriers are equalized using:

X_hat[k] = Y_data[k] / H[k]

## 7. BER performance metric

The bit error rate is computed as:

BER = number of incorrect bits / total transmitted bits

This metric helps evaluate how channel impairments affect system reliability.

## 8. Implementation note

This repository uses a small FFT size for the ESP32 to keep the code manageable and educational. In a production implementation, a larger FFT size such as 64 or 128 would be more representative of practical OFDM systems.

## 9. Future extensions

- real DAC/ADC signal transmission
- SDR-based wireless testbed
- advanced channel estimation methods
- forward error correction
- real-time BER vs SNR plots

## 10. Conclusion

This project provides a practical and understandable way to learn OFDM, DSP, modulation, channel effects, and BER analysis using the ESP32 platform. It is a strong academic project for communication engineering and signal processing courses.
