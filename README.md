# ESP32 OFDM Communication System

This repository contains an educational implementation of an OFDM-based digital communication system on the ESP32 platform. The design follows the architecture shown in the project block diagram:

- random bit generation
- adaptive modulation (QPSK / 16-QAM / 64-QAM)
- OFDM symbol generation with IFFT
- cyclic prefix insertion
- multipath + AWGN channel simulation
- FFT at the receiver
- pilot-based channel estimation
- equalization and demodulation
- BER computation

The project is intended for learning and demonstration. It is not a production-grade Wi-Fi or LTE transceiver, but it demonstrates the core DSP concepts used in real wireless systems.

## Features

- ESP32-compatible Arduino implementation
- OFDM symbol generation using IFFT
- Cyclic prefix insertion and removal
- Adaptive modulation selection
- Pilot signal insertion for channel estimation
- Equalizer-based symbol recovery
- BER calculation for system evaluation
- Suitable for academic projects and DSP demonstrations

## Project structure

- `src/ofdm_esp32_demo.ino` — ESP32 Arduino sketch
- `docs/OFDM_Project_Report.md` — project report and theory notes
- `LICENSE` — MIT license

## Hardware requirements

- ESP32 development board
- USB cable
- Optional external DAC/ADC for real analog demonstration

## Software requirements

- Arduino IDE or VS Code + PlatformIO
- ESP32 board support installed

## How the system works

1. Random bits are generated at the transmitter.
2. Bits are mapped to either QPSK, 16-QAM, or 64-QAM depending on the selected modulation mode.
3. Selected symbols are placed on data carriers, and pilot carriers are inserted.
4. The OFDM symbol is created using an IFFT.
5. A cyclic prefix is added.
6. A simulated wireless channel adds multipath distortion and additive Gaussian noise.
7. The receiver removes the cyclic prefix and applies FFT.
8. Pilot subcarriers are used to estimate the channel response.
9. Data carriers are equalized and demodulated.
10. Recovered bits are compared to transmitted bits to compute BER.

## Adaptive modulation logic

The project uses a simple SNR-based adaptive mode selection:

- QPSK for low SNR
- 16-QAM for medium SNR
- 64-QAM for high SNR

This can be changed manually during testing or extended to a more advanced channel measurement method.

## Uploading and running

1. Open the Arduino IDE.
2. Install ESP32 support if it is not already available.
3. Open `src/ofdm_esp32_demo.ino`.
4. Select the correct ESP32 board and COM port.
5. Compile and upload.
6. Open the Serial Monitor at 115200 baud.

## Example output

```text
Mode: QPSK
Frame bits: 128
Transmitted bits: 101010101010...
Received bits: 101010101010...
BER: 0.000000
```

## Notes

This implementation intentionally keeps the core DSP logic understandable and portable. It can be expanded to include:

- real I2S DAC/ADC signal exchange
- real-time Wi-Fi or SDR-based transmission
- advanced equalization techniques
- channel coding and interleaving
- BER vs SNR plotting in Python or MATLAB

## License

MIT
