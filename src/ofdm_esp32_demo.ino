#include <Arduino.h>
#include <complex>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>

#define FFT_SIZE 64
#define CP_LEN 16
#define ACTIVE_DATA_CARRIERS 48
#define PILOT_COUNT 8
#define MAX_FRAME_BITS 128

const float PI = 3.14159265359f;

enum ModulationMode {
  MOD_QPSK = 0,
  MOD_16QAM = 1,
  MOD_64QAM = 2
};

std::complex<float> qpskConst[4] = {
  std::complex<float>(1.0f,  1.0f),
  std::complex<float>(-1.0f, 1.0f),
  std::complex<float>(-1.0f, -1.0f),
  std::complex<float>(1.0f,  -1.0f)
};

std::complex<float> qam16Const[16] = {
  std::complex<float>(-3.0f, -3.0f), std::complex<float>(-1.0f, -3.0f),
  std::complex<float>(1.0f, -3.0f),  std::complex<float>(3.0f, -3.0f),
  std::complex<float>(-3.0f, -1.0f), std::complex<float>(-1.0f, -1.0f),
  std::complex<float>(1.0f, -1.0f),  std::complex<float>(3.0f, -1.0f),
  std::complex<float>(-3.0f, 1.0f),  std::complex<float>(-1.0f, 1.0f),
  std::complex<float>(1.0f, 1.0f),   std::complex<float>(3.0f, 1.0f),
  std::complex<float>(-3.0f, 3.0f),  std::complex<float>(-1.0f, 3.0f),
  std::complex<float>(1.0f, 3.0f),   std::complex<float>(3.0f, 3.0f)
};

std::complex<float> qam64Const[64] = {
  std::complex<float>(-7.0f, -7.0f), std::complex<float>(-5.0f, -7.0f), std::complex<float>(-3.0f, -7.0f), std::complex<float>(-1.0f, -7.0f),
  std::complex<float>(1.0f, -7.0f),  std::complex<float>(3.0f, -7.0f),  std::complex<float>(5.0f, -7.0f),  std::complex<float>(7.0f, -7.0f),
  std::complex<float>(-7.0f, -5.0f), std::complex<float>(-5.0f, -5.0f), std::complex<float>(-3.0f, -5.0f), std::complex<float>(-1.0f, -5.0f),
  std::complex<float>(1.0f, -5.0f),  std::complex<float>(3.0f, -5.0f),  std::complex<float>(5.0f, -5.0f),  std::complex<float>(7.0f, -5.0f),
  std::complex<float>(-7.0f, -3.0f), std::complex<float>(-5.0f, -3.0f), std::complex<float>(-3.0f, -3.0f), std::complex<float>(-1.0f, -3.0f),
  std::complex<float>(1.0f, -3.0f),  std::complex<float>(3.0f, -3.0f),  std::complex<float>(5.0f, -3.0f),  std::complex<float>(7.0f, -3.0f),
  std::complex<float>(-7.0f, -1.0f), std::complex<float>(-5.0f, -1.0f), std::complex<float>(-3.0f, -1.0f), std::complex<float>(-1.0f, -1.0f),
  std::complex<float>(1.0f, -1.0f),  std::complex<float>(3.0f, -1.0f),  std::complex<float>(5.0f, -1.0f),  std::complex<float>(7.0f, -1.0f),
  std::complex<float>(-7.0f, 1.0f),  std::complex<float>(-5.0f, 1.0f),  std::complex<float>(-3.0f, 1.0f),  std::complex<float>(-1.0f, 1.0f),
  std::complex<float>(1.0f, 1.0f),   std::complex<float>(3.0f, 1.0f),   std::complex<float>(5.0f, 1.0f),   std::complex<float>(7.0f, 1.0f),
  std::complex<float>(-7.0f, 3.0f),  std::complex<float>(-5.0f, 3.0f),  std::complex<float>(-3.0f, 3.0f),  std::complex<float>(-1.0f, 3.0f),
  std::complex<float>(1.0f, 3.0f),   std::complex<float>(3.0f, 3.0f),   std::complex<float>(5.0f, 3.0f),   std::complex<float>(7.0f, 3.0f),
  std::complex<float>(-7.0f, 5.0f),  std::complex<float>(-5.0f, 5.0f),  std::complex<float>(-3.0f, 5.0f),  std::complex<float>(-1.0f, 5.0f),
  std::complex<float>(1.0f, 5.0f),   std::complex<float>(3.0f, 5.0f),   std::complex<float>(5.0f, 5.0f),   std::complex<float>(7.0f, 5.0f),
  std::complex<float>(-7.0f, 7.0f),  std::complex<float>(-5.0f, 7.0f),  std::complex<float>(-3.0f, 7.0f),  std::complex<float>(-1.0f, 7.0f),
  std::complex<float>(1.0f, 7.0f),   std::complex<float>(3.0f, 7.0f),   std::complex<float>(5.0f, 7.0f),   std::complex<float>(7.0f, 7.0f)
};

const int pilotIndices[PILOT_COUNT] = {0, 8, 16, 24, 32, 40, 48, 56};
const int dataIndices[ACTIVE_DATA_CARRIERS] = {
  1,2,3,4,5,6,7,
  9,10,11,12,13,14,15,
  17,18,19,20,21,22,23,
  25,26,27,28,29,30,31,
  33,34,35,36,37,38,39,
  41,42,43,44,45,46,47,
  49,50,51,52,53,54,55,
  57,58,59,60,61,62,63
};

std::complex<float> pilotSymbols[PILOT_COUNT] = {
  std::complex<float>(1.0f, 0.0f),
  std::complex<float>(1.0f, 0.0f),
  std::complex<float>(1.0f, 0.0f),
  std::complex<float>(1.0f, 0.0f),
  std::complex<float>(1.0f, 0.0f),
  std::complex<float>(1.0f, 0.0f),
  std::complex<float>(1.0f, 0.0f),
  std::complex<float>(1.0f, 0.0f)
};

int modBitsPerSymbol(ModulationMode mode) {
  if (mode == MOD_QPSK) return 2;
  if (mode == MOD_16QAM) return 4;
  return 6;
}

std::complex<float> nearestPoint(const std::complex<float>& x, const std::complex<float>* set, int setSize) {
  std::complex<float> nearest = set[0];
  float best = std::numeric_limits<float>::infinity();
  for (int i = 0; i < setSize; i++) {
    float d = std::norm(x - set[i]);
    if (d < best) {
      best = d;
      nearest = set[i];
    }
  }
  return nearest;
}

int modulationIndexFromSNR(float snrDb) {
  if (snrDb < 12.0f) return MOD_QPSK;
  if (snrDb < 22.0f) return MOD_16QAM;
  return MOD_64QAM;
}

const char* modulationName(ModulationMode mode) {
  if (mode == MOD_QPSK) return "QPSK";
  if (mode == MOD_16QAM) return "16-QAM";
  return "64-QAM";
}

int getConstellationSize(ModulationMode mode) {
  if (mode == MOD_QPSK) return 4;
  if (mode == MOD_16QAM) return 16;
  return 64;
}

const std::complex<float>* getConstellation(ModulationMode mode) {
  if (mode == MOD_QPSK) return qpskConst;
  if (mode == MOD_16QAM) return qam16Const;
  return qam64Const;
}

void ifft(const std::complex<float>* in, std::complex<float>* out) {
  for (int n = 0; n < FFT_SIZE; n++) {
    std::complex<float> sum(0.0f, 0.0f);
    for (int k = 0; k < FFT_SIZE; k++) {
      float angle = -2.0f * PI * k * n / FFT_SIZE;
      std::complex<float> w(std::cos(angle), std::sin(angle));
      sum += in[k] * w;
    }
    out[n] = sum / static_cast<float>(FFT_SIZE);
  }
}

void fft(const std::complex<float>* in, std::complex<float>* out) {
  for (int k = 0; k < FFT_SIZE; k++) {
    std::complex<float> sum(0.0f, 0.0f);
    for (int n = 0; n < FFT_SIZE; n++) {
      float angle = -2.0f * PI * k * n / FFT_SIZE;
      std::complex<float> w(std::cos(angle), std::sin(angle));
      sum += in[n] * w;
    }
    out[k] = sum;
  }
}

void addMultipathNoise(std::complex<float>* rx, int length, float noisePower) {
  std::array<float, 4> h = {0.9f, 0.5f, 0.25f, 0.1f};
  std::complex<float> temp[length];
  for (int i = 0; i < length; i++) {
    temp[i] = std::complex<float>(0.0f, 0.0f);
    for (int t = 0; t < 4; t++) {
      int idx = i - t;
      if (idx >= 0) {
        temp[i] += h[t] * rx[idx];
      }
    }
  }

  for (int i = 0; i < length; i++) {
    float noiseReal = ((float)random(0, 1000) / 1000.0f - 0.5f) * 2.0f * noisePower;
    float noiseImag = ((float)random(0, 1000) / 1000.0f - 0.5f) * 2.0f * noisePower;
    rx[i] = temp[i] + std::complex<float>(noiseReal, noiseImag);
  }
}

void printBits(const bool* bits, int length) {
  for (int i = 0; i < length; i++) {
    Serial.print(bits[i] ? '1' : '0');
  }
  Serial.println();
}

int countBitErrors(const bool* a, const bool* b, int length) {
  int errors = 0;
  for (int i = 0; i < length; i++) {
    if (a[i] != b[i]) errors++;
  }
  return errors;
}

bool randomBit() {
  return random(0, 2) == 1;
}

void generateBits(bool* bits, int length) {
  for (int i = 0; i < length; i++) {
    bits[i] = randomBit();
  }
}

void bitsToQAMSymbols(const bool* bits, ModulationMode mode, std::complex<float>* symbols, int symbolCount) {
  const std::complex<float>* constellation = getConstellation(mode);
  int bitsPerSymbol = modBitsPerSymbol(mode);
  int constSize = getConstellationSize(mode);

  for (int s = 0; s < symbolCount; s++) {
    int idx = 0;
    for (int b = 0; b < bitsPerSymbol; b++) {
      idx = (idx << 1) | (bits[s * bitsPerSymbol + b] ? 1 : 0);
    }
    symbols[s] = constellation[idx % constSize];
  }
}

void symbolsToBits(const std::complex<float>* symbols, const std::complex<float>* constellation, int constellationSize, int bitCount, bool* bits) {
  for (int s = 0; s < bitCount; s++) {
    std::complex<float> nearest = nearestPoint(symbols[s], constellation, constellationSize);
    int idx = 0;
    for (int i = 0; i < constellationSize; i++) {
      if (constellation[i] == nearest) {
        idx = i;
        break;
      }
    }

    int bitIndex = 0;
    int tmp = idx;
    while (tmp > 0) {
      bits[s * 6 + bitIndex] = (tmp & 1) == 1;
      tmp >>= 1;
      bitIndex++;
    }
    while (bitIndex < 6) {
      bits[s * 6 + bitIndex] = false;
      bitIndex++;
    }
  }
}

void mapDataToFrequencyDomain(const std::complex<float>* dataSymbols, int symbolCount, std::complex<float>* freqDomain) {
  for (int i = 0; i < FFT_SIZE; i++) {
    freqDomain[i] = std::complex<float>(0.0f, 0.0f);
  }

  for (int i = 0; i < PILOT_COUNT; i++) {
    freqDomain[pilotIndices[i]] = pilotSymbols[i];
  }

  for (int i = 0; i < symbolCount; i++) {
    freqDomain[dataIndices[i]] = dataSymbols[i];
  }
}

void estimateChannelFromPilots(const std::complex<float>* rxFreq, std::complex<float>* H, int size) {
  for (int i = 0; i < FFT_SIZE; i++) {
    H[i] = std::complex<float>(1.0f, 0.0f);
  }

  for (int p = 0; p < PILOT_COUNT; p++) {
    int idx = pilotIndices[p];
    if (std::abs(pilotSymbols[p]) > 0.0001f) {
      H[idx] = rxFreq[idx] / pilotSymbols[p];
    }
  }
}

void interpolateChannel(const std::complex<float>* H, std::complex<float>* HInterp) {
  for (int i = 0; i < FFT_SIZE; i++) {
    HInterp[i] = std::complex<float>(1.0f, 0.0f);
  }

  for (int i = 0; i < ACTIVE_DATA_CARRIERS; i++) {
    int idx = dataIndices[i];
    int left = idx;
    int right = idx;
    while (left > 0 && std::find(std::begin(pilotIndices), std::end(pilotIndices), left) == std::end(pilotIndices)) {
      left--;
    }
    while (right < FFT_SIZE - 1 && std::find(std::begin(pilotIndices), std::end(pilotIndices), right) == std::end(pilotIndices)) {
      right++;
    }

    if (left == idx) {
      HInterp[idx] = H[idx + 1];
    } else if (right == idx) {
      HInterp[idx] = H[idx - 1];
    } else {
      HInterp[idx] = H[left];
    }
  }
}

std::vector<bool> demodulate(const std::complex<float>* symbols, ModulationMode mode) {
  const std::complex<float>* constellation = getConstellation(mode);
  int setSize = getConstellationSize(mode);
  int bitsPerSymbol = modBitsPerSymbol(mode);
  std::vector<bool> bits;

  for (int s = 0; s < ACTIVE_DATA_CARRIERS; s++) {
    std::complex<float> nearest = nearestPoint(symbols[s], constellation, setSize);
    int idx = 0;
    for (int i = 0; i < setSize; i++) {
      if (constellation[i] == nearest) {
        idx = i;
        break;
      }
    }

    for (int b = bitsPerSymbol - 1; b >= 0; b--) {
      bits.push_back((idx >> b) & 1);
    }
  }

  return bits;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(analogRead(0));

  Serial.println("ESP32 OFDM DSP Demonstration");

  const float snrDb = 18.0f;
  ModulationMode mode = static_cast<ModulationMode>(modulationIndexFromSNR(snrDb));
  int bitsPerSymbol = modBitsPerSymbol(mode);
  int numSymbols = ACTIVE_DATA_CARRIERS;
  int totalBits = numSymbols * bitsPerSymbol; 

  bool txBits[MAX_FRAME_BITS];
  std::complex<float> txSymbols[ACTIVE_DATA_CARRIERS];
  std::complex<float> freqDomain[FFT_SIZE];
  std::complex<float> timeDomain[FFT_SIZE];
  std::complex<float> txWithCP[FFT_SIZE + CP_LEN];
  std::complex<float> rxTime[FFT_SIZE + CP_LEN];
  std::complex<float> rxNoCP[FFT_SIZE];
  std::complex<float> rxFreq[FFT_SIZE];
  std::complex<float> H[FFT_SIZE];
  std::complex<float> HInterp[FFT_SIZE];
  std::complex<float> eqSymbols[ACTIVE_DATA_CARRIERS];

  generateBits(txBits, totalBits);
  Serial.print("Mode: ");
  Serial.println(modulationName(mode));
  Serial.print("Frame bits: ");
  Serial.println(totalBits);

  const std::complex<float>* constellation = getConstellation(mode);
  bitsToQAMSymbols(txBits, mode, txSymbols, numSymbols);

  mapDataToFrequencyDomain(txSymbols, numSymbols, freqDomain);
  ifft(freqDomain, timeDomain);

  for (int i = 0; i < CP_LEN; i++) {
    txWithCP[i] = timeDomain[FFT_SIZE - CP_LEN + i];
  }
  for (int i = 0; i < FFT_SIZE; i++) {
    txWithCP[CP_LEN + i] = timeDomain[i];
  }

  for (int i = 0; i < FFT_SIZE + CP_LEN; i++) {
    rxTime[i] = txWithCP[i];
  }
  addMultipathNoise(rxTime, FFT_SIZE + CP_LEN, 0.2f + (22.0f - snrDb) * 0.02f);

  for (int i = 0; i < FFT_SIZE; i++) {
    rxNoCP[i] = rxTime[CP_LEN + i];
  }

  fft(rxNoCP, rxFreq);
  estimateChannelFromPilots(rxFreq, H, FFT_SIZE);
  interpolateChannel(H, HInterp);

  for (int i = 0; i < ACTIVE_DATA_CARRIERS; i++) {
    int idx = dataIndices[i];
    eqSymbols[i] = rxFreq[idx] / HInterp[idx];
  }

  std::vector<bool> recoveredBits = demodulate(eqSymbols, mode);
  bool rxBits[MAX_FRAME_BITS];
  for (int i = 0; i < totalBits; i++) {
    rxBits[i] = recoveredBits[i];
  }

  int bitErrors = countBitErrors(txBits, rxBits, totalBits);
  float ber = (float)bitErrors / (float)totalBits;

  Serial.print("Transmitted bits: ");
  printBits(txBits, totalBits);
  Serial.print("Recovered bits: ");
  printBits(rxBits, totalBits);
  Serial.print("Bit errors: ");
  Serial.println(bitErrors);
  Serial.print("BER: ");
  Serial.println(ber, 6);

  Serial.println("\nDemo complete.");
}

void loop() {
  delay(10000);
}
