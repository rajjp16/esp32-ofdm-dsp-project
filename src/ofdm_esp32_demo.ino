#include <Arduino.h>
#include <complex>
#include <cmath>

const int FFT_SIZE = 8;
const int CP_LEN = 2;
const int DATA_CARRIER_COUNT = 4;
const int PILOT_COUNT = 2;
const float PI = 3.14159265359f;

const int DATA_INDICES[DATA_CARRIER_COUNT] = {1, 2, 5, 6};
const int PILOT_INDICES[PILOT_COUNT] = {0, 4};
const std::complex<float> PILOT_SYMBOL(1.0f, 0.0f);

const std::complex<float> QAM16[16] = {
  std::complex<float>(-3.0f, -3.0f), std::complex<float>(-1.0f, -3.0f),
  std::complex<float>(1.0f, -3.0f),  std::complex<float>(3.0f, -3.0f),
  std::complex<float>(-3.0f, -1.0f), std::complex<float>(-1.0f, -1.0f),
  std::complex<float>(1.0f, -1.0f),  std::complex<float>(3.0f, -1.0f),
  std::complex<float>(-3.0f, 1.0f),  std::complex<float>(-1.0f, 1.0f),
  std::complex<float>(1.0f, 1.0f),   std::complex<float>(3.0f, 1.0f),
  std::complex<float>(-3.0f, 3.0f),  std::complex<float>(-1.0f, 3.0f),
  std::complex<float>(1.0f, 3.0f),   std::complex<float>(3.0f, 3.0f)
};

std::complex<float> nearestPoint(const std::complex<float>& x, const std::complex<float>* set, int setSize) {
  std::complex<float> nearest = set[0];
  float bestDistance = std::numeric_limits<float>::infinity();

  for (int i = 0; i < setSize; i++) {
    float d = std::norm(x - set[i]);
    if (d < bestDistance) {
      bestDistance = d;
      nearest = set[i];
    }
  }

  return nearest;
}

int findNearestIndex(const std::complex<float>& sym) {
  std::complex<float> nearest = nearestPoint(sym, QAM16, 16);
  for (int i = 0; i < 16; i++) {
    if (std::abs(QAM16[i] - nearest) < 0.0001f) {
      return i;
    }
  }
  return 0;
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

void generateRandomBits(bool* bits, int length) {
  for (int i = 0; i < length; i++) {
    bits[i] = (random(0, 2) == 1);
  }
}

void bitsToSymbols(const bool* bits, std::complex<float>* symbols, int symbolCount) {
  for (int s = 0; s < symbolCount; s++) {
    int idx = 0;
    for (int b = 0; b < 4; b++) {
      idx = (idx << 1) | (bits[s * 4 + b] ? 1 : 0);
    }
    symbols[s] = QAM16[idx];
  }
}

void applyRayleighFading(std::complex<float>* signal, int length) {
  std::complex<float> channel[4] = {
    std::complex<float>(0.95f, 0.15f),
    std::complex<float>(0.70f, -0.10f),
    std::complex<float>(0.38f, 0.12f),
    std::complex<float>(0.18f, -0.08f)
  };

  std::complex<float> temp[length];

  for (int i = 0; i < length; i++) {
    temp[i] = std::complex<float>(0.0f, 0.0f);
    for (int t = 0; t < 4; t++) {
      int idx = i - t;
      if (idx >= 0) {
        temp[i] += channel[t] * signal[idx];
      }
    }

    float phase = 0.25f * std::sin((float)i * 0.7f);
    std::complex<float> rot(std::cos(phase), std::sin(phase));
    signal[i] = temp[i] * rot;
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
    if (a[i] != b[i]) {
      errors++;
    }
  }
  return errors;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(analogRead(0));

  Serial.println("ESP32 OFDM Demo with Rayleigh fading channel");

  const int TOTAL_BITS = DATA_CARRIER_COUNT * 4;
  bool txBits[TOTAL_BITS];
  std::complex<float> txSymbols[DATA_CARRIER_COUNT];
  std::complex<float> freqDomain[FFT_SIZE];
  std::complex<float> timeDomain[FFT_SIZE];
  std::complex<float> txWithCP[FFT_SIZE + CP_LEN];
  std::complex<float> rxSignal[FFT_SIZE + CP_LEN];
  std::complex<float> rxNoCP[FFT_SIZE];
  std::complex<float> rxFreq[FFT_SIZE];
  std::complex<float> estChannel[FFT_SIZE];
  std::complex<float> receivedData[DATA_CARRIER_COUNT];
  bool rxBits[TOTAL_BITS];

  generateRandomBits(txBits, TOTAL_BITS);
  bitsToSymbols(txBits, txSymbols, DATA_CARRIER_COUNT);

  for (int i = 0; i < FFT_SIZE; i++) {
    freqDomain[i] = std::complex<float>(0.0f, 0.0f);
  }

  for (int i = 0; i < PILOT_COUNT; i++) {
    freqDomain[PILOT_INDICES[i]] = PILOT_SYMBOL;
  }

  for (int i = 0; i < DATA_CARRIER_COUNT; i++) {
    freqDomain[DATA_INDICES[i]] = txSymbols[i];
  }

  ifft(freqDomain, timeDomain);

  for (int i = 0; i < CP_LEN; i++) {
    txWithCP[i] = timeDomain[FFT_SIZE - CP_LEN + i];
  }
  for (int i = 0; i < FFT_SIZE; i++) {
    txWithCP[CP_LEN + i] = timeDomain[i];
  }

  for (int i = 0; i < FFT_SIZE + CP_LEN; i++) {
    rxSignal[i] = txWithCP[i];
  }

  applyRayleighFading(rxSignal, FFT_SIZE + CP_LEN);

  for (int i = 0; i < FFT_SIZE; i++) {
    rxNoCP[i] = rxSignal[CP_LEN + i];
  }

  fft(rxNoCP, rxFreq);

  for (int i = 0; i < FFT_SIZE; i++) {
    estChannel[i] = std::complex<float>(1.0f, 0.0f);
  }

  for (int p = 0; p < PILOT_COUNT; p++) {
    int idx = PILOT_INDICES[p];
    estChannel[idx] = rxFreq[idx] / PILOT_SYMBOL;
  }

  for (int i = 0; i < DATA_CARRIER_COUNT; i++) {
    int idx = DATA_INDICES[i];
    int leftPilot = -1;
    int rightPilot = -1;

    for (int p = 0; p < PILOT_COUNT; p++) {
      if (PILOT_INDICES[p] < idx) leftPilot = PILOT_INDICES[p];
      if (PILOT_INDICES[p] > idx) {
        rightPilot = PILOT_INDICES[p];
        break;
      }
    }

    if (leftPilot != -1 && rightPilot != -1) {
      estChannel[idx] = (estChannel[leftPilot] + estChannel[rightPilot]) / 2.0f;
    } else if (leftPilot != -1) {
      estChannel[idx] = estChannel[leftPilot];
    } else if (rightPilot != -1) {
      estChannel[idx] = estChannel[rightPilot];
    }
  }

  for (int i = 0; i < DATA_CARRIER_COUNT; i++) {
    int idx = DATA_INDICES[i];
    receivedData[i] = rxFreq[idx] / estChannel[idx];
  }

  for (int i = 0; i < DATA_CARRIER_COUNT; i++) {
    int symbolIndex = findNearestIndex(receivedData[i]);
    for (int b = 0; b < 4; b++) {
      rxBits[i * 4 + b] = (symbolIndex >> (3 - b)) & 1;
    }
  }

  Serial.print("Transmitted bits: ");
  printBits(txBits, TOTAL_BITS);

  Serial.print("Recovered bits: ");
  printBits(rxBits, TOTAL_BITS);

  int bitErrors = countBitErrors(txBits, rxBits, TOTAL_BITS);
  float ber = (float)bitErrors / (float)TOTAL_BITS;

  Serial.print("Bit errors: ");
  Serial.println(bitErrors);
  Serial.print("BER: ");
  Serial.println(ber, 6);
}

void loop() {
  delay(10000);
}
