/*
  ESP32-C3 + WS2812B — СВІТЛОМУЗИКА

  Режим перемикається ОДНИМ прапорцем нижче:
    USE_MICROPHONE 1  -> реальна світломузика (I2S мікрофон INMP441 + FFT)
    USE_MICROPHONE 0  -> демо-режим: 17 світлових ефектів по колу, 30 сек кожен
                         (мікрофон не потрібен, зручно для відладки стрічки/живлення)

  Бібліотеки (Library Manager):
    - FastLED (>=3.7.0)          -- завжди потрібна
    - ArduinoFFT (>=2.0.0)       -- потрібна лише якщо USE_MICROPHONE = 1

  Піни (зміни під свою плату):
    LED_PIN    = 4   -> DIN стрічки (через резистор ~330 Ом)
    I2S_SCK    = 6   -> SCK мікрофона     (лише якщо USE_MICROPHONE = 1)
    I2S_WS     = 7   -> WS (LRCL) мікрофона
    I2S_SD     = 5   -> SD (DOUT) мікрофона
*/

// ======================= ГОЛОВНИЙ ПЕРЕМИКАЧ =======================
#define USE_MICROPHONE 0   // 1 = світломузика з мікрофоном, 0 = демо 17 ефектів
// ===================================================================

#include <FastLED.h>

#if USE_MICROPHONE
  #include <driver/i2s.h>
  #include <ArduinoFFT.h>
#endif

// ---------- НАЛАШТУВАННЯ СТРІЧКИ ----------
#define LED_PIN     4
#define NUM_LEDS    60          // <-- кількість діодів у стрічці
#define BRIGHTNESS  120         // 0-255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
uint8_t gHue = 0;

#if USE_MICROPHONE
// ================================================================
//                     РЕЖИМ: СВІТЛОМУЗИКА (МІКРОФОН)
// ================================================================

#define I2S_WS      7
#define I2S_SD      5
#define I2S_SCK     6
#define I2S_PORT    I2S_NUM_0

#define SAMPLES         512
#define SAMPLING_FREQ   40000
#define NUM_BANDS       8

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);

float bandValues[NUM_BANDS];
float bandPeaks[NUM_BANDS];
uint8_t hueBaseMic = 0;

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void readAudioAndFFT() {
  size_t bytesRead = 0;
  static int32_t buffer32[SAMPLES];

  i2s_read(I2S_PORT, buffer32, sizeof(buffer32), &bytesRead, portMAX_DELAY);
  int samplesRead = bytesRead / sizeof(int32_t);

  for (int i = 0; i < samplesRead; i++) {
    int32_t sample = buffer32[i] >> 14;
    vReal[i] = (double)sample;
    vImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  float freqPerBin = (float)SAMPLING_FREQ / SAMPLES;
  int usableBins = SAMPLES / 2;

  float minFreq = 60.0;
  float maxFreq = SAMPLING_FREQ / 2.0;
  float logMin = log(minFreq);
  float logMax = log(maxFreq);

  for (int b = 0; b < NUM_BANDS; b++) {
    float f0 = exp(logMin + (logMax - logMin) * b / NUM_BANDS);
    float f1 = exp(logMin + (logMax - logMin) * (b + 1) / NUM_BANDS);
    int bin0 = max(1, (int)(f0 / freqPerBin));
    int bin1 = min(usableBins - 1, (int)(f1 / freqPerBin));

    float sum = 0;
    int count = 0;
    for (int i = bin0; i <= bin1; i++) {
      sum += vReal[i];
      count++;
    }
    float avg = count > 0 ? sum / count : 0;
    bandValues[b] = constrain(avg / 4000.0, 0.0, 1.0);
  }
}

void renderSpectrum() {
  int ledsPerBand = NUM_LEDS / NUM_BANDS;

  for (int b = 0; b < NUM_BANDS; b++) {
    if (bandValues[b] > bandPeaks[b]) {
      bandPeaks[b] = bandValues[b];
    } else {
      bandPeaks[b] -= 0.03;
      if (bandPeaks[b] < 0) bandPeaks[b] = 0;
    }

    int litLeds = (int)(bandPeaks[b] * ledsPerBand);
    uint8_t hue = hueBaseMic + b * (255 / NUM_BANDS);

    for (int i = 0; i < ledsPerBand; i++) {
      int idx = b * ledsPerBand + i;
      if (idx >= NUM_LEDS) continue;

      if (i < litLeds) {
        uint8_t val = map(i, 0, ledsPerBand, 120, 255);
        leds[idx] = CHSV(hue, 255, val);
      } else {
        leds[idx] = CRGB::Black;
      }
    }
  }
  hueBaseMic += 1;
}

#else
// ================================================================
//                  РЕЖИМ: ДЕМО 17 ЕФЕКТІВ (БЕЗ МІКРОФОНА)
// ================================================================

const unsigned long EFFECT_DURATION = 30000; // 30 сек на ефект

void addGlitter(fract8 chanceOfGlitter) {
  if (random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

void fxRainbowCycle() {
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
  gHue++;
}

void fxRainbowGlitter() {
  fxRainbowCycle();
  addGlitter(80);
}

void fxConfetti() {
  fadeToBlackBy(leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV(gHue + random8(64), 200, 255);
  gHue++;
}

void fxSinelon() {
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS - 1);
  leds[pos] += CHSV(gHue, 255, 192);
  gHue++;
}

void fxBpm() {
  uint8_t bpm = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(bpm, 64, 255);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
  gHue++;
}

void fxJuggle() {
  fadeToBlackBy(leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for (int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void fxTheaterChase() {
  static unsigned long lastUpdate = 0;
  static int q = 0;
  if (millis() - lastUpdate < 50) return;
  lastUpdate = millis();

  fadeToBlackBy(leds, NUM_LEDS, 255);
  for (int i = 0; i < NUM_LEDS; i += 3) {
    int idx = i + q;
    if (idx < NUM_LEDS) leds[idx] = CHSV(gHue, 255, 255);
  }
  q = (q + 1) % 3;
  gHue += 3;
}

void fxColorWipe() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  static uint8_t colorIndex = 0;
  static const CRGB colors[] = {CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Yellow, CRGB::Cyan, CRGB::Magenta};

  if (millis() - lastUpdate < 30) return;
  lastUpdate = millis();

  leds[pos] = colors[colorIndex % 6];
  pos++;
  if (pos >= NUM_LEDS) {
    pos = 0;
    colorIndex++;
    FastLED.clear();
  }
}

void fxLarsonScanner() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  static int dir = 1;
  if (millis() - lastUpdate < 20) return;
  lastUpdate = millis();

  fadeToBlackBy(leds, NUM_LEDS, 60);
  leds[pos] = CRGB::Red;
  pos += dir;
  if (pos <= 0 || pos >= NUM_LEDS - 1) dir = -dir;
}

void fxFire2012() {
  static byte heat[NUM_LEDS];
  const byte cooling = 55;
  const byte sparking = 120;

  for (int i = 0; i < NUM_LEDS; i++) {
    heat[i] = qsub8(heat[i], random8(0, ((cooling * 10) / NUM_LEDS) + 2));
  }
  for (int k = NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
  if (random8() < sparking) {
    int y = random8(7);
    heat[y] = qadd8(heat[y], random8(160, 255));
  }
  for (int j = 0; j < NUM_LEDS; j++) {
    leds[j] = HeatColor(heat[j]);
  }
}

void fxMeteorRain() {
  static unsigned long lastUpdate = 0;
  static int meteorPos = 0;
  const byte meteorSize = 8;
  const byte meteorTrailDecay = 64;

  if (millis() - lastUpdate < 20) return;
  lastUpdate = millis();

  for (int i = 0; i < NUM_LEDS; i++) {
    if (random8(10) > 5) {
      leds[i].fadeToBlackBy(meteorTrailDecay);
    }
  }
  for (int j = 0; j < meteorSize; j++) {
    if (meteorPos - j >= 0 && meteorPos - j < NUM_LEDS) {
      leds[meteorPos - j] = CHSV(gHue, 255, 255);
    }
  }
  meteorPos++;
  if (meteorPos >= NUM_LEDS + meteorSize) {
    meteorPos = 0;
    gHue += 40;
  }
}

void fxTwinkleRandom() {
  fadeToBlackBy(leds, NUM_LEDS, 10);
  if (random8() < 80) {
    int pos = random16(NUM_LEDS);
    leds[pos] = CHSV(random8(), 200, 255);
  }
}

void fxBreathing() {
  uint8_t bright = beatsin8(15, 20, 255);
  fill_solid(leds, NUM_LEDS, CHSV(gHue, 255, bright));
  EVERY_N_MILLISECONDS(50) { gHue++; }
}

void fxPlasma() {
  static uint8_t t = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t colorIndex = sin8(i * 20 + t) / 2 + cos8(i * 10 - t) / 2;
    leds[i] = ColorFromPalette(RainbowColors_p, colorIndex);
  }
  t++;
}

void fxComet() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  if (millis() - lastUpdate < 25) return;
  lastUpdate = millis();

  fadeToBlackBy(leds, NUM_LEDS, 90);
  leds[pos] = CHSV(gHue, 255, 255);
  pos = (pos + 1) % NUM_LEDS;
  if (pos == 0) gHue += 20;
}

void fxStrobe() {
  static unsigned long lastUpdate = 0;
  static bool on = false;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();

  on = !on;
  fill_solid(leds, NUM_LEDS, on ? CRGB(CHSV(gHue, 255, 255)) : CRGB::Black);
  if (on) gHue += 15;
}

void fxRunningLights() {
  static uint16_t phase = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t bright = sin8(i * 20 + phase);
    leds[i] = CHSV(gHue, 255, bright);
  }
  phase += 4;
  gHue++;
}

typedef void (*EffectFunc)();
EffectFunc effects[] = {
  fxRainbowCycle, fxRainbowGlitter, fxConfetti, fxSinelon, fxBpm,
  fxJuggle, fxTheaterChase, fxColorWipe, fxLarsonScanner, fxFire2012,
  fxMeteorRain, fxTwinkleRandom, fxBreathing, fxPlasma, fxComet,
  fxStrobe, fxRunningLights
};
const uint8_t NUM_EFFECTS = sizeof(effects) / sizeof(effects[0]);

uint8_t currentEffect = 0;
unsigned long lastSwitch = 0;

#endif // USE_MICROPHONE

// ================================================================
//                          SETUP / LOOP
// ================================================================

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

#if USE_MICROPHONE
  setupI2S();
  for (int i = 0; i < NUM_BANDS; i++) bandPeaks[i] = 0;
  Serial.println("Режим: світломузика (мікрофон увімкнено)");
#else
  lastSwitch = millis();
  Serial.printf("Режим: демо-ефекти. Ефект 1/%d: fxRainbowCycle\n", NUM_EFFECTS);
#endif
}

void loop() {
#if USE_MICROPHONE
  readAudioAndFFT();
  renderSpectrum();
  FastLED.show();
#else
  unsigned long now = millis();
  if (now - lastSwitch >= EFFECT_DURATION) {
    lastSwitch = now;
    currentEffect = (currentEffect + 1) % NUM_EFFECTS;
    FastLED.clear();
    Serial.printf("Перемикаю на ефект %d/%d\n", currentEffect + 1, NUM_EFFECTS);
  }
  effects[currentEffect]();
  FastLED.show();
  delay(10);
#endif
}
