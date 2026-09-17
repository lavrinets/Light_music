/*
  ESP32-C3 + WS2812B — СВІТЛОМУЗИКА з WiFi, веб-керуванням та OTA

  Можливості:
    - 17 демо-ефектів, автоперемикання кожні 30 сек (можна вимкнути з вебсторінки)
    - Світломузика з мікрофона (I2S INMP441 + FFT) — вмикається/вимикається з вебсторінки
    - Веб-сторінка керування прямо з плати (відкрий IP плати в браузері)
    - WiFiManager: при першому вмиканні (або якщо WiFi не знайдено) плата підіймає
      власну точку доступу "LightMusic-Setup" — підключись до неї з телефону,
      відкриється сторінка вибору домашньої мережі й пароля. Дані зберігаються,
      наступного разу підключається сама.
    - ArduinoOTA — заливка прошивки по WiFi прямо з PlatformIO (для розробки)
    - HTTP OTA — раз на годину плата перевіряє version.json на твоєму Synology,
      і якщо там інша версія — сама завантажує і прошиває firmware.bin

  Бібліотеки (додай у platformio.ini lib_deps):
    - fastled/FastLED @ ^3.7.0
    - kosme/arduinoFFT @ ^2.0.0
    - tzapu/WiFiManager @ ^2.0.17
    - bblanchon/ArduinoJson @ ^7.0.0

  Піни (зміни під свою плату):
    LED_PIN    = 4   -> DIN стрічки (через резистор ~330 Ом)
    I2S_SCK    = 6   -> SCK мікрофона
    I2S_WS     = 7   -> WS (LRCL) мікрофона
    I2S_SD     = 5   -> SD (DOUT) мікрофона

  === НАЛАШТУВАННЯ НА SYNOLOGY ===
  Постав пакет Web Station (або просто розшар папку через File Station з веб-доступом).
  Створи папку /firmware з двома файлами:

    version.json:
      {
        "version": "1.2.0",
        "url": "https://твій-ddns.synology.me:ПОРТ/firmware/firmware.bin"
      }

    firmware.bin — сам скомпільований бінарник (лежить після Build у
      .pio/build/esp32-c3-devkitm-1/firmware.bin, скопіюй туди вручну після кожної збірки)

  Онови FIRMWARE_UPDATE_URL нижче під свій реальний DDNS-адрес і порт.
*/

// ======================= ВЕРСІЯ ПРОШИВКИ =======================
#define FIRMWARE_VERSION "2.0.1"
// Підніми цю цифру ПЕРЕД заливкою нової версії на Synology,
// інакше плата вирішить, що оновлення не потрібне.
// ===================================================================

// ======================= НАЛАШТУВАННЯ WIFI/OTA =======================
const char* OTA_HOSTNAME = "light-music";
const char* FIRMWARE_UPDATE_URL = "https://mystation.pp.ua:85/Light_music/firmware/version.json";
const unsigned long UPDATE_CHECK_INTERVAL = 3600000UL; // раз на годину (мс)

// ======================= ТЕМП ЕФЕКТІВ =======================
// GetSongBPM видалено — незручний пошук, слабке покриття українських виконавців.
// Темп тепер: або вручну повзунком "Ритм ефекту", або автоматично з мікрофона.
// ===============================================================

#include <FastLED.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <Preferences.h>

// ---------- Лог з дублюванням у пам'ять — видно віддалено на /log ----------
// Оголошено тут, на самому початку, бо це глобальні змінні/макрос,
// а не функції — .ino-конвертер PlatformIO не генерує для них прототипи,
// тож фізичний порядок у файлі має значення.
#define MAX_LOG_LINES 60
String logBuffer[MAX_LOG_LINES];
int logIndex = 0;
int logCount = 0;

void logLine(const String &s) {
  Serial.println(s);
  logBuffer[logIndex] = s;
  logIndex = (logIndex + 1) % MAX_LOG_LINES;
  if (logCount < MAX_LOG_LINES) logCount++;
}

void logLinef(const char* fmt, ...) {
  char buf[220];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  logLine(String(buf));
}
#include <ArduinoJson.h>
#include <driver/i2s_std.h>
#include <ArduinoFFT.h>

// ---------- НАЛАШТУВАННЯ СТРІЧКИ ----------
#define LED_PIN     4
#define NUM_LEDS    18           // <-- 54 фізичних LED / 3 на піксель (12V WS2811-стрічка)
#define LED_TYPE    WS2812B
#define COLOR_ORDER RGB

uint8_t currentBrightness = 15; // 0-255, тепер керується з вебсторінки

// ---------- ФІЗИЧНА КНОПКА СКИДАННЯ WIFI ----------
#define WIFI_RESET_BUTTON_PIN 9   // BOOT-кнопка на більшості ESP32-C3 плат
#define WIFI_RESET_HOLD_MS 5000   // утримувати 5 сек, щоб скинути WiFi
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;

CRGB leds[NUM_LEDS];
uint8_t gHue = 0;

// ---------- РЕЖИМ РОБОТИ (керується з вебсторінки) ----------
bool micEnabled = false;   // false = демо-ефекти, true = світломузика з мікрофона
bool autoCycle  = true;    // false = ефект зафіксований вручну через вебсторінку

WebServer server(80);

// ================================================================
//                  МІКРОФОН (I2S) + FFT — завжди в прошивці,
//                  але активний тільки коли micEnabled == true
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
float micSensitivity = 4000.0; // менше значення = чутливіше (реагує на тихіший звук)
float songBpm = 62; // спільна змінна темпу — оновлюється і пошуком пісні, і детектором ритму з мікрофона

i2s_chan_handle_t rxHandle;

void setupI2S() {
  i2s_chan_config_t chanConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  i2s_new_channel(&chanConfig, NULL, &rxHandle);

  i2s_std_config_t stdConfig = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLING_FREQ),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_SCK,
      .ws   = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_SD,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  stdConfig.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT; // зміни на I2S_STD_SLOT_RIGHT, якщо мікрофон на R/L підтягнутий до VDD

  i2s_channel_init_std_mode(rxHandle, &stdConfig);
  i2s_channel_enable(rxHandle);
}

void readAudioAndFFT() {
  size_t bytesRead = 0;
  static int32_t buffer32[SAMPLES];
  i2s_channel_read(rxHandle, buffer32, sizeof(buffer32), &bytesRead, portMAX_DELAY);
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
  float minFreq = 60.0, maxFreq = SAMPLING_FREQ / 2.0;
  float logMin = log(minFreq), logMax = log(maxFreq);

  for (int b = 0; b < NUM_BANDS; b++) {
    float f0 = exp(logMin + (logMax - logMin) * b / NUM_BANDS);
    float f1 = exp(logMin + (logMax - logMin) * (b + 1) / NUM_BANDS);
    int bin0 = max(1, (int)(f0 / freqPerBin));
    int bin1 = min(usableBins - 1, (int)(f1 / freqPerBin));
    float sum = 0; int count = 0;
    for (int i = bin0; i <= bin1; i++) { sum += vReal[i]; count++; }
    float avg = count > 0 ? sum / count : 0;
    bandValues[b] = constrain(avg / micSensitivity, 0.0, 1.0);
  }
}

void renderSpectrum() {
  int ledsPerBand = NUM_LEDS / NUM_BANDS;
  for (int b = 0; b < NUM_BANDS; b++) {
    if (bandValues[b] > bandPeaks[b]) bandPeaks[b] = bandValues[b];
    else { bandPeaks[b] -= 0.03; if (bandPeaks[b] < 0) bandPeaks[b] = 0; }

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

// ---------- Автоматичне визначення BPM з баса (onset detection) ----------
float bassAvgLevel = 0;
unsigned long lastBeatTime = 0;
float micDetectedBpm = 0;

void detectBeatAndUpdateBpm() {
  float bass = bandValues[0]; // найнижча частотна смуга — там сидить бас/бочка
  bassAvgLevel = bassAvgLevel * 0.95 + bass * 0.05; // повільне "фонове" середнє

  unsigned long now = millis();
  float threshold = bassAvgLevel * 1.4; // поріг над фоном, щоб вважати це "ударом"

  // мінімум 250мс між ударами (= максимум 240 BPM), щоб не ловити шум як подвійний удар
  if (bass > threshold && bass > 0.12 && (now - lastBeatTime) > 250) {
    if (lastBeatTime > 0) {
      unsigned long interval = now - lastBeatTime;
      float instantBpm = 60000.0 / interval;
      if (instantBpm >= 60 && instantBpm <= 200) { // відкидаємо явно нереалістичні значення
        micDetectedBpm = (micDetectedBpm == 0) ? instantBpm : (micDetectedBpm * 0.7 + instantBpm * 0.3);
        songBpm = micDetectedBpm; // одразу підтягуємо в загальний BPM, якщо десь використовується fxBpm
      }
    }
    lastBeatTime = now;
  }
}

// ================================================================
//                        17 ДЕМО-ЕФЕКТІВ
// ================================================================

const unsigned long EFFECT_DURATION = 30000; // 30 сек на ефект

void addGlitter(fract8 chanceOfGlitter) {
  if (random8() < chanceOfGlitter) leds[random16(NUM_LEDS)] += CRGB::White;
}

void fxRainbowCycle() { fill_rainbow(leds, NUM_LEDS, gHue, 7); gHue++; }
void fxRainbowGlitter() { fxRainbowCycle(); addGlitter(80); }

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
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8((uint8_t)constrain(songBpm, 10, 240), 64, 255);
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
  if (pos >= NUM_LEDS) { pos = 0; colorIndex++; FastLED.clear(); }
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
  const byte cooling = 55, sparking = 120;
  for (int i = 0; i < NUM_LEDS; i++) heat[i] = qsub8(heat[i], random8(0, ((cooling * 10) / NUM_LEDS) + 2));
  for (int k = NUM_LEDS - 1; k >= 2; k--) heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  if (random8() < sparking) { int y = random8(7); heat[y] = qadd8(heat[y], random8(160, 255)); }
  for (int j = 0; j < NUM_LEDS; j++) leds[j] = HeatColor(heat[j]);
}

void fxMeteorRain() {
  static unsigned long lastUpdate = 0;
  static int meteorPos = 0;
  const byte meteorSize = 8, meteorTrailDecay = 64;
  if (millis() - lastUpdate < 20) return;
  lastUpdate = millis();
  for (int i = 0; i < NUM_LEDS; i++) {
    if (random8(10) > 5) leds[i].fadeToBlackBy(meteorTrailDecay);
  }
  for (int j = 0; j < meteorSize; j++) {
    if (meteorPos - j >= 0 && meteorPos - j < NUM_LEDS) leds[meteorPos - j] = CHSV(gHue, 255, 255);
  }
  meteorPos++;
  if (meteorPos >= NUM_LEDS + meteorSize) { meteorPos = 0; gHue += 40; }
}

void fxTwinkleRandom() {
  fadeToBlackBy(leds, NUM_LEDS, 10);
  if (random8() < 80) leds[random16(NUM_LEDS)] = CHSV(random8(), 200, 255);
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

// ---------- 18. Фейерверк (ракети злітають і вибухають) ----------
void fxFireworks() {
  static unsigned long lastUpdate = 0;
  static int rocketPos = -1;
  static uint8_t rocketHue = 0;
  static bool exploding = false;
  static unsigned long explodeStart = 0;

  if (millis() - lastUpdate < 20) return;
  lastUpdate = millis();

  fadeToBlackBy(leds, NUM_LEDS, 40);

  if (!exploding) {
    if (rocketPos < 0) { rocketPos = NUM_LEDS - 1; rocketHue = random8(); }
    leds[rocketPos] = CHSV(rocketHue, 60, 255); // майже біла ракета, що летить вгору
    rocketPos -= 2;
    if (rocketPos <= NUM_LEDS / 3) { exploding = true; explodeStart = millis(); }
  } else {
    float t = (millis() - explodeStart) / 500.0;
    if (t > 1.0) { exploding = false; rocketPos = -1; return; }
    for (int i = 0; i < NUM_LEDS; i++) {
      if (random8() < 60 * (1.0 - t)) leds[i] += CHSV(rocketHue, 255, 255 * (1.0 - t));
    }
  }
}

// ---------- 19. Стрибучі м'ячики (фізична симуляція падіння) ----------
void fxBouncingBalls() {
  const int numBalls = 3;
  static float ballPos[numBalls];
  static float ballVel[numBalls];
  static bool initedBalls = false;
  static unsigned long lastUpdate = 0;

  if (!initedBalls) {
    for (int i = 0; i < numBalls; i++) { ballPos[i] = 0; ballVel[i] = 3 + i * 1.5; }
    initedBalls = true;
  }
  if (millis() - lastUpdate < 20) return;
  lastUpdate = millis();

  fadeToBlackBy(leds, NUM_LEDS, 100);
  for (int i = 0; i < numBalls; i++) {
    ballVel[i] -= 0.3; // "гравітація"
    ballPos[i] += ballVel[i];
    if (ballPos[i] < 0) { ballPos[i] = 0; ballVel[i] = -ballVel[i] * 0.85; } // відскок із втратою енергії
    int idx = constrain((int)(ballPos[i] / 15.0 * NUM_LEDS), 0, NUM_LEDS - 1);
    leds[idx] += CHSV(85 * i, 255, 255);
  }
}

// ---------- 20. Шумова плазма (Перлін-шум замість sin/cos) ----------
void fxNoisePlasma() {
  static uint16_t noiseZ = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t noiseVal = inoise8(i * 30, noiseZ);
    leds[i] = ColorFromPalette(RainbowColors_p, noiseVal);
  }
  noiseZ += 6;
}

// ---------- 21. Поліцейські вогні (червоно-сині половини) ----------
void fxPoliceLights() {
  static unsigned long lastUpdate = 0;
  static bool on = false;
  if (millis() - lastUpdate < 80) return;
  lastUpdate = millis();
  on = !on;
  int half = NUM_LEDS / 2;
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < half) leds[i] = on ? CRGB::Red : CRGB::Black;
    else leds[i] = on ? CRGB::Black : CRGB::Blue;
  }
}

// ---------- 22. Блок кольору, що біжить ----------
void fxGradientChase() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  if (millis() - lastUpdate < 30) return;
  lastUpdate = millis();
  fadeToBlackBy(leds, NUM_LEDS, 255);
  int blockSize = max(3, NUM_LEDS / 6);
  for (int i = 0; i < blockSize; i++) {
    int idx = (pos + i) % NUM_LEDS;
    uint8_t val = map(i, 0, blockSize, 255, 60);
    leds[idx] = CHSV(gHue, 255, val);
  }
  pos = (pos + 1) % NUM_LEDS;
  gHue++;
}

// ---------- 23. Густе мерехтіння з повільним згасанням ----------
void fxSparkleFade() {
  fadeToBlackBy(leds, NUM_LEDS, 5); // повільніше згасання, ніж twinkleRandom
  if (random8() < 150) leds[random16(NUM_LEDS)] = CHSV(random8(), 180, 255);
}

// ---------- 24. Райдужна крапка зі шлейфом ----------
void fxRainbowChase() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  if (millis() - lastUpdate < 30) return;
  lastUpdate = millis();
  fadeToBlackBy(leds, NUM_LEDS, 60);
  leds[pos] = CHSV(gHue, 255, 255);
  pos = (pos + 1) % NUM_LEDS;
  gHue += 4;
}

// ---------- 25. Квадратні імпульси, що біжать ----------
void fxSquarePulse() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  if (millis() - lastUpdate < 40) return;
  lastUpdate = millis();
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ((i + pos) % 6 < 3) ? CRGB(CHSV(gHue, 255, 255)) : CRGB::Black;
  }
  pos++;
  gHue++;
}

// ---------- 26. "Серцебиття" — подвійний пульс ----------
void fxHeartbeat() {
  static unsigned long phaseStart = 0;
  unsigned long t = millis() - phaseStart;
  const unsigned long cycle = 1000;
  if (t > cycle) phaseStart = millis();

  uint8_t bright;
  if (t < 150) bright = map(t, 0, 150, 0, 255);
  else if (t < 250) bright = map(t, 150, 250, 255, 60);
  else if (t < 350) bright = map(t, 250, 350, 60, 200);
  else if (t < 500) bright = map(t, 350, 500, 200, 0);
  else bright = 0;

  fill_solid(leds, NUM_LEDS, CHSV(0, 255, bright)); // червоний пульс
}

// ---------- 27. Хвиля-брижі з однієї точки ----------
void fxRipple() {
  static unsigned long lastUpdate = 0;
  static int center = -1;
  static float radius = 0;
  if (millis() - lastUpdate < 20) return;
  lastUpdate = millis();

  if (center < 0) { center = random16(NUM_LEDS); radius = 0; }
  fadeToBlackBy(leds, NUM_LEDS, 20);

  int idx1 = center - (int)radius;
  int idx2 = center + (int)radius;
  if (idx1 >= 0 && idx1 < NUM_LEDS) leds[idx1] = CHSV(gHue, 255, 255);
  if (idx2 >= 0 && idx2 < NUM_LEDS) leds[idx2] = CHSV(gHue, 255, 255);

  radius += 0.5;
  if (radius > NUM_LEDS) { center = -1; gHue += 30; }
}

// ---------- 28. Крижаний вогонь (Fire2012, синя палітра) ----------
void fxIceFire() {
  static byte heat[NUM_LEDS];
  const byte cooling = 55, sparking = 120;
  for (int i = 0; i < NUM_LEDS; i++) heat[i] = qsub8(heat[i], random8(0, ((cooling * 10) / NUM_LEDS) + 2));
  for (int k = NUM_LEDS - 1; k >= 2; k--) heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  if (random8() < sparking) { int y = random8(7); heat[y] = qadd8(heat[y], random8(160, 255)); }
  for (int j = 0; j < NUM_LEDS; j++) {
    uint8_t colorIndex = scale8(heat[j], 240);
    leds[j] = ColorFromPalette(CRGBPalette16(CRGB::Black, CRGB::Blue, CRGB::Aqua, CRGB::White), colorIndex);
  }
}

// ---------- 29. "Матричний дощ" (падаючі краплі) ----------
void fxMatrixRain() {
  static unsigned long lastUpdate = 0;
  static int drops[3] = {0, -8, -16};
  if (millis() - lastUpdate < 40) return;
  lastUpdate = millis();

  fadeToBlackBy(leds, NUM_LEDS, 80);
  for (int d = 0; d < 3; d++) {
    if (drops[d] >= 0 && drops[d] < NUM_LEDS) leds[drops[d]] = CRGB::Green;
    drops[d]++;
    if (drops[d] > NUM_LEDS + 10) drops[d] = -random8(10);
  }
}

// ---------- 30. Обертання суцільних кольорів ----------
void fxColorWheelRotate() {
  static unsigned long lastUpdate = 0;
  static uint8_t hue = 0;
  if (millis() - lastUpdate < 800) return; // тримає кожен колір довше, ніж просто перелив
  lastUpdate = millis();
  hue += 32; // дискретні кроки кольору, а не плавний перелив
  fill_solid(leds, NUM_LEDS, CHSV(hue, 255, 220));
}

// ---------- 31. Марш кольорових блоків ----------
void fxRandomMarch() {
  static unsigned long lastUpdate = 0;
  static uint8_t blockHue = 0;
  if (millis() - lastUpdate < 150) return;
  lastUpdate = millis();

  for (int i = NUM_LEDS - 1; i > 0; i--) leds[i] = leds[i - 1];
  leds[0] = (random8() < 60) ? CRGB(CHSV(random8(), 255, 255)) : CRGB::Black;
}

// ---------- 32. Аврора (повільний шум у зелено-фіолетових тонах) ----------
void fxAurora() {
  static uint16_t noiseZ = 0;
  CRGBPalette16 auroraPalette = CRGBPalette16(
    CRGB::Black, CRGB::DarkGreen, CRGB::Teal, CRGB::Purple
  );
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t noiseVal = inoise8(i * 20, noiseZ);
    leds[i] = ColorFromPalette(auroraPalette, noiseVal, 180);
  }
  noiseZ += 2; // повільніше за плазму — спокійний, "дихаючий" рух
}

typedef void (*EffectFunc)();
EffectFunc effects[] = {
  fxRainbowCycle, fxRainbowGlitter, fxConfetti, fxSinelon, fxBpm,
  fxJuggle, fxTheaterChase, fxColorWipe, fxLarsonScanner, fxFire2012,
  fxMeteorRain, fxTwinkleRandom, fxBreathing, fxPlasma, fxComet,
  fxStrobe, fxRunningLights,
  fxFireworks, fxBouncingBalls, fxNoisePlasma, fxPoliceLights, fxGradientChase,
  fxSparkleFade, fxRainbowChase, fxSquarePulse, fxHeartbeat, fxRipple,
  fxIceFire, fxMatrixRain, fxColorWheelRotate, fxRandomMarch, fxAurora
};

const char* effectNames[] = {
  "Райдужний перелив", "Райдуга з блискітками", "Конфеті", "Синелон", "Пульс (BPM)",
  "Жонглювання", "Театральна доріжка", "Color wipe", "Larson scanner", "Вогонь (Fire2012)",
  "Метеоритний дощ", "Мерехтіння зірок", "Дихання", "Плазма", "Комета",
  "Стробоскоп", "Хвиля (running lights)",
  "Фейерверк", "Стрибучі м'ячики", "Шумова плазма", "Поліцейські вогні", "Блок кольору",
  "Густе мерехтіння", "Райдужна крапка", "Квадратні імпульси", "Серцебиття", "Брижі",
  "Крижаний вогонь", "Матричний дощ", "Обертання кольорів", "Марш блоків", "Аврора"
};

const uint8_t NUM_EFFECTS = sizeof(effects) / sizeof(effects[0]);
uint8_t currentEffect = 0;
unsigned long lastSwitch = 0;

bool forceUpdateCheck = false;
String lastUpdateCheckResult = "ще не перевірялось";
Preferences prefs;

unsigned long lastWifiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000; // спроба раз на 30 сек, поки WiFi відсутній
bool wasWifiConnected = false;

// ================================================================
//                          WIFI / OTA / WEB
// ================================================================

// ======================= СТАТИЧНИЙ WIFI (пріоритетний) =======================
// Якщо задано — плата спершу пробує підключитись сюди напряму (швидко, без порталу).
// Якщо не вдасться за WIFI_STATIC_TIMEOUT_MS — впаде на WiFiManager (портал LightMusic-Setup).
// Залиш порожніми ("") обидва рядки, якщо статичний WiFi не потрібен.
const char* WIFI_STATIC_SSID = "";
const char* WIFI_STATIC_PASSWORD = "";
const unsigned long WIFI_STATIC_TIMEOUT_MS = 10000;
// ================================================================================

void setupWiFi() {
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);

  // Спершу пробуємо статичний WiFi, якщо він заданий (не порожній)
  if (strlen(WIFI_STATIC_SSID) > 0) {
    logLinef("[WiFi] Пробую статичне підключення до \"%s\"...\n", WIFI_STATIC_SSID);
    WiFi.begin(WIFI_STATIC_SSID, WIFI_STATIC_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_STATIC_TIMEOUT_MS) {
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[WiFi] Статичне підключення успішне, IP: ");
      logLine(WiFi.localIP().toString());
      return;
    }
    logLine("[WiFi] Статичне підключення не вдалось, переходжу на WiFiManager...");
  }

  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // 3 хв на налаштування, потім працює далі офлайн демо-режимом
  bool connected = wm.autoConnect("LightMusic-Setup");
  if (connected) {
    Serial.print("WiFi підключено, IP: ");
    logLine(WiFi.localIP().toString());
  } else {
    logLine("WiFi не підключено — працюю офлайн (вебсторінка й OTA недоступні)");
  }
}

void setupOTA() {
  if (WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();
  logLine("ArduinoOTA готовий (заливка прошивки по WiFi з PlatformIO)");
}

void setupMDNS() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (MDNS.begin(OTA_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    logLinef("mDNS готовий — відкривай http://%s.local\n", OTA_HOSTNAME);
  } else {
    logLine("Не вдалось запустити mDNS");
  }
}

const char PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="uk">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Light_music</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; margin:0; padding:16px; }
  h1 { font-size:20px; }
  #status { margin-bottom:16px; padding:10px; background:#222; border-radius:8px; }
  .grid { display:grid; grid-template-columns: repeat(auto-fill, minmax(140px,1fr)); gap:8px; }
  button { padding:10px; border:none; border-radius:8px; background:#333; color:#eee; cursor:pointer; }
  button.active { background:#3a7; color:#000; }
  .row { margin:10px 0; display:flex; align-items:center; gap:10px; }
  input[type=range] { flex:1; max-width:300px; }
</style>
</head>
<body>
<h1>Light_music</h1>
<p><a href="/log" style="color:#6cf; font-size:12px;">📜 Переглянути лог</a></p>
<div id="status">Завантаження...</div>
<div id="updateStatus" style="margin-bottom:16px; font-size:13px; color:#999;"></div>


<div class="row">
  <label><input type="checkbox" id="micToggle"> Мікрофон (світломузика)</label>
</div>
<div class="row" id="micSensitivityRow" style="display:none;">
  <span style="min-width:90px;">Чутливість мік.</span>
  <span>🔈</span>
  <input type="range" id="micSensSlider" min="500" max="10000" value="4000">
  <span>🔊</span>
  <span id="micSensPercent" style="min-width:40px;"></span>
</div>
<div id="micBpmInfo" style="display:none; font-size:13px; color:#999; margin-bottom:10px;"></div>
<div class="row">
  <span style="min-width:90px;">Яскравість</span>
  <span>🔅</span>
  <input type="range" id="brightnessSlider" min="0" max="255" value="120">
  <span>🔆</span>
  <span id="brightnessPercent" style="min-width:40px;">47%</span>
</div>
<div class="row">
  <span style="min-width:90px;">Ритм ефекту</span>
  <span>🐢</span>
  <input type="range" id="bpmSlider" min="40" max="220" value="120">
  <span>🐇</span>
  <span id="bpmPercent" style="min-width:40px;">44%</span>
</div>
<div class="row">
  <button onclick="setAuto()">Авто-перемикання ефектів</button>
  <button onclick="resetWifi()" style="background:#733;">Змінити WiFi</button>
  <button onclick="reboot()" style="background:#753;">Перезавантажити плату</button>
  <button onclick="checkUpdate()">Перевірити оновлення</button>
</div>

<div class="grid" id="effectGrid"></div>

<script>
const EFFECT_NAMES = REPLACE_NAMES;
const loadStatus = async () => {
  const r = await fetch('/status');
  const s = await r.json();
  document.getElementById('status').innerText =
    `Ефект: ${s.effect} | Мікрофон: ${s.mic ? 'увімкнено' : 'вимкнено'} | Авто: ${s.auto ? 'так' : 'ні'} | v${s.version}`;
  document.getElementById('updateStatus').innerText = 'Оновлення: ' + s.updateStatus;
  document.getElementById('micToggle').checked = s.mic;
  document.getElementById('micSensitivityRow').style.display = s.mic ? 'flex' : 'none';
  document.getElementById('micBpmInfo').style.display = s.mic ? 'block' : 'none';
  if (s.mic) {
    document.getElementById('micBpmInfo').innerText =
      s.micDetectedBpm > 0 ? `🥁 Визначений ритм: ${s.micDetectedBpm.toFixed(0)} BPM` : '🥁 Слухаю ритм...';
  }
  if (!micSensDragging) {
    document.getElementById('micSensSlider').value = s.micSensitivity;
    updateSliderPercent('micSensSlider', 'micSensPercent');
  }
  if (!brightnessDragging) {
    document.getElementById('brightnessSlider').value = s.brightness;
    updateSliderPercent('brightnessSlider', 'brightnessPercent');
  }
  if (!bpmDragging) {
    document.getElementById('bpmSlider').value = s.songBpm;
    updateSliderPercent('bpmSlider', 'bpmPercent');
  }
  document.querySelectorAll('.grid button').forEach((b,i)=>{
    b.classList.toggle('active', !s.mic && i === s.index);
  });
};
const buildGrid = () => {
  const grid = document.getElementById('effectGrid');
  EFFECT_NAMES.forEach((name, i) => {
    const btn = document.createElement('button');
    btn.innerText = name;
    btn.onclick = () => fetch('/effect?i=' + i).then(loadStatus);
    grid.appendChild(btn);
  });
};
const setAuto = () => { fetch('/auto').then(loadStatus); };
const resetWifi = () => {
  if (confirm('Скинути WiFi-налаштування? Плата перезавантажиться і підніме точку доступу LightMusic-Setup.')) {
    fetch('/resetwifi');
    document.getElementById('status').innerText = 'Скидаю WiFi, плата перезавантажується...';
  }
};
const reboot = () => {
  if (confirm('Перезавантажити плату?')) {
    fetch('/reboot');
    document.getElementById('status').innerText = 'Перезавантажуюсь...';
  }
};

const checkUpdate = () => {
  fetch('/checkupdate');
  document.getElementById('updateStatus').innerText = 'Оновлення: перевіряю...';
  let attempts = 0;
  const poll = () => {
    attempts++;
    fetch('/status')
      .then(r => r.json())
      .then(s => {
        document.getElementById('updateStatus').innerText = 'Оновлення: ' + s.updateStatus;
        // якщо все ще "оновлююсь" — плата, можливо, перезавантажується, питаємо ще раз
        if (s.updateStatus.includes('оновлююсь') && attempts < 30) {
          setTimeout(poll, 2000);
        } else {
          loadStatus();
        }
      })
      .catch(() => {
        // плата тимчасово недоступна (ймовірно перезавантажується) — пробуємо ще раз
        if (attempts < 30) {
          document.getElementById('updateStatus').innerText = 'Оновлення: плата перезавантажується...';
          setTimeout(poll, 2000);
        }
      });
  };
  setTimeout(poll, 2000);
};
document.getElementById('micToggle').addEventListener('change', (e) => {
  fetch('/mic?on=' + (e.target.checked ? '1' : '0')).then(loadStatus);
});

const updateSliderPercent = (sliderId, percentId) => {
  const el = document.getElementById(sliderId);
  const percent = Math.round((el.value - el.min) / (el.max - el.min) * 100);
  document.getElementById(percentId).innerText = percent + '%';
};

let brightnessDragging = false;
let brightnessDebounce = null;
const brightnessSlider = document.getElementById('brightnessSlider');
brightnessSlider.addEventListener('input', (e) => {
  brightnessDragging = true;
  updateSliderPercent('brightnessSlider', 'brightnessPercent');
  clearTimeout(brightnessDebounce);
  brightnessDebounce = setTimeout(() => {
    fetch('/brightness?v=' + e.target.value);
  }, 150);
});
brightnessSlider.addEventListener('change', () => {
  brightnessDragging = false;
});

let bpmDragging = false;
let bpmDebounce = null;
const bpmSlider = document.getElementById('bpmSlider');
bpmSlider.addEventListener('input', (e) => {
  bpmDragging = true;
  updateSliderPercent('bpmSlider', 'bpmPercent');
  clearTimeout(bpmDebounce);
  bpmDebounce = setTimeout(() => {
    fetch('/applysong?bpm=' + e.target.value);
  }, 150);
});
bpmSlider.addEventListener('change', () => {
  bpmDragging = false;
});

let micSensDragging = false;
let micSensDebounce = null;
const micSensSlider = document.getElementById('micSensSlider');
micSensSlider.addEventListener('input', (e) => {
  micSensDragging = true;
  updateSliderPercent('micSensSlider', 'micSensPercent');
  clearTimeout(micSensDebounce);
  micSensDebounce = setTimeout(() => {
    fetch('/micsensitivity?v=' + e.target.value);
  }, 150);
});
micSensSlider.addEventListener('change', () => {
  micSensDragging = false;
});

buildGrid();
loadStatus();
setInterval(loadStatus, 2000);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  // Підставляємо список назв ефектів у JS-масив прямо в HTML
  String page = FPSTR(PAGE_HTML);
  String namesJs = "[";
  for (int i = 0; i < NUM_EFFECTS; i++) {
    namesJs += "\"" + String(effectNames[i]) + "\"";
    if (i < NUM_EFFECTS - 1) namesJs += ",";
  }
  namesJs += "]";
  page.replace("REPLACE_NAMES", namesJs);
  server.send(200, "text/html", page);
}

void handleStatus() {
  JsonDocument doc;
  doc["effect"] = micEnabled ? "Світломузика (мікрофон)" : effectNames[currentEffect];
  doc["index"] = currentEffect;
  doc["mic"] = micEnabled;
  doc["auto"] = autoCycle;
  doc["brightness"] = currentBrightness;
  doc["songBpm"] = songBpm;
  doc["micSensitivity"] = micSensitivity;
  doc["micDetectedBpm"] = micDetectedBpm;
  doc["version"] = FIRMWARE_VERSION;
  doc["updateStatus"] = lastUpdateCheckResult;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleSetEffect() {
  if (server.hasArg("i")) {
    int i = server.arg("i").toInt();
    if (i >= 0 && i < NUM_EFFECTS) {
      currentEffect = i;
      autoCycle = false;
      micEnabled = false;
      FastLED.clear();
      logLinef("[web] Обрано ефект вручну: %s\n", effectNames[i]);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleSetAuto() {
  autoCycle = true;
  micEnabled = false;
  lastSwitch = millis();
  logLine("[web] Увімкнено авто-перемикання ефектів");
  server.send(200, "text/plain", "OK");
}

void handleMic() {
  if (server.hasArg("on")) {
    micEnabled = server.arg("on") == "1";
    if (micEnabled) autoCycle = false;
    logLinef("[web] Мікрофон: %s\n", micEnabled ? "увімкнено" : "вимкнено");
  }
  server.send(200, "text/plain", "OK");
}

void handleLogPage() {
  String page = R"HTML(
<!DOCTYPE html>
<html lang="uk">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Light_music — лог</title>
<style>
  body { font-family: monospace; background:#111; color:#0f0; margin:0; padding:16px; font-size:13px; }
  h1 { font-family: sans-serif; color:#eee; font-size:18px; }
  #log { white-space: pre-wrap; word-break: break-all; }
  a { color:#6cf; }
</style>
</head>
<body>
<h1>Light_music — Serial-лог (останні )HTML" + String(MAX_LOG_LINES) + R"HTML( рядків)</h1>
<p><a href="/">← Назад на керування</a></p>
<div id="log">Завантаження...</div>
<script>
const loadLog = () => {
  fetch('/logdata').then(r => r.text()).then(t => {
    document.getElementById('log').innerText = t;
    window.scrollTo(0, document.body.scrollHeight);
  });
};
loadLog();
setInterval(loadLog, 2000);
</script>
</body>
</html>
)HTML";
  server.send(200, "text/html", page);
}

void handleLogData() {
  String out;
  for (int i = 0; i < logCount; i++) {
    int idx = (logIndex + i) % MAX_LOG_LINES; // від найстарішого до найновішого
    if (logCount < MAX_LOG_LINES) idx = i; // поки буфер не заповнився — просто по порядку
    out += logBuffer[idx] + "\n";
  }
  server.send(200, "text/plain", out);
}

void handleReboot() {
  server.send(200, "text/plain", "OK, перезавантажуюсь...");
  delay(200);
  ESP.restart();
}

void handleResetWifi() {
  server.send(200, "text/plain", "OK, перезавантажуюсь...");
  delay(200); // встигнути відправити відповідь перед перезавантаженням
  WiFiManager wm;
  wm.resetSettings();
  ESP.restart();
}

void handleMicSensitivity() {
  if (server.hasArg("v")) {
    float v = server.arg("v").toFloat();
    if (v >= 500 && v <= 10000) {
      micSensitivity = v;
      logLinef("[web] Чутливість мікрофона: %.0f\n", micSensitivity);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleBrightness() {
  if (server.hasArg("v")) {
    int v = server.arg("v").toInt();
    if (v >= 0 && v <= 255) {
      currentBrightness = v;
      FastLED.setBrightness(currentBrightness);
      logLinef("[web] Яскравість: %d\n", currentBrightness);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleApplySong() {
  if (!server.hasArg("bpm")) {
    server.send(400, "text/plain", "no bpm");
    return;
  }
  songBpm = server.arg("bpm").toFloat();
  currentEffect = 4; // fxBpm — 5-й у списку effects[]
  autoCycle = false;
  micEnabled = false;
  FastLED.clear();
  logLinef("[web] Застосовано темп: %.1f BPM\n", songBpm);
  server.send(200, "text/plain", "OK");
}

void handleCheckUpdate() {
  forceUpdateCheck = true;
  server.send(200, "text/plain", "OK, перевіряю...");
}

void setupWebServer() {
  if (WiFi.status() != WL_CONNECTED) return;
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/effect", handleSetEffect);
  server.on("/auto", handleSetAuto);
  server.on("/mic", handleMic);
  server.on("/brightness", handleBrightness);
  server.on("/micsensitivity", handleMicSensitivity);
  server.on("/checkupdate", handleCheckUpdate);
  server.on("/applysong", handleApplySong);
  server.on("/resetwifi", handleResetWifi);
  server.on("/reboot", handleReboot);
  server.on("/log", handleLogPage);
  server.on("/logdata", handleLogData);
  server.begin();
  logLine("Веб-сервер запущений — відкрий IP плати в браузері");
}

// ---------- HTTP OTA: перевірка нової версії на Synology ----------
void checkFirmwareUpdate() {
  static unsigned long lastCheck = 0;
  if (WiFi.status() != WL_CONNECTED) return;

  bool forced = forceUpdateCheck;
  if (!forced && lastCheck != 0 && millis() - lastCheck < UPDATE_CHECK_INTERVAL) return;
  lastCheck = millis();
  forceUpdateCheck = false;

  WiFiClientSecure client;
  client.setInsecure(); // ОК для Let's Encrypt теж; прибери й додай сертифікат, якщо хочеш строгу перевірку

  HTTPClient http;
  if (!http.begin(client, FIRMWARE_UPDATE_URL)) {
    logLine("[OTA] Не вдалось відкрити з'єднання для перевірки версії");
    lastUpdateCheckResult = "помилка з'єднання";
    return;
  }

  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    JsonDocument doc;
    if (deserializeJson(doc, payload) == DeserializationError::Ok) {
      String newVersion = doc["version"].as<String>();
      String binUrl = doc["url"].as<String>();
      if (newVersion.length() && newVersion != FIRMWARE_VERSION) {
        logLinef("[OTA] Знайдено нову версію %s (поточна %s), оновлююсь...\n",
                      newVersion.c_str(), FIRMWARE_VERSION);
        lastUpdateCheckResult = "знайдено v" + newVersion + ", оновлююсь...";
        httpUpdate.onProgress([](int cur, int total) {
          static int lastPercent = -1;
          int percent = total > 0 ? (cur * 100 / total) : 0;
          if (percent != lastPercent && percent % 10 == 0) {
            logLinef("[OTA] Завантаження: %d%%\n", percent);
            lastPercent = percent;
          }
        });
        t_httpUpdate_return ret = httpUpdate.update(client, binUrl);
        if (ret == HTTP_UPDATE_FAILED) {
          logLinef("[OTA] Помилка оновлення: %s\n", httpUpdate.getLastErrorString().c_str());
          lastUpdateCheckResult = "помилка оновлення: " + String(httpUpdate.getLastErrorString().c_str());
        }
        // при успіху плата сама перезавантажиться
      } else {
        logLine("[OTA] Версія актуальна");
        lastUpdateCheckResult = "версія актуальна (v" + String(FIRMWARE_VERSION) + ")";
      }
    } else {
      lastUpdateCheckResult = "помилка розбору version.json";
    }
  } else {
    logLinef("[OTA] Не вдалось перевірити версію, код: %d\n", code);
    lastUpdateCheckResult = "помилка перевірки, код " + String(code);
  }
  http.end();
}

// ================================================================
//                          SETUP / LOOP
// ================================================================

void setup() {
  Serial.begin(115200);
  logLinef("=== Light_music firmware v%s ===\n", FIRMWARE_VERSION);

  prefs.begin("lightmusic", false);
  String lastVersion = prefs.getString("version", "");
  if (lastVersion.length() && lastVersion != FIRMWARE_VERSION) {
    lastUpdateCheckResult = "успішно оновлено з v" + lastVersion + " до v" + String(FIRMWARE_VERSION) + "!";
    logLine("[OTA] " + lastUpdateCheckResult);
  }
  prefs.putString("version", FIRMWARE_VERSION);

  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(currentBrightness);
  FastLED.clear();
  FastLED.show();

  setupWiFi();
  setupOTA();
  setupMDNS();
  setupWebServer();
  setupI2S();
  for (int i = 0; i < NUM_BANDS; i++) bandPeaks[i] = 0;
  wasWifiConnected = (WiFi.status() == WL_CONNECTED);

  lastSwitch = millis();
  logLinef("Демо-режим. Ефект 1/%d: %s\n", NUM_EFFECTS, effectNames[0]);
}

void loop() {
  // ---- Фізична кнопка: утримання 5 сек скидає WiFi ----
  bool buttonPressed = (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW);
  if (buttonPressed && !buttonWasPressed) {
    buttonPressStart = millis();
  }
  if (buttonPressed && buttonWasPressed) {
    if (millis() - buttonPressStart >= WIFI_RESET_HOLD_MS) {
      logLine("[кнопка] Утримання 5 сек — скидаю WiFi і перезавантажуюсь");
      WiFiManager wm;
      wm.resetSettings();
      delay(100);
      ESP.restart();
    }
  }
  buttonWasPressed = buttonPressed;

  if (WiFi.status() == WL_CONNECTED) {
    if (!wasWifiConnected) {
      // Щойно відновилось з'єднання (не просто перший запуск) — перезапускаємо
      // mDNS/OTA, бо вони інколи "не оживають" самі після реального обриву
      Serial.print("[WiFi] Підключення відновлено, IP: ");
      logLine(WiFi.localIP().toString());
      setupMDNS();
      setupOTA();
      wasWifiConnected = true;
    }
    server.handleClient();
    ArduinoOTA.handle();
    checkFirmwareUpdate();
  } else {
    wasWifiConnected = false;
    // WiFi відпав — пробуємо перепідключитись раз на WIFI_RECONNECT_INTERVAL,
    // не блокуючи основний цикл (ефекти й далі йдуть, поки чекаємо мережу)
    if (millis() - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
      lastWifiReconnectAttempt = millis();
      logLine("[WiFi] З'єднання втрачено, пробую перепідключитись...");
      WiFi.reconnect();
    }
  }

  if (micEnabled) {
    readAudioAndFFT();
    renderSpectrum();
    detectBeatAndUpdateBpm();
  } else {
    unsigned long now = millis();
    if (autoCycle && now - lastSwitch >= EFFECT_DURATION) {
      lastSwitch = now;
      currentEffect = (currentEffect + 1) % NUM_EFFECTS;
      FastLED.clear();
      logLinef("Перемикаю на ефект %d/%d: %s\n", currentEffect + 1, NUM_EFFECTS, effectNames[currentEffect]);
    }
    EVERY_N_MILLISECONDS(5000) {
      if (autoCycle) {
        unsigned long elapsed = millis() - lastSwitch;
        unsigned long remainingSec = (EFFECT_DURATION > elapsed) ? (EFFECT_DURATION - elapsed) / 1000 : 0;
        logLinef("До зміни ефекту (%s): %lu сек\n", effectNames[currentEffect], remainingSec);
      }
    }
    effects[currentEffect]();
  }

  FastLED.show();
  delay(10);
}
