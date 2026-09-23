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
#define FIRMWARE_VERSION "3.3.0"
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
#include <ArduinoJson.h>
#include <driver/i2s_std.h>
#include <ArduinoFFT.h>

// ---------- НАЛАШТУВАННЯ СТРІЧКИ ----------
#define LED_PIN     4
#define NUM_LEDS    18           // <-- 54 фізичних LED / 3 на піксель (12V WS2811-стрічка)
#define LED_TYPE    WS2812B
#define COLOR_ORDER BRG   // підібрано емпірично під конкретну стрічку — не міняти

uint8_t currentBrightness = 5; // 0-255, тепер керується з вебсторінки

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
bool staticLightEnabled = false; // окремий режим, найвищий пріоритет над ефектами й мікрофоном
CRGB staticColor = CRGB::White;

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
#define NUM_BANDS       6   // 18 LED / 6 смуг = рівно 3 пікселі на смугу, без залишку (раніше 8 лишало 2 діоди завжди чорними)

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

unsigned long effectDuration = 30000; // 30 сек на ефект за замовчуванням; тепер керується з вебу

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
  if (millis() - lastUpdate < 50) return;
  lastUpdate = millis();
  leds[pos] = colors[colorIndex % 6];
  pos++;
  if (pos >= NUM_LEDS) { pos = 0; colorIndex++; FastLED.clear(); }
}

void fxLarsonScanner() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  static int dir = 1;
  if (millis() - lastUpdate < 45) return;
  lastUpdate = millis();
  fadeToBlackBy(leds, NUM_LEDS, 60);
  leds[pos] = CRGB::Red;
  pos += dir;
  if (pos <= 0 || pos >= NUM_LEDS - 1) dir = -dir;
}

void fxFire2012() {
  static byte heat[NUM_LEDS];
  const byte cooling = 55, sparking = 90;
  for (int i = 0; i < NUM_LEDS; i++) heat[i] = qsub8(heat[i], random8(0, 18)); // баланс під коротку стрічку: попередня спроба (0,4) давала перегрів до білого
  for (int k = NUM_LEDS - 1; k >= 2; k--) heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  if (random8() < sparking) { int y = random8(7); heat[y] = qadd8(heat[y], random8(160, 255)); }
  for (int j = 0; j < NUM_LEDS; j++) leds[j] = HeatColor(heat[j]);
}

void fxMeteorRain() {
  static unsigned long lastUpdate = 0;
  static int meteorPos = 0;
  const byte meteorSize = 3, meteorTrailDecay = 96; // менший метеор і швидший шлейф — для 18-пиксельної стрічки
  if (millis() - lastUpdate < 40) return;
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
  if (millis() - lastUpdate < 55) return;
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
    for (int i = 0; i < numBalls; i++) {
      ballPos[i] = NUM_LEDS - 1;       // Починаємо з вершини стрічки
      ballVel[i] = 0;                  // Просто відпускаємо падати
    }
    initedBalls = true;
  }

  if (millis() - lastUpdate < 20) return;
  lastUpdate = millis();

  fadeToBlackBy(leds, NUM_LEDS, 100);

  // Фізичні коефіцієнти (підібрані під розмір стрічки)
  const float gravity = -0.1;          // М'яка гравітація
  const float bounceImpact = -0.90;    // Пружність відскоку (повертає 90% енергії)

  for (int i = 0; i < numBalls; i++) {
    ballVel[i] += gravity;             // Додаємо гравітацію до швидкості
    ballPos[i] += ballVel[i];          // Змінюємо позицію

    // Перевірка удару об землю
    if (ballPos[i] <= 0) {
      ballPos[i] = 0;                  // Залишаємо на землі
      ballVel[i] = ballVel[i] * bounceImpact; // Відскок вгору

      // Анти-затухання: якщо енергії замало для підйому, штовхаємо надійно назад
      if (ballVel[i] < 0.5) {
        ballVel[i] = 2.0 + i * 0.5;
      }
    }

    // Переводимо позицію напряму в індекс світлодіода
    int idx = constrain((int)ballPos[i], 0, NUM_LEDS - 1);

    // Малюємо м'ячик (явне присвоєння кольору, щоб не змішувались у білий)
    leds[idx] = CHSV(85 * i, 255, 255);
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
  if (millis() - lastUpdate < 55) return;
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
  if (millis() - lastUpdate < 55) return;
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
  if (millis() - lastUpdate < 60) return;
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
  if (millis() - lastUpdate < 45) return;
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
  const byte cooling = 55, sparking = 90;
  for (int i = 0; i < NUM_LEDS; i++) heat[i] = qsub8(heat[i], random8(0, 18)); // баланс під коротку стрічку: попередня спроба (0,4) давала перегрів до білого
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

// ---------- 33. Подвійне сканування (дві крапки назустріч) ----------
void fxDualScan() {
  static unsigned long lastUpdate = 0;
  static int pos1 = 0, pos2 = 0;
  static int dir1 = 1, dir2 = -1;
  if (millis() - lastUpdate < 45) return;
  lastUpdate = millis();
  if (pos2 == 0) pos2 = NUM_LEDS - 1; // ініціалізація стартової позиції другої крапки

  fadeToBlackBy(leds, NUM_LEDS, 60);
  leds[pos1] = CRGB::Cyan;
  leds[pos2] = CRGB::Magenta;
  pos1 += dir1;
  pos2 += dir2;
  if (pos1 <= 0 || pos1 >= NUM_LEDS - 1) dir1 = -dir1;
  if (pos2 <= 0 || pos2 >= NUM_LEDS - 1) dir2 = -dir2;
}

// ---------- 34. Сніжне мерехтіння (біле на чорному) ----------
void fxSnowSparkle() {
  fadeToBlackBy(leds, NUM_LEDS, 8);
  if (random8() < 60) leds[random16(NUM_LEDS)] = CRGB::White;
}

// ---------- 35. Гелловінська доріжка (помаранчево-фіолетова) ----------
void fxHalloweenChase() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  if (millis() - lastUpdate < 100) return;
  lastUpdate = millis();
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ((i + pos) % 4 < 2) ? CRGB::OrangeRed : CRGB(80, 0, 130); // помаранчевий / фіолетовий
  }
  pos++;
}

// ---------- 36. Заповнення з двох країв назустріч ----------
void fxColorSweep() {
  static unsigned long lastUpdate = 0;
  static int step = 0;
  if (millis() - lastUpdate < 60) return;
  lastUpdate = millis();
  int half = NUM_LEDS / 2;
  if (step > half) { step = 0; FastLED.clear(); gHue += 40; }
  if (step < NUM_LEDS - step) {
    leds[step] = CHSV(gHue, 255, 255);
    leds[NUM_LEDS - 1 - step] = CHSV(gHue, 255, 255);
  }
  step++;
}

// ---------- 37. Блимання суцільним кольором, що змінюється ----------
void fxBlinkRainbow() {
  static unsigned long lastUpdate = 0;
  static bool on = false;
  if (millis() - lastUpdate < 400) return;
  lastUpdate = millis();
  on = !on;
  if (on) { fill_solid(leds, NUM_LEDS, CHSV(gHue, 255, 255)); gHue += 32; }
  else fill_solid(leds, NUM_LEDS, CRGB::Black);
}

// ---------- 38. Мерехтіння свічки (теплий колір, що тремтить) ----------
void fxFireFlicker() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 50) return;
  lastUpdate = millis();
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t flicker = random8(180, 255);
    leds[i] = CRGB(flicker, flicker / 3, 0); // теплий помаранчевий, що тремтить яскравістю
  }
}

// ---------- 39. Обертання широких кольорових смуг ----------
void fxRotatingBands() {
  static unsigned long lastUpdate = 0;
  static int offset = 0;
  if (millis() - lastUpdate < 60) return;
  lastUpdate = millis();
  int bandSize = max(2, NUM_LEDS / 4);
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t band = ((i + offset) / bandSize) % 4;
    leds[i] = CHSV(band * 64, 255, 255);
  }
  offset++;
}

// ---------- 40. Чорний блок, що біжить по кольоровому фону ----------
void fxChaseBlackout() {
  static unsigned long lastUpdate = 0;
  static int pos = 0;
  if (millis() - lastUpdate < 60) return;
  lastUpdate = millis();
  fill_solid(leds, NUM_LEDS, CHSV(gHue, 255, 200));
  int blockSize = max(2, NUM_LEDS / 6);
  for (int i = 0; i < blockSize; i++) {
    int idx = (pos + i) % NUM_LEDS;
    leds[idx] = CRGB::Black;
  }
  pos = (pos + 1) % NUM_LEDS;
  if (pos == 0) gHue += 30;
}

typedef void (*EffectFunc)();
EffectFunc effects[] = {
  fxRainbowCycle, fxRainbowGlitter, fxConfetti, fxSinelon, fxBpm,
  fxJuggle, fxTheaterChase, fxColorWipe, fxLarsonScanner, fxFire2012,
  fxMeteorRain, fxTwinkleRandom, fxBreathing, fxPlasma, fxComet,
  fxStrobe, fxRunningLights,
  fxFireworks, fxBouncingBalls, fxNoisePlasma, fxPoliceLights, fxGradientChase,
  fxSparkleFade, fxRainbowChase, fxSquarePulse, fxHeartbeat, fxRipple,
  fxIceFire, fxMatrixRain, fxColorWheelRotate, fxRandomMarch, fxAurora,
  fxDualScan, fxSnowSparkle, fxHalloweenChase, fxColorSweep, fxBlinkRainbow,
  fxFireFlicker, fxRotatingBands, fxChaseBlackout
};

const char* effectNames[] = {
  "Райдужний перелив", "Райдуга з блискітками", "Конфеті", "Синелон", "Пульс (BPM)",
  "Жонглювання", "Театральна доріжка", "Color wipe", "Larson scanner", "Вогонь (Fire2012)",
  "Метеоритний дощ", "Мерехтіння зірок", "Дихання", "Плазма", "Комета",
  "Стробоскоп", "Хвиля (running lights)",
  "Фейерверк", "Стрибучі м'ячики", "Шумова плазма", "Поліцейські вогні", "Блок кольору",
  "Густе мерехтіння", "Райдужна крапка", "Квадратні імпульси", "Серцебиття", "Брижі",
  "Крижаний вогонь", "Матричний дощ", "Обертання кольорів", "Марш блоків", "Аврора",
  "Подвійне сканування", "Сніжне мерехтіння", "Гелловін", "Заповнення з країв", "Блимання кольором",
  "Мерехтіння свічки", "Обертання смуг", "Чорний блок"
};

const uint8_t NUM_EFFECTS = sizeof(effects) / sizeof(effects[0]);
uint8_t currentEffect = 0;
unsigned long lastSwitch = 0;

bool forceUpdateCheck = false;
unsigned long bootTime = 0; // час старту — щоб відкласти першу перевірку оновлень
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

void setupTime() {
  if (WiFi.status() != WL_CONNECTED) return;
  // EET-2EEST,M3.5.0/3,M10.5.0/4 — часовий пояс Києва з автоматичним переходом на літній/зимовий час
  configTzTime("EET-2EEST,M3.5.0/3,M10.5.0/4", "pool.ntp.org", "time.google.com");

  unsigned long start = millis();
  time_t now = time(nullptr);
  while (now < 1700000000 && millis() - start < 3000) { // чекаємо максимум 3 сек
    delay(100);
    now = time(nullptr);
  }

  if (now >= 1700000000) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char buf[32];
    strftime(buf, sizeof(buf), "%d.%m.%Y %H:%M:%S", &timeinfo);
    Serial.printf("[NTP] Час синхронізовано: %s\n", buf);
  } else {
    Serial.println("[NTP] Не вдалось синхронізувати час за 3 сек (спробує далі у фоні)");
  }
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[WiFi-подія] Розрив з'єднання, причина: %d\n", info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi-подія] STA підключено до точки доступу");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi-подія] Отримано IP: %s, RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      break;
    default:
      break;
  }
}

void setupWiFi() {
  WiFi.onEvent(onWiFiEvent);
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false); // вимикає modem-sleep — радіо завжди активне, без затримок на "прокидання"

  // Спершу пробуємо статичний WiFi, якщо він заданий (не порожній)
  if (strlen(WIFI_STATIC_SSID) > 0) {
    Serial.printf("[WiFi] Пробую статичне підключення до \"%s\"...\n", WIFI_STATIC_SSID);
    WiFi.begin(WIFI_STATIC_SSID, WIFI_STATIC_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_STATIC_TIMEOUT_MS) {
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Статичне підключення успішне, IP: %s\n", WiFi.localIP().toString().c_str());
      return;
    }
    Serial.println("[WiFi] Статичне підключення не вдалось, переходжу на WiFiManager...");
  }

  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // 3 хв на налаштування, потім працює далі офлайн демо-режимом
  wm.setCustomHeadElement(
    "<p style='background:#2a2;color:#fff;padding:10px;border-radius:6px;text-align:center;'>"
    "Після підключення до WiFi відкрий <b>http://light-music.local</b> у браузері "
    "(телефон/комп'ютер мають бути в тій самій мережі). "
    "Якщо адреса не відкриється — подивись IP плати в налаштуваннях роутера."
    "</p>"
  );
  bool connected = wm.autoConnect("LightMusic-Setup");
  if (connected) {
    Serial.printf("WiFi підключено, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi не підключено — працюю офлайн (вебсторінка й OTA недоступні)");
  }
}

void setupOTA() {
  if (WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();
  Serial.println("ArduinoOTA готовий (заливка прошивки по WiFi з PlatformIO)");
}

void setupMDNS() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (MDNS.begin(OTA_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS готовий — відкривай http://%s.local\n", OTA_HOSTNAME);
  } else {
    Serial.println("Не вдалось запустити mDNS");
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
<div id="clock" style="font-size:48px; font-weight:bold; text-align:center; margin:10px 0; letter-spacing:2px; color:#6cf;">--:--:--</div>
<div id="status">Завантаження...</div>
<div id="updateStatus" style="margin-bottom:16px; font-size:13px; color:#999;"></div>

<div class="row">
  <button id="staticLightBtn" onclick="toggleStaticLight()">💡 Статичне світло</button>
  <input type="color" id="staticColorPicker" value="#ffffff">
</div>

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
  <span style="min-width:90px;">Тривалість</span>
  <span>⏱️</span>
  <input type="range" id="durationSlider" min="5" max="120" value="30">
  <span id="durationValue" style="min-width:60px;">30 сек</span>
</div>
<div class="row">
  <button onclick="setAuto()">Авто-перемикання ефектів</button>
  <button onclick="resetWifi()" style="background:#733;">Змінити WiFi</button>
  <button onclick="reboot()" style="background:#753;">Перезавантажити плату</button>
  <button onclick="checkUpdate()">Перевірити оновлення</button>
</div>

<div class="grid" id="effectGrid"></div>

<script>
let EFFECT_NAMES = [];
let clockOffsetMs = null;
const tickClock = () => {
  if (clockOffsetMs === null) return;
  const now = new Date(Date.now() + clockOffsetMs);
  const pad = n => String(n).padStart(2, '0');
  document.getElementById('clock').innerText =
    `${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}`;
};
setInterval(tickClock, 1000);

const loadStatus = async () => {
  try {
    const r = await fetch('/status');
    const s = await r.json();
    document.getElementById('status').innerText =
      `Ефект: ${s.effect} | Мікрофон: ${s.mic ? 'увімкнено' : 'вимкнено'} | Авто: ${s.auto ? 'так' : 'ні'}` +
      (s.auto && !s.mic ? ` | наступний через ${s.effectRemainingSec}с` : '') +
      ` | v${s.version}`;
    document.getElementById('updateStatus').innerText = 'Оновлення: ' + s.updateStatus;
    if (s.epochSec > 1700000000) {
      clockOffsetMs = s.epochSec * 1000 - Date.now();
    }
    document.getElementById('micToggle').checked = s.mic;
    const staticBtn = document.getElementById('staticLightBtn');
    staticBtn.classList.toggle('active', s.staticLight);
    if (!staticColorDragging) {
      document.getElementById('staticColorPicker').value = '#' + s.staticColor;
    }
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
  if (!durationDragging) {
    document.getElementById('durationSlider').value = s.effectDuration;
    document.getElementById('durationValue').innerText = s.effectDuration + ' сек';
  }
  document.querySelectorAll('.grid button').forEach((b,i)=>{
    b.classList.toggle('active', !s.mic && i === s.index);
  });
  } catch (err) {
    document.getElementById('status').innerText = '⚠️ Плата не відповідає (перевір WiFi/живлення)';
  }
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
let staticColorDragging = false;
const toggleStaticLight = () => {
  const isOn = document.getElementById('staticLightBtn').classList.contains('active');
  const color = document.getElementById('staticColorPicker').value.substring(1);
  fetch('/staticlight?on=' + (isOn ? '0' : '1') + '&color=' + color).then(loadStatus);
};
document.getElementById('staticColorPicker').addEventListener('input', () => {
  staticColorDragging = true;
});
document.getElementById('staticColorPicker').addEventListener('change', (e) => {
  const color = e.target.value.substring(1);
  fetch('/staticlight?on=1&color=' + color).then(loadStatus);
  staticColorDragging = false;
});

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

let durationDragging = false;
let durationDebounce = null;
const durationSlider = document.getElementById('durationSlider');
durationSlider.addEventListener('input', (e) => {
  durationDragging = true;
  document.getElementById('durationValue').innerText = e.target.value + ' сек';
  clearTimeout(durationDebounce);
  durationDebounce = setTimeout(() => {
    fetch('/effectduration?sec=' + e.target.value);
  }, 150);
});
durationSlider.addEventListener('change', () => {
  durationDragging = false;
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

fetch('/effectnames').then(r => r.json()).then(names => {
  EFFECT_NAMES = names;
  buildGrid();
  loadStatus();
  setInterval(loadStatus, 5000); // 2с -> 5с: менше TCP-з'єднань, менше навантаження на синхронний WebServer
});
</script>
</body>
</html>
)HTML";

void handleRoot() {
  // Віддаємо HTML напряму з flash, без копіювання в String і без .replace() —
  // ця операція раніше блокувала loop() (і разом з ним ефекти) на помітний час
  // при кожному відкритті сторінки. Список назв ефектів тепер підвантажується
  // окремим маленьким запитом з JS (handleEffectNames нижче).
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleEffectNames() {
  String namesJs = "[";
  for (int i = 0; i < NUM_EFFECTS; i++) {
    namesJs += "\"" + String(effectNames[i]) + "\"";
    if (i < NUM_EFFECTS - 1) namesJs += ",";
  }
  namesJs += "]";
  server.send(200, "application/json", namesJs);
}

void handleStatus() {
  JsonDocument doc;
  doc["effect"] = micEnabled ? "Світломузика (мікрофон)" : effectNames[currentEffect];
  doc["index"] = currentEffect;
  doc["mic"] = micEnabled;
  doc["staticLight"] = staticLightEnabled;
  char colorHex[7];
  sprintf(colorHex, "%02X%02X%02X", staticColor.r, staticColor.g, staticColor.b);
  doc["staticColor"] = String(colorHex);
  doc["auto"] = autoCycle;
  doc["brightness"] = currentBrightness;
  doc["songBpm"] = songBpm;
  doc["effectDuration"] = effectDuration / 1000; // в секундах для вебу
  unsigned long elapsed = millis() - lastSwitch;
  doc["effectRemainingSec"] = (effectDuration > elapsed) ? (effectDuration - elapsed) / 1000 : 0;
  doc["epochSec"] = (unsigned long)time(nullptr);
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
      Serial.printf("[web] Обрано ефект вручну: %s\n", effectNames[i]);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleSetAuto() {
  autoCycle = true;
  micEnabled = false;
  lastSwitch = millis();
  Serial.println("[web] Увімкнено авто-перемикання ефектів");
  server.send(200, "text/plain", "OK");
}

void handleMic() {
  if (server.hasArg("on")) {
    micEnabled = server.arg("on") == "1";
    if (micEnabled) autoCycle = false;
    Serial.printf("[web] Мікрофон: %s\n", micEnabled ? "увімкнено" : "вимкнено");
  }
  server.send(200, "text/plain", "OK");
}

void handleStaticLight() {
  if (server.hasArg("color")) {
    String hex = server.arg("color");
    if (hex.length() == 6) {
      long rgb = strtol(hex.c_str(), NULL, 16);
      staticColor = CRGB((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
    }
  }
  if (server.hasArg("on")) {
    staticLightEnabled = server.arg("on") == "1";
    if (staticLightEnabled) {
      micEnabled = false;
      autoCycle = false;
    } else {
      autoCycle = true;
      lastSwitch = millis();
    }
  }
  Serial.printf("[web] Статичне світло: %s\n", staticLightEnabled ? "увімкнено" : "вимкнено");
  server.send(200, "text/plain", "OK");
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
      Serial.printf("[web] Чутливість мікрофона: %.0f\n", micSensitivity);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleEffectDuration() {
  if (server.hasArg("sec")) {
    int sec = server.arg("sec").toInt();
    if (sec >= 5 && sec <= 300) {
      effectDuration = (unsigned long)sec * 1000UL;
      Serial.printf("[web] Тривалість ефекту: %d сек\n", sec);
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
      Serial.printf("[web] Яскравість: %d\n", currentBrightness);
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
  Serial.printf("[web] Застосовано темп: %.1f BPM\n", songBpm);
  server.send(200, "text/plain", "OK");
}

void handleCheckUpdate() {
  forceUpdateCheck = true;
  server.send(200, "text/plain", "OK, перевіряю...");
}

void setupWebServer() {
  if (WiFi.status() != WL_CONNECTED) return;
  server.on("/", handleRoot);
  server.on("/effectnames", handleEffectNames);
  server.on("/status", handleStatus);
  server.on("/effect", handleSetEffect);
  server.on("/auto", handleSetAuto);
  server.on("/mic", handleMic);
  server.on("/staticlight", handleStaticLight);
  server.on("/brightness", handleBrightness);
  server.on("/effectduration", handleEffectDuration);
  server.on("/micsensitivity", handleMicSensitivity);
  server.on("/checkupdate", handleCheckUpdate);
  server.on("/applysong", handleApplySong);
  server.on("/resetwifi", handleResetWifi);
  server.on("/reboot", handleReboot);
  server.begin();
  Serial.println("Веб-сервер запущений — відкрий IP плати в браузері");
}

// ---------- HTTP OTA: перевірка нової версії на Synology ----------
void checkFirmwareUpdate() {
  static unsigned long lastCheck = 0;
  static unsigned long lastFailedAttempt = 0;
  const unsigned long FAILURE_COOLDOWN = 10000UL; // 10 сек — не бомбити повторними спробами одразу після невдачі

  if (WiFi.status() != WL_CONNECTED) return;

  bool forced = forceUpdateCheck;
  if (lastFailedAttempt != 0 && millis() - lastFailedAttempt < FAILURE_COOLDOWN) {
    if (forced) {
      forceUpdateCheck = false;
      lastUpdateCheckResult = "зачекай кілька секунд після попередньої невдалої спроби";
    }
    return;
  }
  const unsigned long STARTUP_DELAY = 300000UL; // 5 хв — не блокувати щойно піднятий вебсервер одразу після старту
  if (!forced && millis() - bootTime < STARTUP_DELAY) return;
  if (!forced && lastCheck != 0 && millis() - lastCheck < UPDATE_CHECK_INTERVAL) return;
  lastCheck = millis();
  forceUpdateCheck = false;

  WiFiClientSecure client;
  client.setInsecure(); // ОК для Let's Encrypt теж; прибери й додай сертифікат, якщо хочеш строгу перевірку

  HTTPClient http;
  if (!http.begin(client, FIRMWARE_UPDATE_URL)) {
    Serial.println("[OTA] Не вдалось відкрити з'єднання для перевірки версії");
    lastUpdateCheckResult = "помилка з'єднання";
    lastFailedAttempt = millis();
    client.stop();
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
        Serial.printf("[OTA] Знайдено нову версію %s (поточна %s), оновлююсь...\n",
                      newVersion.c_str(), FIRMWARE_VERSION);
        lastUpdateCheckResult = "знайдено v" + newVersion + ", оновлююсь...";
        httpUpdate.onProgress([](int cur, int total) {
          static int lastPercent = -1;
          int percent = total > 0 ? (cur * 100 / total) : 0;
          if (percent != lastPercent && percent % 10 == 0) {
            Serial.printf("[OTA] Завантаження: %d%%\n", percent);
            lastPercent = percent;
          }
        });
        t_httpUpdate_return ret = httpUpdate.update(client, binUrl);
        if (ret == HTTP_UPDATE_FAILED) {
          Serial.printf("[OTA] Помилка оновлення: %s\n", httpUpdate.getLastErrorString().c_str());
          lastUpdateCheckResult = "помилка оновлення: " + String(httpUpdate.getLastErrorString().c_str());
          lastFailedAttempt = millis();
        }
        // при успіху плата сама перезавантажиться
      } else {
        Serial.println("[OTA] Версія актуальна");
        lastUpdateCheckResult = "версія актуальна (v" + String(FIRMWARE_VERSION) + ")";
      }
    } else {
      lastUpdateCheckResult = "помилка розбору version.json";
    }
  } else {
    Serial.printf("[OTA] Не вдалось перевірити версію, код: %d\n", code);
    lastUpdateCheckResult = "помилка перевірки, код " + String(code);
    lastFailedAttempt = millis();
  }
  http.end();
  client.stop();
}

// ================================================================
//                          SETUP / LOOP
// ================================================================

// Стандартний стек loop()-задачі ESP32 — лише 8КБ, замало для такого обсягу коду
// (WiFiManager + ArduinoJson + HTTPClient + FastLED на 40 ефектів + великі HTML-рядки).
// Це офіційний спосіб від Espressif збільшити його без правки core-файлів.
SET_LOOP_TASK_STACK_SIZE(24 * 1024);

void setup() {
  bootTime = millis();
  Serial.begin(115200);
  Serial.printf("=== Light_music firmware v%s ===\n", FIRMWARE_VERSION);

  prefs.begin("lightmusic", false);
  String lastVersion = prefs.getString("version", "");
  if (lastVersion.length() && lastVersion != FIRMWARE_VERSION) {
    lastUpdateCheckResult = "успішно оновлено з v" + lastVersion + " до v" + String(FIRMWARE_VERSION) + "!";
    Serial.println("[OTA] " + lastUpdateCheckResult);
  }
  prefs.putString("version", FIRMWARE_VERSION);

  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(currentBrightness);
  FastLED.clear();
  FastLED.show();

  setupWiFi();
  setupTime();
  setupOTA();
  setupMDNS();
  setupWebServer();
  setupI2S();
  for (int i = 0; i < NUM_BANDS; i++) bandPeaks[i] = 0;
  wasWifiConnected = (WiFi.status() == WL_CONNECTED);

  lastSwitch = millis();
  Serial.printf("Демо-режим. Ефект 1/%d: %s\n", NUM_EFFECTS, effectNames[0]);
}

void loop() {
  // ---- Фізична кнопка: утримання 5 сек скидає WiFi ----
  bool buttonPressed = (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW);
  if (buttonPressed && !buttonWasPressed) {
    buttonPressStart = millis();
  }
  if (buttonPressed && buttonWasPressed) {
    if (millis() - buttonPressStart >= WIFI_RESET_HOLD_MS) {
      Serial.println("[кнопка] Утримання 5 сек — скидаю WiFi і перезавантажуюсь");
      WiFiManager wm;
      wm.resetSettings();
      delay(100);
      ESP.restart();
    }
  }
  buttonWasPressed = buttonPressed;

  EVERY_N_SECONDS(60) {
    UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);
    if (freeStack < 1024) {
      Serial.printf("[УВАГА] Мало вільного стеку: %u байт лишилось!", (unsigned)freeStack);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!wasWifiConnected) {
      // Щойно відновилось з'єднання (не просто перший запуск) — перезапускаємо
      // mDNS/OTA, бо вони інколи "не оживають" самі після реального обриву
      Serial.printf("[WiFi] Підключення відновлено, IP: %s\n", WiFi.localIP().toString().c_str());
      setupMDNS();
      setupOTA();
      wasWifiConnected = true;
    }
    server.handleClient();
    ArduinoOTA.handle();
    // Автоматична фонова перевірка ТИМЧАСОВО вимкнена — підозра, що саме OTA-процес
    // пов'язаний з крашами heap corruption (маркер 0xbaad5678 у дампі стеку).
    // Кнопка "Перевірити оновлення" на вебсторінці й далі працює (forceUpdateCheck),
    // бо форсована перевірка минає цю перевірку нижче.
    if (forceUpdateCheck) checkFirmwareUpdate();
  } else {
    wasWifiConnected = false;
    // WiFi відпав — пробуємо перепідключитись раз на WIFI_RECONNECT_INTERVAL,
    // не блокуючи основний цикл (ефекти й далі йдуть, поки чекаємо мережу)
    if (millis() - lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
      lastWifiReconnectAttempt = millis();
      Serial.println("[WiFi] З'єднання втрачено, пробую перепідключитись...");
      WiFi.reconnect();
    }
  }

  if (staticLightEnabled) {
    EVERY_N_MILLISECONDS(50) {
      fill_solid(leds, NUM_LEDS, staticColor);
      FastLED.show();
    }
  } else if (micEnabled) {
    readAudioAndFFT();
    renderSpectrum();
    detectBeatAndUpdateBpm();
    FastLED.show();
  } else {
    unsigned long now = millis();
    if (autoCycle && now - lastSwitch >= effectDuration) {
      lastSwitch = now;
      currentEffect = (currentEffect + 1) % NUM_EFFECTS;
      FastLED.clear();
      Serial.printf("Перемикаю на ефект %d/%d: %s\n", currentEffect + 1, NUM_EFFECTS, effectNames[currentEffect]);
    }
    // Фіксований кадр ~60 FPS замість delay(10) — сталіша частота кадрів,
    // і loop() крутиться швидше між кадрами, встигаючи частіше обслуговувати
    // веб-сервер/OTA/кнопку, поки чекає наступного кадру
    EVERY_N_MILLISECONDS(16) {
      effects[currentEffect]();
      FastLED.show();
    }
  }

  // Критично: WebServer.handleClient() у "тугому" циклі без жодного yield
  // "заморює" внутрішній таймер lwIP (офіційно задокументована проблема
  // arduino-esp32, issue #4348) — з часом це призводить саме до того,
  // що ми бачили: ERR_CONNECTION_REFUSED після тривалої роботи.
  // 1мс з 16.67мс бюджету кадру (60 FPS) — непомітно для плавності.
  delay(1);
}
