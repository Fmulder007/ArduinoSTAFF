/*
  ZeroCounter — частотомер для Waveshare RP2040-Zero
  Версия 0.2.0

  Исправление 2026-11-26 13:25:06 (+10:00), строки 553–555:
    начальные значения входного и опорного счётчиков фиксируются
    из текущих inputNow/referenceNow вместо предыдущего снимка.

  Сейчас реализован только ВЧ-вход B.
  НЧ-вход A будет добавлен отдельно.

  Измерение:
    - Вход B: аппаратный счётчик фронтов PWM после внешнего прескалера.
    - Опора: отдельный аппаратный счётчик фронтов PWM от TCXO.
    - Частота = delta_B * PRESCALER_K * REF_FREQ_HZ / delta_REF.
    - Измерение накопительное: пока сигнал не пропадает, база копится,
      а показания обновляются на границах опорного окна — каждые
      GATE_REF_COUNTS периодов TCXO. С каждым обновлением точность
      растёт (±1 импульс опоры и ±1 импульс входа на всю базу).
    - Обнуление измерительной базы происходит только после
      пропадания входных импульсов.

  Требуется Arduino-Pico core by Earle F. Philhower.
*/

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pwm.h>
#include <hardware/sync.h>

// ============================================================
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
// ============================================================

// Частота внешнего TCXO, Гц. Значение может быть любым целым —
// сюда вносится фактическая, калиброванная частота опоры,
// например 9999987UL. Она используется непосредственно в
// формуле пересчёта частоты, а не как номинал.
#define REF_FREQ_HZ       10000000UL

// Коэффициент внешнего прескалера на входе B.
#define PRESCALER_K       64UL

// Ориентировочная длительность опорного окна, мс.
#define GATE_MS           100UL

// Окно счёта в импульсах опоры: REF_FREQ_HZ * GATE_MS / 1000.
// Приведение к uint64_t обязательно: произведение может
// достигать 4e10 и не помещается в 32-битный unsigned long.
// Каст допустим, потому что макрос используется только в
// рантайме — препроцессорных проверок с ним больше нет.
// Усечение деления (если REF_FREQ_HZ не кратен 1000) на точность
// не влияет: длина окна лишь чуть отличается от задуманной,
// а в формуле участвует фактическое число импульсов опоры.
#define GATE_REF_COUNTS   ((uint64_t)REF_FREQ_HZ * GATE_MS / 1000UL)

#if (REF_FREQ_HZ < 4000000UL) || (REF_FREQ_HZ > 40000000UL)
  #error "REF_FREQ_HZ must be in range 4000000...40000000 Hz"
#endif

#if (PRESCALER_K < 1UL) || (PRESCALER_K > 256UL)
  #error "PRESCALER_K must be in range 1...256"
#endif

#if (GATE_MS < 1UL)
  #error "GATE_MS must be at least 1 ms"
#endif


// ============================================================
// РАСПИНОВКА RP2040-ZERO
// ============================================================

// OLED SSD1306, I2C0.
#define PIN_OLED_SDA       4
#define PIN_OLED_SCL       5
#define OLED_I2C_ADDRESS   0x3C

// Вход B: выход внешнего ВЧ-прескалера.
// GPIO7 = PWM slice 3, канал B.
#define PIN_INPUT_B        7

// Опорный вход: цифровой сигнал TCXO.
// GPIO9 = PWM slice 4, канал B.
#define PIN_REFERENCE      9

// Цифровой вход готовности термостата опоры.
// HIGH = термостат сообщает о готовности.
#define PIN_REF_READY      12

// Период обновления индикации и тайм-аут пропадания импульсов.
#define DISPLAY_INTERVAL_MS  100UL
#define SIGNAL_TIMEOUT_MS    100UL

static_assert((PIN_INPUT_B & 1u) == 1u,
              "PIN_INPUT_B must be a PWM channel B pin");
static_assert((PIN_REFERENCE & 1u) == 1u,
              "PIN_REFERENCE must be a PWM channel B pin");
static_assert(PIN_INPUT_B != PIN_REFERENCE,
              "Input and reference must use different pins");

// ============================================================
// АППАРАТНЫЕ СЧЁТЧИКИ PWM
// ============================================================

// PWM-счётчики имеют разрядность 16 бит.
// Переполнения расширяются программными счётчиками до 64 бит.
static uint gInputSlice = 0;
static uint gReferenceSlice = 0;

static volatile uint64_t gInputWraps = 0;
static volatile uint64_t gReferenceWraps = 0;

static void pwmWrapISR() {
  const uint32_t status = pwm_get_irq_status_mask();

  if (status & (1u << gInputSlice)) {
    pwm_clear_irq(gInputSlice);
    ++gInputWraps;
  }

  if (status & (1u << gReferenceSlice)) {
    pwm_clear_irq(gReferenceSlice);
    ++gReferenceWraps;
  }
}

static uint initRisingEdgeCounter(uint gpioPin) {
  const uint slice = pwm_gpio_to_slice_num(gpioPin);

  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, 0xFFFFu);
  pwm_config_set_clkdiv_int(&config, 1u);
  pwm_config_set_clkdiv_mode(&config, PWM_DIV_B_RISING);

  // Сначала настраиваем PWM, не запуская его.
  pwm_init(slice, &config, false);

  // Вход должен быть подключён именно к каналу B.
  gpio_pull_down(gpioPin);
  gpio_set_function(gpioPin, GPIO_FUNC_PWM);

  pwm_set_counter(slice, 0);
  pwm_clear_irq(slice);

  return slice;
}

static void initFrequencyCounters() {
  gInputSlice = initRisingEdgeCounter(PIN_INPUT_B);
  gReferenceSlice = initRisingEdgeCounter(PIN_REFERENCE);

  // Общий обработчик переполнений двух PWM-срезов RP2040.
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwmWrapISR);

  pwm_set_irq_enabled(gInputSlice, true);
  pwm_set_irq_enabled(gReferenceSlice, true);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  pwm_set_enabled(gInputSlice, true);
  pwm_set_enabled(gReferenceSlice, true);
}

// Снимок 16-битного PWM-счётчика вместе с числом переполнений.
// Проверка pending-флага закрывает гонку на границе переполнения.
static uint64_t readExtendedCounter(uint slice,
                                    volatile uint64_t &wrapCounter) {
  const uint32_t irqState = save_and_disable_interrupts();

  uint64_t wraps = wrapCounter;
  const uint16_t before = pwm_get_counter(slice);
  const uint32_t mask = 1u << slice;
  const bool wrapPending = (pwm_get_irq_status_mask() & mask) != 0;
  const uint16_t after = pwm_get_counter(slice);

  if (wrapPending || (after < before)) {
    ++wraps;
    wrapCounter = wraps;
    pwm_clear_irq(slice);
  }

  restore_interrupts(irqState);

  return (wraps << 16) | after;
}

// ============================================================
// ЦЕЛОЧИСЛЕННЫЙ РАСЧЁТ ЧАСТОТЫ
// ============================================================

// Вычисляет floor(a * b / divisor) и остаток.
// Предполагается: a < divisor. Умножение не переполняет uint64_t.
static void multiplyFractionDivide(uint64_t a,
                                   uint32_t b,
                                   uint64_t divisor,
                                   uint64_t &quotient,
                                   uint64_t &remainder) {
  uint64_t q = 0;
  uint64_t r = 0;

  // Двоичное умножение с делением по ходу вычисления.
  // Все промежуточные остатки остаются меньше divisor.
  for (int bit = 31; bit >= 0; --bit) {
    q <<= 1;

    // r = (2 * r) % divisor, с учётом переноса в частное.
    const uint64_t twiceLimit = divisor - r;
    if (r >= twiceLimit) {
      r -= twiceLimit;
      ++q;
    } else {
      r += r;
    }

    if ((b >> bit) & 1u) {
      // r = (r + a) % divisor, с учётом переноса в частное.
      const uint64_t addLimit = divisor - a;
      if (r >= addLimit) {
        r -= addLimit;
        ++q;
      } else {
        r += a;
      }
    }
  }

  quotient = q;
  remainder = r;
}

// Округление результата до ближайшего целого герца.
// Все вычисления выполняются целочисленно.
static uint64_t multiplyDivideRound(uint64_t a,
                                    uint32_t b,
                                    uint64_t divisor) {
  if (divisor == 0) {
    return 0;
  }

  const uint64_t whole = a / divisor;
  const uint64_t part = a % divisor;

  uint64_t fractionalQuotient = 0;
  uint64_t fractionalRemainder = 0;

  multiplyFractionDivide(part, b, divisor,
                         fractionalQuotient,
                         fractionalRemainder);

  uint64_t result = whole * (uint64_t)b + fractionalQuotient;

  // Округление половины вверх без вычисления 2 * remainder.
  const uint64_t roundThreshold =
      (divisor >> 1) + (divisor & 1u);

  if (fractionalRemainder >= roundThreshold) {
    ++result;
  }

  return result;
}

// ============================================================
// МИНИМАЛЬНЫЙ ДРАЙВЕР OLED SSD1306
// Без Adafruit_GFX и дополнительных библиотек.
// ============================================================

static uint8_t oledBuffer[128 * 64 / 8];

static const uint8_t digitFont[10][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};

static const uint8_t glyphH[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
static const uint8_t glyphZ[5] = {0x41, 0x61, 0x51, 0x49, 0x46};
static const uint8_t glyphC[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
static const uint8_t glyphN[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
static const uint8_t glyphT[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
static const uint8_t glyphR[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
static const uint8_t glyphE[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
static const uint8_t glyphF[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};

static void oledCommandList(const uint8_t *commands, size_t count) {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  Wire.write(0x00);

  for (size_t i = 0; i < count; ++i) {
    Wire.write(commands[i]);
  }

  Wire.endTransmission();
}

static void oledBegin() {
  Wire.setSDA(PIN_OLED_SDA);
  Wire.setSCL(PIN_OLED_SCL);
  Wire.begin();
  Wire.setClock(400000);

  const uint8_t initCommands[] = {
    0xAE,       // Display OFF
    0xD5, 0x80, // Clock divide
    0xA8, 0x3F, // Multiplex 1/64
    0xD3, 0x00, // Display offset
    0x40,       // Start line 0
    0x8D, 0x14, // Charge pump ON
    0x20, 0x00, // Horizontal addressing mode
    0xA1,       // Segment remap
    0xC8,       // COM scan direction
    0xDA, 0x12, // COM pins
    0x81, 0x7F, // Contrast
    0xD9, 0xF1, // Pre-charge
    0xDB, 0x40, // VCOMH
    0xA4,       // Display follows RAM
    0xA6,       // Normal display
    0xAF        // Display ON
  };

  oledCommandList(initCommands, sizeof(initCommands));
}

static void oledClear() {
  memset(oledBuffer, 0, sizeof(oledBuffer));
}

static void oledPixel(int x, int y) {
  if (x < 0 || x >= 128 || y < 0 || y >= 64) {
    return;
  }

  oledBuffer[x + (y >> 3) * 128] |= (uint8_t)(1u << (y & 7));
}

static void oledFillRect(int x, int y, int width, int height) {
  for (int yy = 0; yy < height; ++yy) {
    for (int xx = 0; xx < width; ++xx) {
      oledPixel(x + xx, y + yy);
    }
  }
}

static void oledDrawDigit(uint8_t digit, int x, int y, uint8_t scale) {
  if (digit > 9) {
    return;
  }

  for (uint8_t column = 0; column < 5; ++column) {
    const uint8_t bits = digitFont[digit][column];

    for (uint8_t row = 0; row < 7; ++row) {
      if (bits & (1u << row)) {
        oledFillRect(x + column * scale,
                     y + row * scale,
                     scale,
                     scale);
      }
    }
  }
}

static const uint8_t *oledGlyph(char c) {
  switch (c) {
    case 'H': return glyphH;
    case 'Z': return glyphZ;
    case 'C': return glyphC;
    case 'N': return glyphN;
    case 'T': return glyphT;
    case 'R': return glyphR;
    case 'E': return glyphE;
    case 'F': return glyphF;
    default:  return nullptr;
  }
}

static void oledDrawText(const char *text, int x, int y) {
  while (*text) {
    const uint8_t *glyph = oledGlyph(*text++);

    if (glyph) {
      for (uint8_t column = 0; column < 5; ++column) {
        for (uint8_t row = 0; row < 7; ++row) {
          if (glyph[column] & (1u << row)) {
            oledPixel(x + column, y + row);
          }
        }
      }
    }

    x += 6;
  }
}

static void oledDrawIndicator(int cx, int cy, bool active) {
  const int radius = 3;
  const int radiusSquared = radius * radius;
  const int innerSquared = (radius - 1) * (radius - 1);

  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int distanceSquared = dx * dx + dy * dy;

      if (distanceSquared <= radiusSquared &&
          (active || distanceSquared >= innerSquared)) {
        oledPixel(cx + dx, cy + dy);
      }
    }
  }
}

static void oledFlush() {
  const uint8_t addressCommands[] = {
    0x21, 0x00, 0x7F, // Column address 0..127
    0x22, 0x00, 0x07  // Page address 0..7
  };

  oledCommandList(addressCommands, sizeof(addressCommands));

  // 31 байт данных + управляющий байт помещаются даже
  // в стандартный 32-байтовый буфер Wire.
  for (size_t offset = 0; offset < sizeof(oledBuffer); offset += 31) {
    const size_t remaining = sizeof(oledBuffer) - offset;
    const size_t chunk = (remaining > 31) ? 31 : remaining;

    Wire.beginTransmission(OLED_I2C_ADDRESS);
    Wire.write(0x40);

    for (size_t i = 0; i < chunk; ++i) {
      Wire.write(oledBuffer[offset + i]);
    }

    Wire.endTransmission();
  }
}

static void oledShowFrequency(uint32_t frequencyHz,
                              bool countActive,
                              bool referenceReady) {
  oledClear();

  // Девять цифр всегда остаются на одном месте.
  // Между цифрами внутри тройки — 2 пикселя,
  // между группами тысяч — 8 пикселей.
  char digits[10];

  for (int i = 8; i >= 0; --i) {
    digits[i] = (char)('0' + (frequencyHz % 10u));
    frequencyHz /= 10u;
  }
  digits[9] = '\0';

  int x = 5;

  for (uint8_t i = 0; i < 9; ++i) {
    oledDrawDigit((uint8_t)(digits[i] - '0'), x, 3, 2);
    x += 12;

    if (i == 2 || i == 5) {
      x += 6;
    }
  }

  oledDrawText("HZ", 58, 21);

  oledDrawText("CNT", 16, 50);
  oledDrawIndicator(43, 53, countActive);

  oledDrawText("REF", 79, 50);
  oledDrawIndicator(106, 53, referenceReady);

  oledFlush();
}

// ============================================================
// СОСТОЯНИЕ ИЗМЕРЕНИЯ
// ============================================================

static uint64_t previousInputCount = 0;
static uint64_t previousReferenceCount = 0;

static uint64_t measurementStartInput = 0;
static uint64_t measurementStartReference = 0;

// Снимок опорного счётчика на момент последнего обновления
// показаний: границы обновлений привязаны к опоре.
static uint64_t lastUpdateReference = 0;

static uint32_t lastPulseMs = 0;
static uint32_t lastDisplayMs = 0;

static bool measuring = false;
static bool countSinceDisplay = false;

static uint32_t displayedFrequencyHz = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {
  pinMode(PIN_REF_READY, INPUT_PULLDOWN);

  oledBegin();
  initFrequencyCounters();

  previousInputCount =
      readExtendedCounter(gInputSlice, gInputWraps);
  previousReferenceCount =
      readExtendedCounter(gReferenceSlice, gReferenceWraps);

  const uint32_t now = millis();
  lastPulseMs = now;
  lastDisplayMs = now;

  oledShowFrequency(0, false, digitalRead(PIN_REF_READY) == HIGH);
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  // Считываем оба аппаратных счётчика. Сам подсчёт фронтов
  // идёт независимо от выполнения loop() и передачи OLED.
  const uint64_t inputNow =
      readExtendedCounter(gInputSlice, gInputWraps);

  const uint64_t referenceNow =
      readExtendedCounter(gReferenceSlice, gReferenceWraps);

  const uint32_t now = millis();

  // Обнаружили новые импульсы на входе B.
  if (inputNow != previousInputCount) {
    lastPulseMs = now;
    countSinceDisplay = true;

    // Начало нового измерения — снимок непосредственно
    // перед первым обнаруженным изменением счётчика.
    if (!measuring) {
      measurementStartInput = inputNow;
      measurementStartReference = referenceNow;
      lastUpdateReference = referenceNow;
      measuring = true;
      displayedFrequencyHz = 0;
    }
  }

  previousInputCount = inputNow;
  previousReferenceCount = referenceNow;

  // Сигнал пропал — измерение завершено, база обнуляется.
  if (measuring &&
      (uint32_t)(now - lastPulseMs) >= SIGNAL_TIMEOUT_MS) {
    measuring = false;
    displayedFrequencyHz = 0;
    countSinceDisplay = false;
  }

  // Показания обновляются на границах опорного окна:
  // каждые GATE_REF_COUNTS периодов опоры. Само измерение
  // при этом НЕ закрывается — база копится, пока идёт сигнал,
  // и точность показаний растёт со временем измерения.
  if (measuring) {
    if (referenceNow - lastUpdateReference >= GATE_REF_COUNTS) {
      const uint64_t deltaInput =
          inputNow - measurementStartInput;

      const uint64_t deltaReference =
          referenceNow - measurementStartReference;

      if (deltaInput > 0 && deltaReference > 0) {
        const uint64_t correctedInput =
            deltaInput * (uint64_t)PRESCALER_K;

        uint64_t resultHz =
            multiplyDivideRound(correctedInput,
                                (uint32_t)REF_FREQ_HZ,
                                deltaReference);

        // На экране предусмотрено девять разрядов.
        if (resultHz > 999999999ULL) {
          resultHz = 999999999ULL;
        }

        displayedFrequencyHz = (uint32_t)resultHz;
      } else {
        displayedFrequencyHz = 0;
      }

      lastUpdateReference = referenceNow;
      countSinceDisplay = true;
    }
  }

  // Перерисовка OLED раз в DISPLAY_INTERVAL_MS.
  if ((uint32_t)(now - lastDisplayMs) >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs += DISPLAY_INTERVAL_MS;

    const bool referenceReady =
        (digitalRead(PIN_REF_READY) == HIGH);

    oledShowFrequency(displayedFrequencyHz,
                      countSinceDisplay,
                      referenceReady);

    countSinceDisplay = false;
  }
}
