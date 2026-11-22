/*
  ArduinoUPS — RP2040-Zero, четыре выхода PIO, RUN, АКБ и журнал событий.

  Выходы драйверов:
    GP0 — DRIVER_A_H: верхнее плечо драйвера A.
    GP1 — DRIVER_A_L: нижнее плечо драйвера A.
    GP2 — DRIVER_B_H: верхнее плечо драйвера B.
    GP3 — DRIVER_B_L: нижнее плечо драйвера B.

  Входы:
    GP4  — RUN: HIGH = генерация нужна, LOW = сеть появилась.
    GP26 — измерение напряжения аккумулятора, ADC0.

  OLED SSD1306 128x32 и будущие часы DS1307:
    GP8 — I2C0 SDA (Wire).
    GP9 — I2C0 SCL (Wire).
    GP16 — встроенный RGB-светодиод WS2812 (PIO1).

  Пока дисплей показывает только журнал событий.
  Дата/время временно заменены заглушкой до подключения DS1307.

  PIO0 самостоятельно формирует все четыре сигнала драйверов.
  PIO1 отдельно обслуживает встроенный WS2812.
  CPU не обслуживает отдельные фронты силовых сигналов и не участвует
  в формировании их временных интервалов.
*/

#include <Arduino.h>
#include <Wire.h>
#include "src/libraries/GyverOLED/src/GyverOLED.h"
#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "driver_pio.h"
#include "status_led.h"

// ======================================================
// ВЫВОДЫ RP2040-ZERO
// ======================================================

constexpr uint8_t PIN_DRIVER_A_H = 0;  // GP0
constexpr uint8_t PIN_DRIVER_A_L = 1;  // GP1
constexpr uint8_t PIN_DRIVER_B_H = 2;  // GP2
constexpr uint8_t PIN_DRIVER_B_L = 3;  // GP3

constexpr uint8_t PIN_RUN = 4;         // GP4
constexpr uint8_t PIN_I2C_SDA = 8;     // GP8 / I2C0 SDA
constexpr uint8_t PIN_I2C_SCL = 9;     // GP9 / I2C0 SCL
constexpr uint8_t PIN_BATTERY = 26;    // GP26 / ADC0

// ======================================================
// ПАРАМЕТРЫ ГЕНЕРАТОРА
// ======================================================

#define UPS_FREQUENCY_HZ                 50UL
#define UPS_DEADTIME_PERCENT              10UL
#define UPS_DRIVER_DEADTIME_US            50UL
#define UPS_STARTUP_DELAY_MS            1000UL
#define UPS_RUN_OFF_DELAY_MS            5000UL

// PIO работает на 2,5 МГц: один цикл = 0,4 мкс.
// При штатной частоте clk_sys = 125 МГц делитель равен ровно 50.
constexpr uint32_t PIO_CLOCK_HZ = 2500000UL;

// PIO IRQ0 — внутренний флаг запроса остановки.
// IRQ0 = 1: завершить текущий полный период A+B и ждать.
// IRQ0 = 0: разрешить следующий полный период.
constexpr uint8_t PIO_IRQ_STOP_REQUEST = 0;

// ======================================================
// ПАРАМЕТРЫ ИЗМЕРЕНИЯ АКБ
// ======================================================

#define BATTERY_MEASURE_INTERVAL_MS       1000UL
#define BATTERY_ADC_SAMPLES                 16
#define BATTERY_ADC_REFERENCE_MV          3300UL
#define BATTERY_ADC_MAX_COUNTS            4095UL

#define BATTERY_DIVIDER_TOP_OHM         330000UL
#define BATTERY_DIVIDER_BOTTOM_OHM       10000UL

// Пороги задаются в сотых долях вольта.
// Нули временно отключают батарейную отсечку для макета.
#define BATTERY_GENERATOR_STOP_CV             0
#define BATTERY_GENERATOR_RECOVERY_CV         0
#define BATTERY_CHARGE_ON_CV                  0
#define BATTERY_CHARGE_OFF_CV                 0

// ======================================================
// БИТЫ ПРИЧИН ЗАПРЕТА ГЕНЕРАЦИИ
// ======================================================

#define BLOCK_BATTERY_LOW                   0x01
#define BLOCK_OVERHEAT                      0x02
#define BLOCK_OVERLOAD                     0x04
#define BLOCK_MAINS_FAULT                  0x08
#define BLOCK_SENSOR_FAULT                 0x10
#define BLOCK_RESERVED_5                   0x20
#define BLOCK_RESERVED_6                   0x40
#define BLOCK_RESERVED_7                   0x80

// 0 — запретов нет; любое ненулевое значение запрещает генерацию.
// В этой версии фактически работает батарейный бит.
// Остальные биты зарезервированы для будущих модулей защиты.
volatile uint8_t generatorBlockFlags = 0;
uint8_t lastLoggedBlockFlags = 0;

// Последнее измеренное напряжение — сотые доли вольта.
// Например, 1234 означает 12,34 В.
uint16_t batteryVoltageCentiV = 0;
uint32_t lastBatteryMeasureMs = 0;

// ======================================================
// СОСТОЯНИЕ RUN
// ======================================================

bool runLowTiming = false;
bool mainsPresentConfirmed = false;
bool generatorRunRequest = false;
uint32_t runLowSince = 0;

// Последнее разрешение, переданное автомату PIO.
bool pioRunPermissionApplied = false;

// Фактическое состояние PIO-генератора, доступное другим модулям.
volatile bool generatorIsRunning = false;
static int generatorPioSm = -1;

// ======================================================
// OLED И ЖУРНАЛ
// ======================================================

// GyverOLED 1.6.4, SSD1306 128x32, программный буфер 512 байт.
// Два экземпляра нужны для сохранения автоматического выбора адреса 0x3C/0x3D.
using OledType = GyverOLED<SSD1306_128x32, OLED_BUFFER>;
OledType oled3C(0x3C);
OledType oled3D(0x3D);
OledType *oled = nullptr;

constexpr uint8_t OLED_ADDRESS_1 = 0x3C;
constexpr uint8_t OLED_ADDRESS_2 = 0x3D;
constexpr uint32_t I2C_CLOCK_HZ = 100000UL;

constexpr uint8_t LOG_HISTORY_LINES = 20;
constexpr size_t LOG_LINE_BYTES = 96;
constexpr uint8_t OLED_VISIBLE_LINES = 4;
constexpr uint8_t OLED_LINE_CHARS = 21;

char logHistory[LOG_HISTORY_LINES][LOG_LINE_BYTES];
uint8_t logHistoryHead = 0;
uint8_t logHistoryCount = 0;

bool oledReady = false;
bool oledDirty = false;

// ======================================================
// БЕЗОПАСНОЕ СОСТОЯНИЕ ПРИ ОШИБКЕ ИНИЦИАЛИЗАЦИИ
// ======================================================

static void holdAllDriverOutputsLow() {
  const uint8_t pins[] = {
    PIN_DRIVER_A_H,
    PIN_DRIVER_A_L,
    PIN_DRIVER_B_H,
    PIN_DRIVER_B_L
  };

  for (uint8_t i = 0; i < sizeof(pins); ++i) {
    gpio_set_function(pins[i], GPIO_FUNC_SIO);
    gpio_set_dir(pins[i], GPIO_OUT);
    gpio_put(pins[i], false);
  }

  while (true) {
    // Ошибка конфигурации: силовые ключи не запускать.
  }
}

// ======================================================
// ВРЕМЯ ДЛЯ ЖУРНАЛА
// ======================================================

// Временная заглушка до подключения DS1307.
// Позже меняется только эта функция.
static void getLogTimestamp(char *buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "--.--.---- --:--:--");
}

// ======================================================
// ВЫВОД СОБЫТИЯ В SERIAL И OLED
// ======================================================

static void appendDisplayLine(const char *line) {
  strncpy(logHistory[logHistoryHead], line, LOG_LINE_BYTES - 1);
  logHistory[logHistoryHead][LOG_LINE_BYTES - 1] = '\0';

  logHistoryHead++;
  if (logHistoryHead >= LOG_HISTORY_LINES) {
    logHistoryHead = 0;
  }

  if (logHistoryCount < LOG_HISTORY_LINES) {
    logHistoryCount++;
  }

  oledDirty = true;
}

static uint8_t utf8CharacterBytes(uint8_t firstByte) {
  if (firstByte < 0x80) return 1;
  if ((firstByte & 0xE0) == 0xC0) return 2;
  if ((firstByte & 0xF0) == 0xE0) return 3;
  if ((firstByte & 0xF8) == 0xF0) return 4;
  return 1;
}

// GyverOLED печатает UTF-8 кириллицу через Print.
// Делим журнал на строки по 21 символу (6 пикселей на знак).
// Разрезать многобайтный UTF-8 символ между строками нельзя.
static void appendWrappedDisplayText(const char *text) {
  if (!oledReady) {
    return;
  }

  char line[LOG_LINE_BYTES];
  size_t lineLength = 0;
  uint8_t characterCount = 0;
  line[0] = '\0';

  const uint8_t *source = reinterpret_cast<const uint8_t *>(text);

  while (*source != 0) {
    uint8_t characterLength = utf8CharacterBytes(*source);

    if (characterLength > 1) {
      for (uint8_t i = 1; i < characterLength; ++i) {
        if (source[i] == 0 || (source[i] & 0xC0) != 0x80) {
          characterLength = 1;
          break;
        }
      }
    }

    if (lineLength + characterLength >= sizeof(line)) {
      line[lineLength] = '\0';
      appendDisplayLine(line);
      lineLength = 0;
      characterCount = 0;
    }

    for (uint8_t i = 0; i < characterLength; ++i) {
      line[lineLength++] = static_cast<char>(source[i]);
    }
    line[lineLength] = '\0';
    source += characterLength;
    characterCount++;

    if (characterCount >= OLED_LINE_CHARS) {
      appendDisplayLine(line);
      lineLength = 0;
      characterCount = 0;
      line[0] = '\0';
    }
  }

  if (lineLength > 0) {
    appendDisplayLine(line);
  }
}

static void updateLogDisplay() {
  if (!oledReady || !oledDirty || oled == nullptr) {
    return;
  }

  oled->clear();
  oled->setScale(1);
  oled->autoPrintln(false);

  uint8_t visibleCount = logHistoryCount;
  if (visibleCount > OLED_VISIBLE_LINES) {
    visibleCount = OLED_VISIBLE_LINES;
  }

  const uint8_t oldestIndex =
      (logHistoryHead + LOG_HISTORY_LINES - logHistoryCount) %
      LOG_HISTORY_LINES;

  const uint8_t firstVisible =
      logHistoryCount - visibleCount;

  for (uint8_t row = 0; row < visibleCount; ++row) {
    const uint8_t index =
        (oldestIndex + firstVisible + row) % LOG_HISTORY_LINES;

    oled->setCursor(0, row);
    oled->print(logHistory[index]);
  }

  oled->update();
  oledDirty = false;
}

static void logEvent(const char *tag, const char *message) {
  char timestamp[24];
  char output[192];

  getLogTimestamp(timestamp, sizeof(timestamp));

  snprintf(
      output,
      sizeof(output),
      "%s [%s] %s",
      timestamp,
      tag,
      message);

  // USB CDC не ждём: если терминал не открыт, событие остаётся на OLED.
  if (Serial) {
    Serial.println(output);
  }

  appendWrappedDisplayText(output);
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ OLED НА I2C0
// ======================================================

static bool probeI2cAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static void initializeLogDisplay() {
  // Arduino-Pico: сначала назначаем выводы I2C0, затем запускаем Wire.
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  if (probeI2cAddress(OLED_ADDRESS_1)) {
    oled = &oled3C;
  } else if (probeI2cAddress(OLED_ADDRESS_2)) {
    oled = &oled3D;
  } else {
    oled = nullptr;
    oledReady = false;
    return;
  }

  // GyverOLED вызывает Wire.begin() внутри init().
  oled->init();
  Wire.setClock(I2C_CLOCK_HZ);
  oled->setScale(1);
  oled->autoPrintln(false);

  oledReady = true;
  oledDirty = true;
  updateLogDisplay();
}

// ОПИСАНИЯ ПРИЧИН ЗАПРЕТА
// ======================================================

static const char *blockReasonText(uint8_t bitMask) {
  switch (bitMask) {
    case BLOCK_BATTERY_LOW:
      return "Низкое напряжение аккумулятора";

    case BLOCK_OVERHEAT:
      return "Перегрев";

    case BLOCK_OVERLOAD:
      return "Перегрузка";

    case BLOCK_MAINS_FAULT:
      return "Неисправность или недопустимое состояние сети";

    case BLOCK_SENSOR_FAULT:
      return "Неисправность датчика";

    case BLOCK_RESERVED_5:
      return "Резервная причина запрета 5";

    case BLOCK_RESERVED_6:
      return "Резервная причина запрета 6";

    case BLOCK_RESERVED_7:
      return "Резервная причина запрета 7";

    default:
      return "Неизвестная причина запрета";
  }
}

static const char *blockClearedText(uint8_t bitMask) {
  switch (bitMask) {
    case BLOCK_BATTERY_LOW:
      return "Блокировка по низкому напряжению снята";

    case BLOCK_OVERHEAT:
      return "Блокировка по перегреву снята";

    case BLOCK_OVERLOAD:
      return "Блокировка по перегрузке снята";

    case BLOCK_MAINS_FAULT:
      return "Блокировка по неисправности сети снята";

    case BLOCK_SENSOR_FAULT:
      return "Блокировка по неисправности датчика снята";

    case BLOCK_RESERVED_5:
      return "Снята резервная блокировка 5";

    case BLOCK_RESERVED_6:
      return "Снята резервная блокировка 6";

    case BLOCK_RESERVED_7:
      return "Снята резервная блокировка 7";

    default:
      return "Причина запрета снята";
  }
}

// Пишем журнал только при изменении маски.
// Отдельная запись — для каждой добавленной или снятой причины.
// Общие BLOCK и ALLOW выводятся только при переходах 0 -> не 0 и не 0 -> 0.
static void logBlockFlagsChanges() {
  const uint8_t currentFlags = generatorBlockFlags;

  if (currentFlags == lastLoggedBlockFlags) {
    return;
  }

  const uint8_t addedFlags =
      currentFlags & static_cast<uint8_t>(~lastLoggedBlockFlags);

  const uint8_t removedFlags =
      lastLoggedBlockFlags & static_cast<uint8_t>(~currentFlags);

  for (uint8_t bit = 0; bit < 8; ++bit) {
    const uint8_t bitMask = static_cast<uint8_t>(1U << bit);

    if (addedFlags & bitMask) {
      char message[128];

      snprintf(
          message,
          sizeof(message),
          "flags=0x%02X, %s",
          currentFlags,
          blockReasonText(bitMask));

      logEvent("BLOCK", message);
    }
  }

  if (lastLoggedBlockFlags == 0 && currentFlags != 0) {
    char message[64];

    snprintf(
        message,
        sizeof(message),
        "flags=0x%02X, Генерация запрещена",
        currentFlags);

    logEvent("BLOCK", message);
  }

  for (uint8_t bit = 0; bit < 8; ++bit) {
    const uint8_t bitMask = static_cast<uint8_t>(1U << bit);

    if (removedFlags & bitMask) {
      char message[128];

      snprintf(
          message,
          sizeof(message),
          "flags=0x%02X, %s",
          currentFlags,
          blockClearedText(bitMask));

      logEvent("CLEAR", message);
    }
  }

  if (lastLoggedBlockFlags != 0 && currentFlags == 0) {
    logEvent(
        "ALLOW",
        "flags=0x00, Все запреты генерации сняты");
  }

  lastLoggedBlockFlags = currentFlags;
}

// ======================================================
// РАСЧЁТ ВРЕМЕННЫХ ИНТЕРВАЛОВ ДЛЯ PIO
// ======================================================

// Для задержки между двумя SET PINS:
// SET + MOV X + (X + 1) команд JMP X-- + следующий SET.
// Поэтому X = длительность_в_тактах - 3.
//
// В межполупериодной задержке есть дополнительный NOP.
// На границе полного периода вместо NOP выполняется WAIT.
// Эти две инструкции компенсируют друг друга.
static bool calculatePioDelayCounts(
    uint32_t &activeLoopCount,
    uint32_t &driverLoopCount,
    uint32_t &interphaseLoopCount) {

  if (UPS_FREQUENCY_HZ == 0 ||
      UPS_DEADTIME_PERCENT == 0 ||
      UPS_DEADTIME_PERCENT >= 100 ||
      UPS_DRIVER_DEADTIME_US == 0) {
    return false;
  }

  const uint32_t halfPeriodDenominator = 2UL * UPS_FREQUENCY_HZ;

  if ((PIO_CLOCK_HZ % halfPeriodDenominator) != 0) {
    return false;
  }

  const uint32_t halfPeriodTicks =
      PIO_CLOCK_HZ / halfPeriodDenominator;

  const uint32_t deadTimeTicks =
      (halfPeriodTicks * UPS_DEADTIME_PERCENT) / 100UL;

  const uint32_t activeTicks =
      halfPeriodTicks - deadTimeTicks;

  const uint64_t driverTicksNumerator =
      static_cast<uint64_t>(PIO_CLOCK_HZ) * UPS_DRIVER_DEADTIME_US;

  const uint32_t driverDeadTimeTicks =
      static_cast<uint32_t>(
          (driverTicksNumerator + 999999ULL) / 1000000ULL);

  if (deadTimeTicks <= (2UL * driverDeadTimeTicks)) {
    return false;
  }

  const uint32_t interphaseTicks =
      deadTimeTicks - (2UL * driverDeadTimeTicks);

  if (activeTicks <= 3UL ||
      driverDeadTimeTicks <= 3UL ||
      interphaseTicks <= 4UL) {
    return false;
  }

  activeLoopCount = activeTicks - 3UL;
  driverLoopCount = driverDeadTimeTicks - 3UL;
  interphaseLoopCount = interphaseTicks - 4UL;

  return true;
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ PIO-ГЕНЕРАТОРА
// ======================================================

static bool initializePioGenerator() {
  if (!pio_can_add_program_at_offset(
          pio0,
          &ups_driver_program,
          0)) {
    return false;
  }

  const uint32_t systemClockHz = clock_get_hz(clk_sys);

  if ((systemClockHz % PIO_CLOCK_HZ) != 0) {
    return false;
  }

  const uint32_t clockDivider = systemClockHz / PIO_CLOCK_HZ;

  if (clockDivider == 0 || clockDivider > 65535UL) {
    return false;
  }

  uint32_t activeLoopCount = 0;
  uint32_t driverLoopCount = 0;
  uint32_t interphaseLoopCount = 0;

  if (!calculatePioDelayCounts(
          activeLoopCount,
          driverLoopCount,
          interphaseLoopCount)) {
    return false;
  }

  const int sm = pio_claim_unused_sm(pio0, false);
  if (sm < 0) {
    return false;
  }
  generatorPioSm = sm;

  const int programOffset =
      pio_add_program_at_offset(pio0, &ups_driver_program, 0);

  if (programOffset != 0) {
    return false;
  }

  pio_sm_config config = pio_get_default_sm_config();

  sm_config_set_set_pins(
      &config,
      PIN_DRIVER_A_H,
      4);

  sm_config_set_wrap(
      &config,
      UPS_DRIVER_PIO_WRAP_TARGET,
      UPS_DRIVER_PIO_WRAP);

  sm_config_set_clkdiv_int_frac8(
      &config,
      clockDivider,
      0);

  pio_sm_init(pio0, static_cast<uint>(sm), 0, &config);

  for (uint8_t pin = PIN_DRIVER_A_H; pin <= PIN_DRIVER_B_L; ++pin) {
    pio_gpio_init(pio0, pin);
  }

  if (pio_sm_set_consecutive_pindirs(
          pio0,
          static_cast<uint>(sm),
          PIN_DRIVER_A_H,
          4,
          true) != 0) {
    return false;
  }

  // Безопасное рабочее состояние: включены оба нижних плеча.
  pio_sm_set_pins_with_mask(
      pio0,
      static_cast<uint>(sm),
      0b1010,
      0x0F);

  // TX FIFO:
  // Y   — активный импульс 9 мс;
  // ISR — защитная пауза 50 мкс;
  // OSR — межполупериодное ожидание 900 мкс.
  pio_sm_put_blocking(
      pio0,
      static_cast<uint>(sm),
      activeLoopCount);

  pio_sm_put_blocking(
      pio0,
      static_cast<uint>(sm),
      driverLoopCount);

  pio_sm_put_blocking(
      pio0,
      static_cast<uint>(sm),
      interphaseLoopCount);

  // До получения разрешения PIO остаётся в NORMAL.
  pio_interrupt_clear(pio0, PIO_IRQ_STOP_REQUEST);
  pio0->irq_force = (1u << PIO_IRQ_STOP_REQUEST);
  pioRunPermissionApplied = false;

  pio_sm_set_enabled(pio0, static_cast<uint>(sm), true);

  return true;
}

// ======================================================
// КОНТРОЛЬ RUN
// ======================================================

static void updateRunStatus(uint32_t now) {
  // Только эта функция читает физический вход GP4/RUN.
  const bool runIsHigh = (digitalRead(PIN_RUN) == HIGH);

  const bool oldMainsConfirmed = mainsPresentConfirmed;
  const bool oldGeneratorRequest = generatorRunRequest;

  if (runIsHigh) {
    runLowTiming = false;
    mainsPresentConfirmed = false;
    generatorRunRequest = true;
  } else if (!runLowTiming) {
    runLowTiming = true;
    runLowSince = now;
  } else if (now - runLowSince >= UPS_RUN_OFF_DELAY_MS) {
    mainsPresentConfirmed = true;
    generatorRunRequest = false;
  }

  if (!oldMainsConfirmed && mainsPresentConfirmed) {
    logEvent("RUN", "Сеть подтверждена");
  } else if (oldMainsConfirmed && !mainsPresentConfirmed) {
    logEvent("RUN", "Сеть пропала");
  }

  if (!oldGeneratorRequest && generatorRunRequest) {
    logEvent("RUN", "Запрошена генерация");
  }
}

// ======================================================
// ПЕРЕДАЧА РАЗРЕШЕНИЯ ГЕНЕРАТОРУ PIO
// ======================================================

// PIO проверяет STOP только на границе полного периода A+B.
// Генерация разрешена, только если RUN запрашивает её и маска запретов = 0.
static void applyGeneratorPermission() {
  const bool permission =
      generatorRunRequest && (generatorBlockFlags == 0);

  if (permission == pioRunPermissionApplied) {
    return;
  }

  if (permission) {
    pio_interrupt_clear(pio0, PIO_IRQ_STOP_REQUEST);
  } else {
    pio0->irq_force = (1u << PIO_IRQ_STOP_REQUEST);
  }

  pioRunPermissionApplied = permission;
}

// Обновляем глобальный статус по реальному положению PIO State Machine.
// PC=wrap_target — точка границы полного периода A+B перед следующим циклом.
// При разрешённой генерации кратковременное прохождение этой точки не сбрасывает
// флаг. При снятии разрешения ждём возврата PIO в эту точку после полного периода.
static void updateGeneratorRunningStatus() {
  if (generatorPioSm < 0) {
    generatorIsRunning = false;
    return;
  }

  const uint8_t currentPc =
      pio_sm_get_pc(pio0, static_cast<uint>(generatorPioSm));
  const bool atPeriodBoundary =
      currentPc == static_cast<uint8_t>(UPS_DRIVER_PIO_WRAP_TARGET);

  if (pioRunPermissionApplied) {
    if (!atPeriodBoundary) {
      generatorIsRunning = true;
    }
  } else if (atPeriodBoundary) {
    generatorIsRunning = false;
  }
}

// ======================================================
// ИЗМЕРЕНИЕ НАПРЯЖЕНИЯ АККУМУЛЯТОРА
// ======================================================

static void measureBatteryVoltage() {
  uint32_t adcSum = 0;

  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; ++i) {
    adcSum += static_cast<uint32_t>(analogRead(PIN_BATTERY));
  }

  const uint16_t adcAverage =
      static_cast<uint16_t>(
          (adcSum + (BATTERY_ADC_SAMPLES / 2)) / BATTERY_ADC_SAMPLES);

  const uint32_t dividerRatioX100 =
      ((BATTERY_DIVIDER_TOP_OHM + BATTERY_DIVIDER_BOTTOM_OHM) * 100UL) /
      BATTERY_DIVIDER_BOTTOM_OHM;

  // Результат в сотых долях вольта.
  // uint64_t нужен, чтобы промежуточное произведение не переполнилось.
  const uint64_t numerator =
      static_cast<uint64_t>(adcAverage) *
      BATTERY_ADC_REFERENCE_MV *
      dividerRatioX100;

  batteryVoltageCentiV =
      static_cast<uint16_t>(
          numerator /
          (static_cast<uint64_t>(BATTERY_ADC_MAX_COUNTS) * 1000ULL));

  // Нулевые пороги временно отключают батарейную отсечку.
  if (BATTERY_GENERATOR_STOP_CV == 0 &&
      BATTERY_GENERATOR_RECOVERY_CV == 0) {
    generatorBlockFlags &=
        static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
    return;
  }

  // Неправильно заданные пороги запрещают генерацию.
  if (BATTERY_GENERATOR_STOP_CV <= 0 ||
      BATTERY_GENERATOR_RECOVERY_CV <= BATTERY_GENERATOR_STOP_CV) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
    return;
  }

  // Низкое напряжение защёлкивает запрет.
  // Самостоятельный подъём напряжения после снятия нагрузки не снимает его.
  if (batteryVoltageCentiV <= BATTERY_GENERATOR_STOP_CV) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
  }
}

// Снять батарейную защёлку можно только при подтверждённой сети
// и достижении порога восстановления.
static void checkBatteryRecovery() {
  if (!mainsPresentConfirmed ||
      (generatorBlockFlags & BLOCK_BATTERY_LOW) == 0 ||
      BATTERY_GENERATOR_STOP_CV <= 0 ||
      BATTERY_GENERATOR_RECOVERY_CV <= BATTERY_GENERATOR_STOP_CV ||
      batteryVoltageCentiV < BATTERY_GENERATOR_RECOVERY_CV) {
    return;
  }

  logEvent(
      "RECOVERY",
      "Напряжение аккумулятора восстановлено");

  generatorBlockFlags &=
      static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ
// ======================================================

void setup() {
  // USB CDC Serial. Значение 9600 оставлено для совместимости
  // с терминалами, которые требуют указать скорость COM-порта.
  Serial.begin(9600);

  // Вход RUN без внутренней подтяжки — как в исходной версии.
  pinMode(PIN_RUN, INPUT);

  // АЦП RP2040: 12 бит, диапазон 0..4095.
  analogReadResolution(12);
  pinMode(PIN_BATTERY, INPUT);

  // До передачи GP0–GP3 автомату PIO все четыре выхода LOW.
  pinMode(PIN_DRIVER_A_H, OUTPUT);
  pinMode(PIN_DRIVER_A_L, OUTPUT);
  pinMode(PIN_DRIVER_B_H, OUTPUT);
  pinMode(PIN_DRIVER_B_L, OUTPUT);

  digitalWrite(PIN_DRIVER_A_H, LOW);
  digitalWrite(PIN_DRIVER_A_L, LOW);
  digitalWrite(PIN_DRIVER_B_H, LOW);
  digitalWrite(PIN_DRIVER_B_L, LOW);

  // I2C0 и OLED. DS1307 в будущем подключится к той же шине.
  initializeLogDisplay();

  logEvent("BOOT", "ArduinoUPS RP2040 запущен");

  if (!oledReady) {
    logEvent("OLED", "Дисплей SSD1306 не обнаружен");
  }

  if (!initializePioGenerator()) {
    logEvent("ERROR", "Не удалось запустить автомат PIO");
    updateLogDisplay();
    holdAllDriverOutputsLow();
  }

  // Первые отсчёты отбрасываем после настройки АЦП.
  for (uint8_t i = 0; i < 4; ++i) {
    analogRead(PIN_BATTERY);
  }

  // Стартовая выдержка из исходной четырёхвыходной версии.
  delay(UPS_STARTUP_DELAY_MS);

  // Встроенный WS2812 на GP16 использует отдельный PIO1.
  // Ошибка RGB-индикатора не должна останавливать генератор.
  if (!initializeStatusLed()) {
    logEvent("LED", "Не удалось инициализировать RGB-индикатор");
  }

  // Первое измерение АКБ выполняется до разрешения генерации.
  measureBatteryVoltage();
  lastBatteryMeasureMs = millis();

  updateRunStatus(lastBatteryMeasureMs);
  checkBatteryRecovery();
  logBlockFlagsChanges();
  applyGeneratorPermission();
  updateGeneratorRunningStatus();
  updateStatusLed();
  updateLogDisplay();
}

// ======================================================
// ОСНОВНОЙ ЦИКЛ
// ======================================================

void loop() {
  const uint32_t now = millis();

  updateRunStatus(now);

  // Измерение АКБ раз в секунду. PIO продолжает генерацию автономно.
  if (now - lastBatteryMeasureMs >= BATTERY_MEASURE_INTERVAL_MS) {
    lastBatteryMeasureMs = now;
    measureBatteryVoltage();
  }

  checkBatteryRecovery();

  // Сначала регистрируем изменение запретов, затем передаём PIO
  // актуальное разрешение на следующий полный период.
  logBlockFlagsChanges();
  applyGeneratorPermission();
  updateGeneratorRunningStatus();

  // RGB-индикатор отображает блокировки и фактическую работу генератора.
  updateStatusLed();

  // OLED обновляется только после появления новых сообщений.
  updateLogDisplay();
}
