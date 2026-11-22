/*
  ArduinoUPS — RP2040-Zero, четырёхвыходной PIO-генератор.

  Перенос ветки arduino-ups-four-output-driver:
    GP0 — DRIVER_A_H: верхнее плечо драйвера A.
    GP1 — DRIVER_A_L: нижнее плечо драйвера A.
    GP2 — DRIVER_B_H: верхнее плечо драйвера B.
    GP3 — DRIVER_B_L: нижнее плечо драйвера B.
    GP4 — RUN: HIGH = генерация нужна, LOW = сеть появилась.

  Пока перенесены только четыре выхода и RUN. АКБ, реле, дисплей,
  энкодер и остальные функции намеренно не включены.

  Все четыре выхода управляются одной инструкцией SET PINS автомата PIO.
  CPU не обслуживает отдельные фронты и не участвует в формировании
  временных интервалов.
*/

#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "driver_pio.h"

// ======================================================
// ВЫВОДЫ RP2040-ZERO
// ======================================================

// Четыре последовательных GPIO управляются одной SET PINS командой PIO.
constexpr uint8_t PIN_DRIVER_A_H = 0;  // GP0
constexpr uint8_t PIN_DRIVER_A_L = 1;  // GP1
constexpr uint8_t PIN_DRIVER_B_H = 2;  // GP2
constexpr uint8_t PIN_DRIVER_B_L = 3;  // GP3

// Единственный физический вход этой испытательной версии.
constexpr uint8_t PIN_RUN = 4;         // GP4

// ======================================================
// ПАРАМЕТРЫ ГЕНЕРАТОРА
// ======================================================

#define UPS_FREQUENCY_HZ                 50UL
#define UPS_DEADTIME_PERCENT              10UL
#define UPS_DRIVER_DEADTIME_US            50UL
#define UPS_STARTUP_DELAY_MS            1000UL
#define UPS_RUN_OFF_DELAY_MS            5000UL

// PIO работает на 2,5 МГц: один цикл = 0,4 мкс.
// При штатной частоте clk_sys = 125 МГц делитель равен ровно 50,
// то есть дробный делитель и его фазовая модуляция не используются.
constexpr uint32_t PIO_CLOCK_HZ = 2500000UL;

// PIO IRQ0 используется как внутренний флаг STOP.
// IRQ0 = 1: остановить на ближайшей границе полного периода.
// IRQ0 = 0: разрешить следующий полный период.
constexpr uint8_t PIO_IRQ_STOP_REQUEST = 0;

// Порядок битов SET PINS: GP3 GP2 GP1 GP0 = B_L B_H A_L A_H.
constexpr uint8_t PATTERN_NORMAL       = 0b1010; // A_L + B_L
constexpr uint8_t PATTERN_A_PREPARE    = 0b1000; // B_L
constexpr uint8_t PATTERN_A_ACTIVE     = 0b1001; // B_L + A_H
constexpr uint8_t PATTERN_B_PREPARE    = 0b0010; // A_L
constexpr uint8_t PATTERN_B_ACTIVE     = 0b0110; // A_L + B_H
constexpr uint8_t PATTERN_ALL_OFF      = 0b0000;

// Compile-time проверка: верхнее и нижнее плечи одного драйвера
// ни в одном используемом шаблоне не включаются одновременно.
constexpr uint8_t DRIVER_A_MASK = 0b0011;
constexpr uint8_t DRIVER_B_MASK = 0b1100;

#define DRIVER_PATTERN_SAFE(value) \
  ((((value) & DRIVER_A_MASK) != DRIVER_A_MASK) && \
   (((value) & DRIVER_B_MASK) != DRIVER_B_MASK))

static_assert(DRIVER_PATTERN_SAFE(PATTERN_NORMAL), "Unsafe NORMAL pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_A_PREPARE), "Unsafe A_PREPARE pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_A_ACTIVE), "Unsafe A_ACTIVE pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_B_PREPARE), "Unsafe B_PREPARE pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_B_ACTIVE), "Unsafe B_ACTIVE pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_ALL_OFF), "Unsafe ALL_OFF pattern");

// ======================================================
// СОСТОЯНИЕ RUN
// ======================================================

// Поведение перенесено из updateRunStatus() исходной четырёхвыходной ветки.
bool runLowTiming = false;
bool mainsPresentConfirmed = false;
bool generatorRunRequest = false;
uint32_t runLowSince = 0;

// Состояние внутреннего разрешения PIO, последнее переданное из loop().
bool pioRunPermissionApplied = false;

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
    // Ошибка конфигурации: не запускать силовые ключи.
  }
}

// ======================================================
// РАСЧЁТ ВРЕМЕННЫХ ИНТЕРВАЛОВ ДЛЯ PIO
// ======================================================

// Параметры передаются в PIO как значения счётчика X.
//
// Обычная задержка между двумя SET PINS:
//   SET PINS + MOV X, регистр + (X + 1) инструкций JMP X--
// Поэтому X = длительность_в_тактах - 3.
//
// Межполупериодная задержка использует дополнительный NOP в первой
// половине и WAIT 0 IRQ на границе периода во второй. Эти две инструкции
// компенсируют друг друга, поэтому обе межполупериодные паузы остаются
// одинаковыми и полный период составляет ровно 20 мс.
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

  // Округление вверх — защитная пауза не может быть короче заданной.
  const uint64_t driverTicksNumerator =
      static_cast<uint64_t>(PIO_CLOCK_HZ) * UPS_DRIVER_DEADTIME_US;

  const uint32_t driverDeadTimeTicks =
      static_cast<uint32_t>(
          (driverTicksNumerator + 999999ULL) / 1000000ULL);

  // В общем dead time две защитные паузы драйверов,
  // остаток — межполупериодное ожидание.
  if (deadTimeTicks <= (2UL * driverDeadTimeTicks)) {
    return false;
  }

  const uint32_t interphaseTicks =
      deadTimeTicks - (2UL * driverDeadTimeTicks);

  // Проверяем минимальные значения, необходимые для счётных циклов PIO.
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
  // PIO-программа занимает всю память инструкций PIO0 (32 слова)
  // и загружается с фиксированного адреса 0, на который рассчитаны JMP.
  if (!pio_can_add_program_at_offset(
          pio0, &ups_driver_program, 0)) {
    return false;
  }

  const uint32_t systemClockHz = clock_get_hz(clk_sys);

  // Требуем целый делитель. Для штатных 125 МГц это ровно 50,
  // следовательно PIO получает стабильные 2,5 МГц без дробного деления.
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

  const int programOffset =
      pio_add_program_at_offset(pio0, &ups_driver_program, 0);

  if (programOffset != 0) {
    return false;
  }

  pio_sm_config config = pio_get_default_sm_config();

  // SET PINS управляет GP0–GP3 одной четырёхбитной командой.
  sm_config_set_set_pins(
      &config,
      PIN_DRIVER_A_H,
      4);

  sm_config_set_wrap(
      &config,
      UPS_DRIVER_PIO_WRAP_TARGET,
      UPS_DRIVER_PIO_WRAP);

  // Только целый делитель частоты PIO.
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

  // До запуска программы сразу устанавливаем безопасное нормальное
  // состояние: оба нижних плеча включены, оба верхних выключены.
  pio_sm_set_pins_with_mask(
      pio0,
      static_cast<uint>(sm),
      PATTERN_NORMAL,
      0x0F);

  // PIO-программа загружает эти три значения в Y, ISR и OSR:
  // Y   — длительность активного импульса 9 мс;
  // ISR — защитная пауза драйвера 50 мкс;
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

  // Сначала устанавливаем STOP, чтобы PIO остался в NORMAL
  // и не начал генерацию до подтверждения RUN.
  pio_interrupt_clear(pio0, PIO_IRQ_STOP_REQUEST);
  pio0->irq_force = (1u << PIO_IRQ_STOP_REQUEST);
  pioRunPermissionApplied = false;

  pio_sm_set_enabled(pio0, static_cast<uint>(sm), true);

  return true;
}

// ======================================================
// КОНТРОЛЬ ВХОДА RUN
// ======================================================

void updateRunStatus(uint32_t now) {
  // Только эта функция читает физический вход RUN.
  const bool runIsHigh = (digitalRead(PIN_RUN) == HIGH);
  bool runLowConfirmed = false;

  if (runIsHigh) {
    runLowTiming = false;
  } else if (!runLowTiming) {
    runLowTiming = true;
    runLowSince = now;
  } else if (now - runLowSince >= UPS_RUN_OFF_DELAY_MS) {
    runLowConfirmed = true;
  }

  if (runIsHigh) {
    mainsPresentConfirmed = false;
    generatorRunRequest = true;
  } else if (runLowConfirmed) {
    mainsPresentConfirmed = true;
    generatorRunRequest = false;
  }
}

// ======================================================
// ПЕРЕДАЧА РАЗРЕШЕНИЯ ГЕНЕРАТОРУ PIO
// ======================================================

// PIO проверяет флаг только на границе полного периода.
// IRQ0 = 0 — следующий период разрешён.
// IRQ0 = 1 — текущий период завершается, затем PIO остаётся в NORMAL.
//
// WAIT 0 IRQ не очищает флаг при выполнении условия. Это позволяет
// использовать IRQ0 как устойчивый уровень разрешения/запрета.
static void applyGeneratorRunRequest() {
  if (generatorRunRequest == pioRunPermissionApplied) {
    return;
  }

  if (generatorRunRequest) {
    // Снять STOP: PIO сможет начать следующий период.
    pio_interrupt_clear(pio0, PIO_IRQ_STOP_REQUEST);
  } else {
    // Установить STOP: PIO остановится только после полного периода A+B.
    pio0->irq_force = (1u << PIO_IRQ_STOP_REQUEST);
  }

  pioRunPermissionApplied = generatorRunRequest;
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ И ОСНОВНОЙ ЦИКЛ
// ======================================================

void setup() {
  // До передачи управления PIO все четыре вывода принудительно LOW.
  pinMode(PIN_DRIVER_A_H, OUTPUT);
  pinMode(PIN_DRIVER_A_L, OUTPUT);
  pinMode(PIN_DRIVER_B_H, OUTPUT);
  pinMode(PIN_DRIVER_B_L, OUTPUT);

  digitalWrite(PIN_DRIVER_A_H, LOW);
  digitalWrite(PIN_DRIVER_A_L, LOW);
  digitalWrite(PIN_DRIVER_B_H, LOW);
  digitalWrite(PIN_DRIVER_B_L, LOW);

  // RUN — вход без внутренней подтяжки, как в исходной версии.
  pinMode(PIN_RUN, INPUT);

  if (!initializePioGenerator()) {
    holdAllDriverOutputsLow();
  }

  // Стартовая выдержка исходного проекта.
  delay(UPS_STARTUP_DELAY_MS);

  // Первое чтение RUN после стартовой выдержки.
  updateRunStatus(millis());
  applyGeneratorRunRequest();
}

void loop() {
  const uint32_t now = millis();

  updateRunStatus(now);
  applyGeneratorRunRequest();
}
