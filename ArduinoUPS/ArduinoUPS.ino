/*
  ArduinoUPS — испытательный генератор и контроль АКБ
  Плата: Arduino Nano, ATmega328P, 16 МГц.

  ВАЖНО ДЛЯ СБОРКИ:
    GyverCore -> Tools -> System timer -> Disable.
    В этой конфигурации GyverCore не создаёт периодическое прерывание
    TIMER0_OVF и не предоставляет millis()/micros().
    Проект использует собственную временную базу от Timer1.

  RUN:
    A0 / PC0 — HIGH: генерация запрошена, LOW: сеть появилась.
    Физический вход RUN читает только updateRunStatus().

  Четыре управляющих выхода — один порт PORTD:
    D4 / PD4 — DRIVER_A_H: верхнее плечо канала A.
    D5 / PD5 — DRIVER_A_L: нижнее плечо канала A.
    D6 / PD6 — DRIVER_B_H: верхнее плечо канала B.
    D7 / PD7 — DRIVER_B_L: нижнее плечо канала B.

  Кнопка энкодера переносится с D4 на A2.
  D2/D3 остаются каналами энкодера A/B.
  D8 / PB0 — CHARGE_RELAY, пока всегда LOW.
  ADC6 — измерение напряжения аккумулятора.

  На каждом входе затвора управляющего 2N7002 обязательна подтяжка 10 кОм
  к GND. У верхнего BSS84 нужна подтяжка затвор–исток к +8 В.
  У каждого силового IRFZ44N нужен собственный резистор затвор–исток.

  Важно: код управляет только логическими входами драйверов. До подключения
  силовой части проверить все четыре сигнала осциллографом на низковольтном
  макете.
*/

#if !defined(_GYVERCORE_NOMILLIS)
#error "Select GyverCore: Tools -> System timer -> Disable"
#endif

#if !defined(_GYVERCORE_NOINIT)
#error "Select GyverCore: Tools -> Initialization -> Disable"
#endif

#if F_CPU != 16000000UL
#error "ArduinoUPS timing is configured for ATmega328P at 16 MHz"
#endif

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// ======================================================
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
// ======================================================

// Настройки генератора
#define UPS_FREQUENCY_HZ                    50  // Частота выходного сигнала, Гц
#define UPS_DEADTIME_PERCENT                10  // Общее межполупериодное окно, % от полупериода
#define UPS_DRIVER_DEADTIME_US              50UL // Защитная пауза внутри драйвера, мкс
#define UPS_STARTUP_DELAY_MS              1000  // Минимальная выдержка перед запуском, мс
#define UPS_RUN_OFF_DELAY_MS              5000  // RUN=LOW для подтверждения сети, мс

// Настройки измерения аккумулятора
#define BATTERY_MEASURE_INTERVAL_MS       1000UL // Интервал между полными измерениями, мс
#define BATTERY_ADC_SAMPLE_INTERVAL_MS       2UL // Пауза между отдельными отсчётами АЦП, мс
#define BATTERY_ADC_SAMPLES                 16  // Количество измерений для усреднения
#define BATTERY_ADC_DUMMY_SAMPLES            4  // Отбрасываемые первые отсчёты после выбора опоры
#define BATTERY_ADC_REFERENCE_MV          1100UL // Номинальная опора INTERNAL, мВ; уточнить калибровкой
#define BATTERY_DIVIDER_TOP_OHM         330000UL // Верхний резистор делителя: АКБ+ -> A6, Ом
#define BATTERY_DIVIDER_BOTTOM_OHM       10000UL // Нижний резистор делителя: A6 -> GND, Ом

// Пороговые напряжения АКБ — позднее будут настраиваться из меню
#define BATTERY_GENERATOR_STOP_CV             0 // Запрет генерации ниже этого напряжения, сотые В; 0 = отсечка отключена
#define BATTERY_GENERATOR_RECOVERY_CV         0 // Снятие запрета выше этого напряжения, сотые В; 0 = отсечка отключена
#define BATTERY_CHARGE_ON_CV                  0 // Включить заряд ниже этого напряжения, сотые В; пока не используется
#define BATTERY_CHARGE_OFF_CV                 0 // Выключить заряд выше этого напряжения, сотые В; пока не используется

// Коды причин запрета генерации. Каждому модулю принадлежит свой бит.
#define BLOCK_BATTERY_LOW                   0x01 // Бит 0: низкое напряжение аккумулятора
#define BLOCK_OVERHEAT                      0x02 // Бит 1: перегрев
#define BLOCK_OVERLOAD                      0x04 // Бит 2: перегрузка
#define BLOCK_MAINS_FAULT                   0x08 // Бит 3: неисправность/недопустимое состояние сети
#define BLOCK_SENSOR_FAULT                  0x10 // Бит 4: неисправность датчика
#define BLOCK_RESERVED_5                   0x20 // Бит 5: резерв
#define BLOCK_RESERVED_6                   0x40 // Бит 6: резерв
#define BLOCK_RESERVED_7                   0x80 // Бит 7: резерв

// ======================================================
// ПИНЫ И ТАБЛИЦА ВЫХОДНЫХ СОСТОЯНИЙ
// ======================================================

#define RUN_MASK             _BV(PC0) // A0: вход датчика сети / RUN
#define CHARGE_RELAY_MASK    _BV(PB0) // D8: реле зарядки

// Четыре выхода занимают PD4–PD7.
// Код фоновых задач не должен изменять PORTD во время работы генератора.
#define DRIVER_OUTPUT_MASK   0xF0
#define DRIVER_A_H_MASK      _BV(PD4)
#define DRIVER_A_L_MASK      _BV(PD5)
#define DRIVER_B_H_MASK      _BV(PD6)
#define DRIVER_B_L_MASK      _BV(PD7)
#define DRIVER_A_MASK        (DRIVER_A_H_MASK | DRIVER_A_L_MASK)
#define DRIVER_B_MASK        (DRIVER_B_H_MASK | DRIVER_B_L_MASK)

// Значения старшей тетрады PORTD в порядке PD7…PD4.
#define PATTERN_NORMAL_VALUE       0xA0 // 1010: оба нижних плеча включены
#define PATTERN_A_PREPARE_VALUE    0x80 // 1000: нижнее A выключено
#define PATTERN_A_ACTIVE_VALUE     0x90 // 1001: верхнее A включено
#define PATTERN_B_PREPARE_VALUE    0x20 // 0010: нижнее B выключено
#define PATTERN_B_ACTIVE_VALUE     0x60 // 0110: верхнее B включено
#define PATTERN_ALL_OFF_VALUE      0x00 // 0000: все четыре управляющих входа выключены

#define DRIVER_PATTERN_SAFE(value) \
  ((((value) & DRIVER_A_MASK) != DRIVER_A_MASK) && \
   (((value) & DRIVER_B_MASK) != DRIVER_B_MASK))

static_assert(DRIVER_PATTERN_SAFE(PATTERN_NORMAL_VALUE), "Unsafe normal driver pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_A_PREPARE_VALUE), "Unsafe A prepare pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_A_ACTIVE_VALUE), "Unsafe A active pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_B_PREPARE_VALUE), "Unsafe B prepare pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_B_ACTIVE_VALUE), "Unsafe B active pattern");
static_assert(DRIVER_PATTERN_SAFE(PATTERN_ALL_OFF_VALUE), "Unsafe all-off pattern");

enum DriverPattern : uint8_t {
  DRIVER_PATTERN_NORMAL,
  DRIVER_PATTERN_A_PREPARE,
  DRIVER_PATTERN_A_ACTIVE,
  DRIVER_PATTERN_B_PREPARE,
  DRIVER_PATTERN_B_ACTIVE,
  DRIVER_PATTERN_ALL_OFF
};

// Таблица хранится во Flash. Каждая строка — один байт для PORTD.
const uint8_t driverPortPatterns[] PROGMEM = {
  PATTERN_NORMAL_VALUE,
  PATTERN_A_PREPARE_VALUE,
  PATTERN_A_ACTIVE_VALUE,
  PATTERN_B_PREPARE_VALUE,
  PATTERN_B_ACTIVE_VALUE,
  PATTERN_ALL_OFF_VALUE
};

static inline void writeDriverPattern(DriverPattern pattern) {
  const uint8_t value =
      pgm_read_byte(&driverPortPatterns[static_cast<uint8_t>(pattern)]);

  // Одной записью меняем PD4–PD7; PD0–PD3 сохраняются.
  PORTD = static_cast<uint8_t>(
      (PORTD & static_cast<uint8_t>(~DRIVER_OUTPUT_MASK)) |
      (value & DRIVER_OUTPUT_MASK));
}

// ======================================================
// ПРОГРАММНЫЕ ФЛАГИ
// ======================================================

// Эти флаги записывает только основной цикл одним байтом.
// Timer1 только читает их. Поэтому cli() для обмена не нужен.
#define RUN_FLAG_GENERATOR_REQUEST  0x01
#define RUN_FLAG_MAINS_CONFIRMED    0x02
#define RUN_FLAG_STARTUP_READY      0x04

volatile uint8_t runControlFlags = 0;

// Общий байт причин запрета. Его изменяют фоновые модули,
// Timer1 только читает на границе периода.
volatile uint8_t generatorBlockFlags = 0;

// ======================================================
// СОБСТВЕННОЕ СИСТЕМНОЕ ВРЕМЯ ОТ TIMER1
// ======================================================

// Timer1: Normal mode, предделитель 8.
// При 16 МГц один тик равен 0,5 мкс.
// Timer1 создаёт события в моменты переключения драйверов или на границе
// периода ожидания. Отдельного прерывания каждую миллисекунду нет.
//
// systemMillisBase и systemSubMsTicks описывают время последнего события
// сравнения. systemLastCompare содержит значение OCR1A этого события.
// Между событиями текущее время уточняется по свободно бегущему TCNT1.
//
// systemTimeSequence — однобайтовый seqlock. ISR увеличивает его до и после
// обновления многобайтовых данных. Основной цикл при несовпадении версии
// просто повторяет чтение на следующем проходе, не запрещая прерывания.

struct SystemTimeSnapshot {
  uint32_t milliseconds;
  uint16_t subMillisecondTicks; // 0..1999; один тик = 0,5 мкс
};

volatile uint8_t systemTimeSequence = 0;
volatile uint32_t systemMillisBase = 0;
volatile uint16_t systemSubMsTicks = 0;
volatile uint16_t systemLastCompare = 0;

static bool tryReadSystemTime(SystemTimeSnapshot &result) {
  const uint8_t versionBefore = systemTimeSequence;

  if (versionBefore & 0x01) {
    return false; // ISR сейчас обновляет многобайтовые данные
  }

  const uint32_t baseMillis = systemMillisBase;
  const uint16_t baseSubTicks = systemSubMsTicks;
  const uint16_t lastCompare = systemLastCompare;
  const uint16_t currentCounter = TCNT1;

  const uint8_t versionAfter = systemTimeSequence;

  if (versionBefore != versionAfter || (versionAfter & 0x01)) {
    return false; // За время чтения прошло прерывание Timer1
  }

  // Все интервалы до следующего compare короче полного оборота Timer1.
  const uint16_t elapsedTicks =
      static_cast<uint16_t>(currentCounter - lastCompare);

  const uint16_t totalSubTicks =
      static_cast<uint16_t>(baseSubTicks + elapsedTicks);

  const uint8_t extraMilliseconds =
      static_cast<uint8_t>(totalSubTicks / 2000U);

  result.milliseconds = baseMillis + extraMilliseconds;
  result.subMillisecondTicks =
      static_cast<uint16_t>(totalSubTicks -
                            static_cast<uint16_t>(extraMilliseconds * 2000U));

  return true;
}

static inline bool timeReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

// ======================================================
// ПАРАМЕТРЫ TIMER1 И АВТОМАТ ДРАЙВЕРОВ
// ======================================================

volatile uint16_t activeTicks = 0;
volatile uint16_t driverDeadTimeTicks = 0;
volatile uint16_t interphaseWaitTicks = 0;
volatile uint16_t periodTicks = 0;

// Интервал, который закончился текущим событием compare.
// Используется для продвижения собственной временной базы.
volatile uint16_t timer1NextEventTicks = 1;

bool generatorTimingValid = false;

enum GeneratorPhase : uint8_t {
  GEN_PERIOD_BOUNDARY,       // Единственное место проверки разрешения
  GEN_WAIT_A_ON,             // Защитная пауза перед верхним A
  GEN_A_ACTIVE,              // Верхнее A включено
  GEN_WAIT_A_LOWER,          // Защитная пауза перед нижним A
  GEN_WAIT_B_LOWER_OFF,      // Отключение нижнего B на границе полупериода
  GEN_WAIT_B_ON,             // Защитная пауза перед верхним B
  GEN_B_ACTIVE,              // Верхнее B включено
  GEN_WAIT_B_LOWER,          // Защитная пауза перед нижним B
  GEN_FAULT_RECOVERY         // Возврат из неизвестного состояния
};

volatile GeneratorPhase generatorPhase = GEN_PERIOD_BOUNDARY;

static inline void scheduleNextTimerEvent(uint16_t deltaTicks) {
  timer1NextEventTicks = deltaTicks;
  OCR1A = static_cast<uint16_t>(OCR1A + deltaTicks);
}

// ======================================================
// ЕДИНЫЙ АВТОМАТ ЧЕТЫРЁХ УПРАВЛЯЮЩИХ ВЫХОДОВ
// ======================================================

// Функция вызывается только из ISR Timer1.
// Разрешения читаются только в GEN_PERIOD_BOUNDARY.
// Внутри полного периода A+B проверки запретов не выполняются.
static void driverControlStep() {
  switch (generatorPhase) {
    case GEN_PERIOD_BOUNDARY: {
      const uint8_t runFlags = runControlFlags;
      const bool startupReady =
          (runFlags & RUN_FLAG_STARTUP_READY) != 0;
      const bool runRequested =
          (runFlags & RUN_FLAG_GENERATOR_REQUEST) != 0;

      if (startupReady &&
          runRequested &&
          generatorBlockFlags == 0) {
        // Начинаем полный период. Сначала выключаем нижнее плечо A.
        writeDriverPattern(DRIVER_PATTERN_A_PREPARE);
        scheduleNextTimerEvent(driverDeadTimeTicks);
        generatorPhase = GEN_WAIT_A_ON;
      } else {
        // Нормальное состояние сохраняется; повторная проверка через 20 мс.
        writeDriverPattern(DRIVER_PATTERN_NORMAL);
        scheduleNextTimerEvent(periodTicks);
      }
      break;
    }

    case GEN_WAIT_A_ON:
      writeDriverPattern(DRIVER_PATTERN_A_ACTIVE);
      scheduleNextTimerEvent(activeTicks);
      generatorPhase = GEN_A_ACTIVE;
      break;

    case GEN_A_ACTIVE:
      writeDriverPattern(DRIVER_PATTERN_A_PREPARE);
      scheduleNextTimerEvent(driverDeadTimeTicks);
      generatorPhase = GEN_WAIT_A_LOWER;
      break;

    case GEN_WAIT_A_LOWER:
      writeDriverPattern(DRIVER_PATTERN_NORMAL);
      scheduleNextTimerEvent(interphaseWaitTicks);
      generatorPhase = GEN_WAIT_B_LOWER_OFF;
      break;

    case GEN_WAIT_B_LOWER_OFF:
      writeDriverPattern(DRIVER_PATTERN_B_PREPARE);
      scheduleNextTimerEvent(driverDeadTimeTicks);
      generatorPhase = GEN_WAIT_B_ON;
      break;

    case GEN_WAIT_B_ON:
      writeDriverPattern(DRIVER_PATTERN_B_ACTIVE);
      scheduleNextTimerEvent(activeTicks);
      generatorPhase = GEN_B_ACTIVE;
      break;

    case GEN_B_ACTIVE:
      writeDriverPattern(DRIVER_PATTERN_B_PREPARE);
      scheduleNextTimerEvent(driverDeadTimeTicks);
      generatorPhase = GEN_WAIT_B_LOWER;
      break;

    case GEN_WAIT_B_LOWER:
      writeDriverPattern(DRIVER_PATTERN_NORMAL);
      scheduleNextTimerEvent(interphaseWaitTicks);
      generatorPhase = GEN_PERIOD_BOUNDARY;
      break;

    case GEN_FAULT_RECOVERY:
      // После ранее выполненного ALL_OFF возвращаем оба нижних плеча.
      writeDriverPattern(DRIVER_PATTERN_NORMAL);
      scheduleNextTimerEvent(periodTicks);
      generatorPhase = GEN_PERIOD_BOUNDARY;
      break;

    default:
      // Неизвестное состояние: все малые управляющие транзисторы выключить.
      writeDriverPattern(DRIVER_PATTERN_ALL_OFF);
      scheduleNextTimerEvent(driverDeadTimeTicks);
      generatorPhase = GEN_FAULT_RECOVERY;
      break;
  }
}

// ======================================================
// КОНТРОЛЬ RUN — ТОЛЬКО ОСНОВНОЙ ЦИКЛ
// ======================================================

bool runLowTiming = false;
uint32_t runLowSinceMs = 0;

static void updateRunStatus(uint32_t nowMs) {
  // Это единственное место во всём проекте, где читается физический RUN.
  const bool runIsHigh = (PINC & RUN_MASK) != 0;
  uint8_t newFlags = runControlFlags;

  if (runIsHigh) {
    runLowTiming = false;
    newFlags |= RUN_FLAG_GENERATOR_REQUEST;
    newFlags &= static_cast<uint8_t>(~RUN_FLAG_MAINS_CONFIRMED);
  } else {
    if (!runLowTiming) {
      runLowTiming = true;
      runLowSinceMs = nowMs;
    }

    // До подтверждения сети сохраняем прежний запрос генерации:
    // текущая пара A+B завершится без прерывания.
    if (static_cast<uint32_t>(nowMs - runLowSinceMs) >=
        UPS_RUN_OFF_DELAY_MS) {
      newFlags &= static_cast<uint8_t>(~RUN_FLAG_GENERATOR_REQUEST);
      newFlags |= RUN_FLAG_MAINS_CONFIRMED;
    }
  }

  // Одна атомарная запись восьмибитного набора флагов.
  runControlFlags = newFlags;
}

// ======================================================
// ИЗМЕРЕНИЕ АКБ — НЕБЛОКИРУЮЩАЯ ПОШАГОВАЯ ЗАДАЧА
// ======================================================

// АЦП настроен на внутреннюю опору 1,1 В и делитель /128.
// Одна задача запускает преобразование, другая проверяет его завершение.
// Основной цикл не ждёт окончания преобразования.

bool batteryAdcConversionActive = false;
bool batteryMeasurementActive = true;
bool batteryInitialMeasurementDone = false;

uint8_t batteryDummyRemaining = BATTERY_ADC_DUMMY_SAMPLES;
uint8_t batterySamplesCollected = 0;
uint32_t batteryAdcSum = 0;

uint32_t batteryNextAdcStartMs = 5; // Внутренняя опора стабилизируется 5 мс
uint32_t lastBatteryMeasureMs = 0;

uint16_t batteryVoltageCentiV = 0; // 1234 = 12,34 В

static void applyBatteryProtection() {
  if (BATTERY_GENERATOR_STOP_CV == 0 &&
      BATTERY_GENERATOR_RECOVERY_CV == 0) {
    generatorBlockFlags &= static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
    return;
  }

  if (BATTERY_GENERATOR_STOP_CV <= 0 ||
      BATTERY_GENERATOR_RECOVERY_CV <= BATTERY_GENERATOR_STOP_CV) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
    return;
  }

  if (batteryVoltageCentiV <= BATTERY_GENERATOR_STOP_CV) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
  }
}

static void finishBatteryMeasurement(uint32_t nowMs) {
  const uint16_t adcAverage =
      static_cast<uint16_t>(
          (batteryAdcSum + (BATTERY_ADC_SAMPLES / 2)) /
          BATTERY_ADC_SAMPLES);

  const uint32_t dividerRatioX100 =
      ((BATTERY_DIVIDER_TOP_OHM + BATTERY_DIVIDER_BOTTOM_OHM) * 100UL) /
      BATTERY_DIVIDER_BOTTOM_OHM;

  batteryVoltageCentiV =
      (static_cast<uint32_t>(adcAverage) * BATTERY_ADC_REFERENCE_MV *
       dividerRatioX100) / (1023UL * 1000UL);

  applyBatteryProtection();

  batteryInitialMeasurementDone = true;
  batteryMeasurementActive = false;
  lastBatteryMeasureMs = nowMs;
}

static void serviceBatteryMeasurement(uint32_t nowMs) {
  if (batteryAdcConversionActive) {
    // Преобразование ещё выполняется — не ждём, возвращаемся в loop().
    if (ADCSRA & _BV(ADSC)) {
      return;
    }

    const uint16_t sample = ADC;
    batteryAdcConversionActive = false;

    if (batteryDummyRemaining > 0) {
      --batteryDummyRemaining;
      batteryNextAdcStartMs =
          nowMs + BATTERY_ADC_SAMPLE_INTERVAL_MS;
      return;
    }

    batteryAdcSum += sample;
    ++batterySamplesCollected;

    if (batterySamplesCollected >= BATTERY_ADC_SAMPLES) {
      finishBatteryMeasurement(nowMs);
      return;
    }

    batteryNextAdcStartMs =
        nowMs + BATTERY_ADC_SAMPLE_INTERVAL_MS;
    return;
  }

  if (!batteryMeasurementActive) {
    if (!batteryInitialMeasurementDone ||
        static_cast<uint32_t>(nowMs - lastBatteryMeasureMs) <
            BATTERY_MEASURE_INTERVAL_MS) {
      return;
    }

    // Начинаем очередную серию из 16 отдельных преобразований.
    batteryMeasurementActive = true;
    batterySamplesCollected = 0;
    batteryAdcSum = 0;
    batteryNextAdcStartMs = nowMs;
  }

  if (batteryDummyRemaining > 0 ||
      batteryMeasurementActive) {
    if (!timeReached(nowMs, batteryNextAdcStartMs)) {
      return;
    }

    ADCSRA |= _BV(ADSC);
    batteryAdcConversionActive = true;
  }
}

// ======================================================
// СТАРТОВОЕ РАЗРЕШЕНИЕ И ВОССТАНОВЛЕНИЕ АКБ
// ======================================================

static void serviceStartupGate(uint32_t nowMs) {
  if (nowMs < UPS_STARTUP_DELAY_MS ||
      !batteryInitialMeasurementDone) {
    return;
  }

  // Основной цикл — единственный писатель runControlFlags.
  runControlFlags |= RUN_FLAG_STARTUP_READY;
}

static void serviceBatteryRecovery() {
  const bool mainsConfirmed =
      (runControlFlags & RUN_FLAG_MAINS_CONFIRMED) != 0;

  if (mainsConfirmed &&
      BATTERY_GENERATOR_STOP_CV > 0 &&
      BATTERY_GENERATOR_RECOVERY_CV > BATTERY_GENERATOR_STOP_CV &&
      batteryVoltageCentiV >= BATTERY_GENERATOR_RECOVERY_CV) {
    generatorBlockFlags &= static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
  }
}

// ======================================================
// КООПЕРАТИВНЫЙ ДИСПЕТЧЕР ФОНОВЫХ ЗАДАЧ
// ======================================================

// За один проход loop() выполняется только одна небольшая задача.
// Большие операции разбиваются на продолжения в следующих проходах.
// Timer1 может прервать любую фоновую задачу в любой момент.

enum BackgroundTask : uint8_t {
  TASK_RUN_STATUS,
  TASK_BATTERY_ADC,
  TASK_STARTUP_GATE,
  TASK_BATTERY_RECOVERY,
  TASK_COUNT
};

uint8_t nextBackgroundTask = 0;

void loop() {
  SystemTimeSnapshot now;

  // Если Timer1 обновлял системное время во время чтения, пробуем на следующем
  // проходе loop(). Никаких cli(), ожидания или повторного цикла здесь нет.
  if (!tryReadSystemTime(now)) {
    return;
  }

  switch (nextBackgroundTask) {
    case TASK_RUN_STATUS:
      updateRunStatus(now.milliseconds);
      break;

    case TASK_BATTERY_ADC:
      serviceBatteryMeasurement(now.milliseconds);
      break;

    case TASK_STARTUP_GATE:
      serviceStartupGate(now.milliseconds);
      break;

    case TASK_BATTERY_RECOVERY:
      serviceBatteryRecovery();
      break;

    default:
      nextBackgroundTask = 0;
      return;
  }

  ++nextBackgroundTask;
  if (nextBackgroundTask >= TASK_COUNT) {
    nextBackgroundTask = 0;
  }
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ
// ======================================================

void setup() {
  // Сначала все четыре управляющих выхода в LOW.
  // Это выполняется до назначения нормального состояния драйверов.
  PORTD &= static_cast<uint8_t>(~DRIVER_OUTPUT_MASK);
  DDRD |= DRIVER_OUTPUT_MASK;

  // Нормальное состояние: оба нижних плеча включены.
  writeDriverPattern(DRIVER_PATTERN_NORMAL);

  // D8 — выход реле зарядки; пока реле всегда выключено.
  PORTB &= static_cast<uint8_t>(~CHARGE_RELAY_MASK);
  DDRB |= CHARGE_RELAY_MASK;

  // A0 / PC0 — вход RUN, внутренняя подтяжка отключена.
  DDRC &= static_cast<uint8_t>(~RUN_MASK);
  PORTC &= static_cast<uint8_t>(~RUN_MASK);

  // GyverCore собрал системный тик Timer0 без обработчика из-за
  // _GYVERCORE_NOMILLIS. Останавливаем сам Timer0: он нам не нужен.
  TIMSK0 = 0;
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;

  // Полностью перенастраиваем Timer1 под собственное расписание.
  TIMSK1 = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  TIFR1 = _BV(ICF1) | _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);

  // АЦП: внутренняя опора 1,1 В, вход ADC6, частота АЦП 125 кГц.
  // Делитель /128 выбран для нормальной точности преобразования.
  ADMUX = _BV(REFS1) | _BV(REFS0) | 6;
  ADCSRB = 0;
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  // Рассчитываем временные параметры Timer1.
  const uint32_t timerClockHz = F_CPU / 8UL;
  uint32_t halfPeriodTicks = 0;
  uint32_t calculatedDeadTicks = 0;
  uint32_t calculatedActiveTicks = 0;
  uint32_t calculatedDriverDeadTicks = 0;
  uint32_t calculatedInterphaseTicks = 0;
  uint32_t calculatedPeriodTicks = 0;

  if (UPS_FREQUENCY_HZ > 0 &&
      UPS_DEADTIME_PERCENT > 0 &&
      UPS_DEADTIME_PERCENT < 100 &&
      UPS_DRIVER_DEADTIME_US > 0) {

    halfPeriodTicks = timerClockHz / (2UL * UPS_FREQUENCY_HZ);
    calculatedDeadTicks =
        (halfPeriodTicks * UPS_DEADTIME_PERCENT) / 100UL;
    calculatedActiveTicks = halfPeriodTicks - calculatedDeadTicks;

    // Округляем защитную паузу вверх, чтобы не получить меньше заданной.
    calculatedDriverDeadTicks =
        (timerClockHz * UPS_DRIVER_DEADTIME_US + 999999UL) / 1000000UL;

    calculatedPeriodTicks = 2UL * halfPeriodTicks;

    // В каждом общем dead time размещаются две защитные паузы драйвера.
    // Остаток — ожидание до точной границы следующего полупериода.
    if (calculatedDeadTicks > (2UL * calculatedDriverDeadTicks)) {
      calculatedInterphaseTicks =
          calculatedDeadTicks - (2UL * calculatedDriverDeadTicks);
    }

    if (halfPeriodTicks <= 65535UL &&
        calculatedPeriodTicks <= 65535UL &&
        calculatedActiveTicks > 0 &&
        calculatedActiveTicks <= 65535UL &&
        calculatedDriverDeadTicks > 0 &&
        calculatedDriverDeadTicks <= 65535UL &&
        calculatedInterphaseTicks > 0 &&
        calculatedInterphaseTicks <= 65535UL) {

      activeTicks = static_cast<uint16_t>(calculatedActiveTicks);
      driverDeadTimeTicks = static_cast<uint16_t>(calculatedDriverDeadTicks);
      interphaseWaitTicks = static_cast<uint16_t>(calculatedInterphaseTicks);
      periodTicks = static_cast<uint16_t>(calculatedPeriodTicks);
      generatorTimingValid = true;
    }
  }

  // Запуск Timer1. Системное время начинается с нуля.
  // Первое событие через один тик переводит автомат на границу периода.
  if (generatorTimingValid) {
    TCNT1 = 0;
    OCR1A = 1;
    timer1NextEventTicks = 1;

    systemTimeSequence = 0;
    systemMillisBase = 0;
    systemSubMsTicks = 0;
    systemLastCompare = 0;

    generatorPhase = GEN_PERIOD_BOUNDARY;

    // Timer1: Normal mode, предделитель 8.
    TIMSK1 = _BV(OCIE1A);
    TCCR1B = _BV(CS11);
  }

  // GyverCore init() отключён настройкой _GYVERCORE_NOINIT.
  // Глобальные прерывания разрешаем только после полной настройки GPIO,
  // АЦП и Timer1. В рабочем коде cli() не используется.
  sei();
}

// ======================================================
// ПРЕРЫВАНИЕ TIMER1
// ======================================================

ISR(TIMER1_COMPA_vect) {
  // Запоминаем интервал и значение compare, на котором произошло событие.
  const uint16_t elapsedTicks = timer1NextEventTicks;
  const uint16_t eventCompare = OCR1A;

  // Начало атомарного снимка времени. Прерывания вложенно не разрешаем.
  ++systemTimeSequence;

  // Сначала выполняем силовую операцию — как можно раньше после входа в ISR.
  driverControlStep();

  // После изменения PORTD обновляем системное время.
  // Все расчёты выполняются по целым тикам Timer1 (0,5 мкс).
  const uint16_t totalSubTicks =
      static_cast<uint16_t>(systemSubMsTicks + elapsedTicks);

  const uint8_t addedMilliseconds =
      static_cast<uint8_t>(totalSubTicks / 2000U);

  systemSubMsTicks =
      static_cast<uint16_t>(
          totalSubTicks -
          static_cast<uint16_t>(addedMilliseconds * 2000U));

  systemMillisBase += addedMilliseconds;
  systemLastCompare = eventCompare;

  // Конец обновления: основной цикл теперь может получить согласованный снимок.
  ++systemTimeSequence;
}
