/*
  ArduinoUPS — испытательный генератор и контроль АКБ
  Плата: Arduino Nano, ATmega328P, 16 МГц.

  RUN:
    A0 / PC0 — HIGH: генерация запрошена, LOW: сеть появилась.
    Физический вход RUN читает только updateRunStatus().

  Управление двумя драйверами — четыре выхода одного порта PORTD:
    D4 / PD4 — DRIVER_A_H: верхнее плечо канала A.
    D5 / PD5 — DRIVER_A_L: нижнее плечо канала A.
    D6 / PD6 — DRIVER_B_H: верхнее плечо канала B.
    D7 / PD7 — DRIVER_B_L: нижнее плечо канала B.

  Кнопку энкодера переносим с D4 на A2, чтобы освободить PD4.
  D2/D3 остаются каналами энкодера A/B.
  D8 / PB0 — CHARGE_RELAY, пока всегда LOW.
  A6 / ADC6 — измерение напряжения аккумулятора.

  Для каждого управляющего входа 2N7002 обязательна внешняя подтяжка
  затвор–GND 10 кОм. На каждом IRFZ44N также нужен отдельный резистор
  затвор–исток, чтобы силовой MOSFET оставался закрытым при Hi-Z/RESET.

  У верхнего BSS84 обязателен резистор затвор–исток к его источнику +8 В:
  он гарантирует закрытие BSS84, когда управляющий 2N7002 выключен.
  На каждом IRFZ44N также нужен отдельный резистор затвор–исток, чтобы
  силовой MOSFET оставался закрытым при Hi-Z/RESET.

  Важно: код управляет только логическими входами драйверов. До подключения
  силовой части проверить все четыре сигнала и их защитные интервалы
  осциллографом на низковольтном макете.
*/

// ======================================================
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
// ======================================================

// Настройки генератора
#define UPS_FREQUENCY_HZ                    50  // Частота выходного сигнала, Гц
#define UPS_DEADTIME_PERCENT                10  // Общее межполупериодное окно, % от полупериода
#define UPS_DRIVER_DEADTIME_US              50UL // Пауза между плечами одного драйвера, мкс; начальное значение для проверки
#define UPS_STARTUP_DELAY_MS              1000  // Пауза после включения МК, мс
#define UPS_RUN_OFF_DELAY_MS              5000  // Время устойчивого RUN=LOW до подтверждения сети, мс
#define UPS_TIMER_STALL_TIMEOUT_MS          100  // Тайм-аут контроля продвижения автомата, мс

// Настройки измерения аккумулятора
#define BATTERY_MEASURE_INTERVAL_MS       1000  // Период измерения напряжения АКБ, мс
#define BATTERY_ADC_SAMPLES                 16  // Количество измерений АЦП для усреднения
#define BATTERY_ADC_REFERENCE_MV          1100UL // Опорное напряжение INTERNAL, мВ; уточнить калибровкой
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

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// ======================================================
// ПИНЫ И ТАБЛИЦА ВЫХОДНЫХ СОСТОЯНИЙ
// ======================================================

#define RUN_MASK          _BV(PC0)  // A0: вход датчика сети / RUN
#define BATTERY_PIN       A6        // A6: аналоговый вход измерения АКБ
#define CHARGE_RELAY_MASK _BV(PB0)  // D8: выход управления реле зарядки

// Все четыре управляющих выхода находятся на PD4–PD7.
// PD0–PD3 сохраняются при каждой записи в PORTD.
#define DRIVER_OUTPUT_MASK 0xF0
#define DRIVER_A_H_MASK    _BV(PD4)
#define DRIVER_A_L_MASK    _BV(PD5)
#define DRIVER_B_H_MASK    _BV(PD6)
#define DRIVER_B_L_MASK    _BV(PD7)
#define DRIVER_A_MASK      (DRIVER_A_H_MASK | DRIVER_A_L_MASK)
#define DRIVER_B_MASK      (DRIVER_B_H_MASK | DRIVER_B_L_MASK)

// Значения именно старшей тетрады PORTD (PD7..PD4).
// Нормальное состояние: A_L и B_L включены, A_H и B_H выключены.
#define PATTERN_NORMAL_VALUE       0xA0
#define PATTERN_A_PREPARE_VALUE    0x80
#define PATTERN_A_ACTIVE_VALUE     0x90
#define PATTERN_B_PREPARE_VALUE    0x20
#define PATTERN_B_ACTIVE_VALUE     0x60
#define PATTERN_ALL_OFF_VALUE      0x00

// Проверка таблицы на невозможность включить оба плеча одного драйвера.
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

// Таблица хранится во Flash. Каждая запись — один байт для PORTD.
const uint8_t driverPortPatterns[] PROGMEM = {
  PATTERN_NORMAL_VALUE,       // 0: 1010 — A_L + B_L, нормальное состояние
  PATTERN_A_PREPARE_VALUE,    // 1: 1000 — A_L выключено, B_L включено
  PATTERN_A_ACTIVE_VALUE,     // 2: 1001 — A_H + B_L
  PATTERN_B_PREPARE_VALUE,    // 3: 0010 — A_L включено, B_L выключено
  PATTERN_B_ACTIVE_VALUE,     // 4: 0110 — A_L + B_H
  PATTERN_ALL_OFF_VALUE       // 5: 0000 — все управляющие 2N7002 выключены
};

static inline void writeDriverPattern(DriverPattern pattern) {
  const uint8_t value =
      pgm_read_byte(&driverPortPatterns[static_cast<uint8_t>(pattern)]);

  // Одной записью меняем PD4–PD7; PD0–PD3 не затрагиваем.
  PORTD = static_cast<uint8_t>(
      (PORTD & static_cast<uint8_t>(~DRIVER_OUTPUT_MASK)) |
      (value & DRIVER_OUTPUT_MASK));
}

// ======================================================
// ВРЕМЕННЫЕ ПАРАМЕТРЫ И СОСТОЯНИЯ АВТОМАТА
// ======================================================

// Timer1: Normal mode, предделитель 8.
// При F_CPU=16 МГц частота таймера 2 МГц, один тик = 0,5 мкс.
volatile uint16_t activeTicks = 0;
volatile uint16_t driverDeadTimeTicks = 0;
volatile uint16_t interphaseWaitTicks = 0;
volatile uint16_t periodTicks = 0;

bool generatorTimingValid = false;
volatile bool generatorRunning = false;

// Общий байт причин запрета: 0 — запретов нет, ненулевое значение запрещает
// запуск следующего периода. Каждый модуль меняет только собственный бит.
volatile uint8_t generatorBlockFlags = 0;

// RUN читает только updateRunStatus().
// generatorRunRequest — текущий RUN=HIGH.
// mainsPresentConfirmed — RUN=LOW непрерывно 5 секунд.
volatile bool generatorRunRequest = false;
volatile bool mainsPresentConfirmed = false;
bool runLowTiming = false;
uint32_t runLowSince = 0;

// Контроль того, что Timer1 продолжает выдавать события.
volatile uint8_t timer1Heartbeat = 0;
uint8_t lastTimer1Heartbeat = 0;
uint32_t lastTimer1HeartbeatMs = 0;

// Последнее измеренное напряжение аккумулятора и время последнего измерения.
uint16_t batteryVoltageCentiV = 0; // 1234 = 12,34 В
uint32_t lastBatteryMeasureMs = 0;

enum GeneratorPhase : uint8_t {
  GEN_PERIOD_BOUNDARY,       // Единственное место проверки разрешения периода
  GEN_WAIT_A_ON,             // Ожидание защитного интервала перед верхним A
  GEN_A_ACTIVE,              // Импульс верхнего плеча A завершится по таймеру
  GEN_WAIT_A_LOWER,          // Ожидание перед возвратом A в нормальное состояние
  GEN_WAIT_B_LOWER_OFF,      // Ожидание границы начала полупериода B
  GEN_WAIT_B_ON,             // Ожидание защитного интервала перед верхним B
  GEN_B_ACTIVE,              // Импульс верхнего плеча B завершится по таймеру
  GEN_WAIT_B_LOWER,          // Ожидание перед возвратом B в нормальное состояние
  GEN_FAULT_RECOVERY         // Возврат из ошибочного состояния в нормальное
};

volatile GeneratorPhase generatorPhase = GEN_PERIOD_BOUNDARY;

// ======================================================
// ЕДИНЫЙ МЕХАНИЗМ УПРАВЛЕНИЯ ЧЕТЫРЬМЯ ВЫХОДАМИ
// ======================================================

// Эта функция вызывается только из ISR Timer1.
// Проверка generatorBlockFlags и RUN-переменных выполняется только
// в GEN_PERIOD_BOUNDARY — один раз на границе полного периода 20 мс.
static void driverControlStep() {
  switch (generatorPhase) {
    case GEN_PERIOD_BOUNDARY: {
      const bool previousPeriodMayContinue =
          generatorRunning && !mainsPresentConfirmed;
      const bool newPeriodRequested = generatorRunRequest;

      if (generatorBlockFlags == 0 &&
          (newPeriodRequested || previousPeriodMayContinue)) {
        generatorRunning = true;

        // Из нормального состояния выключаем только нижнее плечо A.
        writeDriverPattern(DRIVER_PATTERN_A_PREPARE);
        OCR1A += driverDeadTimeTicks;
        generatorPhase = GEN_WAIT_A_ON;
      } else {
        // Остаёмся в нормальном состоянии и проверяем снова через 20 мс.
        generatorRunning = false;
        writeDriverPattern(DRIVER_PATTERN_NORMAL);
        OCR1A += periodTicks;
      }
      break;
    }

    case GEN_WAIT_A_ON:
      // Защитная пауза A завершена: включаем верхнее плечо A.
      writeDriverPattern(DRIVER_PATTERN_A_ACTIVE);
      OCR1A += activeTicks;
      generatorPhase = GEN_A_ACTIVE;
      break;

    case GEN_A_ACTIVE:
      // Импульс A закончен: выключаем верхнее A.
      // Нижнее A пока остаётся выключенным.
      writeDriverPattern(DRIVER_PATTERN_A_PREPARE);
      OCR1A += driverDeadTimeTicks;
      generatorPhase = GEN_WAIT_A_LOWER;
      break;

    case GEN_WAIT_A_LOWER:
      // Защитная пауза завершена: возвращаем A в нормальное состояние.
      writeDriverPattern(DRIVER_PATTERN_NORMAL);
      OCR1A += interphaseWaitTicks;
      generatorPhase = GEN_WAIT_B_LOWER_OFF;
      break;

    case GEN_WAIT_B_LOWER_OFF:
      // Начало второй половины: выключаем только нижнее плечо B.
      writeDriverPattern(DRIVER_PATTERN_B_PREPARE);
      OCR1A += driverDeadTimeTicks;
      generatorPhase = GEN_WAIT_B_ON;
      break;

    case GEN_WAIT_B_ON:
      // Защитная пауза B завершена: включаем верхнее плечо B.
      writeDriverPattern(DRIVER_PATTERN_B_ACTIVE);
      OCR1A += activeTicks;
      generatorPhase = GEN_B_ACTIVE;
      break;

    case GEN_B_ACTIVE:
      // Импульс B закончен: выключаем верхнее B.
      writeDriverPattern(DRIVER_PATTERN_B_PREPARE);
      OCR1A += driverDeadTimeTicks;
      generatorPhase = GEN_WAIT_B_LOWER;
      break;

    case GEN_WAIT_B_LOWER:
      // Защитная пауза завершена: оба драйвера снова в нормальном состоянии.
      writeDriverPattern(DRIVER_PATTERN_NORMAL);
      OCR1A += interphaseWaitTicks;
      generatorPhase = GEN_PERIOD_BOUNDARY;
      break;

    case GEN_FAULT_RECOVERY:
      // Ошибка состояния: все управляющие 2N7002 были выключены.
      // После паузы возвращаем нижние плечи и ждём следующую границу периода.
      writeDriverPattern(DRIVER_PATTERN_NORMAL);
      generatorRunning = false;
      OCR1A += periodTicks;
      generatorPhase = GEN_PERIOD_BOUNDARY;
      break;

    default:
      // Неизвестное состояние: не продолжаем генерацию.
      // Сначала выключаем все четыре малых ключа, затем вернём нормальное
      // состояние через защитную паузу отдельным событием Timer1.
      writeDriverPattern(DRIVER_PATTERN_ALL_OFF);
      generatorRunning = false;
      OCR1A += driverDeadTimeTicks;
      generatorPhase = GEN_FAULT_RECOVERY;
      break;
  }
}

// ======================================================
// КОНТРОЛЬ ВХОДА RUN
// ======================================================

void updateRunStatus(uint32_t now) {
  // Только эта функция читает физический вход A0/RUN.
  const bool runIsHigh = (PINC & RUN_MASK) != 0;
  bool runLowConfirmed = false;

  if (runIsHigh) {
    runLowTiming = false;
  } else if (!runLowTiming) {
    runLowTiming = true;
    runLowSince = now;
  } else if (now - runLowSince >= UPS_RUN_OFF_DELAY_MS) {
    runLowConfirmed = true;
  }

  // Два флага, которые читает ISR, обновляем атомарно.
  const uint8_t savedSREG = SREG;
  cli();

  if (runIsHigh) {
    mainsPresentConfirmed = false;
    generatorRunRequest = true;
  } else if (runLowConfirmed) {
    mainsPresentConfirmed = true;
    generatorRunRequest = false;
  }

  SREG = savedSREG;
}

// ======================================================
// КОНТРОЛЬ ЗАВИСАНИЯ TIMER1
// ======================================================

// Если Timer1 перестал выдавать события, loop() отключает все четыре
// управляющих сигнала. Через защитный интервал Timer1 возвращает нормальное
// состояние. Это программный резерв на случай остановки самого таймера.
void superviseTimer(uint32_t now) {
  if (!generatorTimingValid) {
    return;
  }

  const uint8_t currentHeartbeat = timer1Heartbeat;

  if (currentHeartbeat != lastTimer1Heartbeat) {
    lastTimer1Heartbeat = currentHeartbeat;
    lastTimer1HeartbeatMs = now;
    return;
  }

  if (now - lastTimer1HeartbeatMs <= UPS_TIMER_STALL_TIMEOUT_MS) {
    return;
  }

  const uint8_t savedSREG = SREG;
  cli();

  TCCR1B = 0;
  TIMSK1 = 0;

  writeDriverPattern(DRIVER_PATTERN_ALL_OFF);
  generatorRunning = false;
  generatorPhase = GEN_FAULT_RECOVERY;

  TCNT1 = 0;
  OCR1A = driverDeadTimeTicks;
  TIFR1 = _BV(OCF1A) | _BV(TOV1);
  TIMSK1 = _BV(OCIE1A);
  TCCR1B = _BV(CS11);

  lastTimer1Heartbeat = timer1Heartbeat;
  lastTimer1HeartbeatMs = now;

  SREG = savedSREG;
}

// ======================================================
// ИЗМЕРЕНИЕ НАПРЯЖЕНИЯ АККУМУЛЯТОРА
// ======================================================

void measureBatteryVoltage() {
  uint32_t adcSum = 0;

  // Усредняем несколько измерений ADC6.
  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    adcSum += analogRead(BATTERY_PIN);
  }

  const uint16_t adcAverage =
      (adcSum + (BATTERY_ADC_SAMPLES / 2)) / BATTERY_ADC_SAMPLES;

  // Коэффициент делителя в сотых: для 330 кОм / 10 кОм получится 3400.
  const uint32_t dividerRatioX100 =
      ((BATTERY_DIVIDER_TOP_OHM + BATTERY_DIVIDER_BOTTOM_OHM) * 100UL) /
      BATTERY_DIVIDER_BOTTOM_OHM;

  // Результат — сотые доли вольта. Арифметика целочисленная.
  batteryVoltageCentiV =
      (static_cast<uint32_t>(adcAverage) * BATTERY_ADC_REFERENCE_MV *
       dividerRatioX100) / (1023UL * 1000UL);

  // Если оба порога равны нулю, батарейная отсечка отключена для макета.
  if (BATTERY_GENERATOR_STOP_CV == 0 &&
      BATTERY_GENERATOR_RECOVERY_CV == 0) {
    generatorBlockFlags &= static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
    return;
  }

  // Неправильные пороги запрещают генерацию.
  if (BATTERY_GENERATOR_STOP_CV <= 0 ||
      BATTERY_GENERATOR_RECOVERY_CV <= BATTERY_GENERATOR_STOP_CV) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
    return;
  }

  // Низкое напряжение защёлкивает запрет до подтверждения сети
  // и достижения порога восстановления.
  if (batteryVoltageCentiV <= BATTERY_GENERATOR_STOP_CV) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
  }
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ
// ======================================================

void setup() {
  // Самое первое действие — все четыре выхода драйверов в LOW.
  const uint8_t savedSREG = SREG;
  cli();

  PORTD &= static_cast<uint8_t>(~DRIVER_OUTPUT_MASK);
  DDRD |= DRIVER_OUTPUT_MASK;

  // После перевода PD4–PD7 в OUTPUT устанавливаем нормальное состояние:
  // оба нижних плеча включены, оба верхних выключены.
  writeDriverPattern(DRIVER_PATTERN_NORMAL);

  // D8 — выход реле зарядки; пока реле всегда выключено.
  PORTB &= static_cast<uint8_t>(~CHARGE_RELAY_MASK);
  DDRB |= CHARGE_RELAY_MASK;

  // Timer1 полностью остановлен и очищен.
  TIMSK1 = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  TIFR1 = _BV(ICF1) | _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);

  // A0 / PC0 — вход RUN, внутренняя подтяжка отключена.
  DDRC &= static_cast<uint8_t>(~RUN_MASK);
  PORTC &= static_cast<uint8_t>(~RUN_MASK);

  SREG = savedSREG;

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

    // Округляем защитную паузу вверх, чтобы не получить меньше заданного.
    calculatedDriverDeadTicks =
        (timerClockHz * UPS_DRIVER_DEADTIME_US + 999999UL) / 1000000UL;

    calculatedPeriodTicks = 2UL * halfPeriodTicks;

    // В каждом общем dead time размещаются две защитные паузы драйвера.
    // Остаток — ожидание до точной границы следующего полупериода.
    if (calculatedDeadTicks > (2UL * calculatedDriverDeadTicks)) {
      calculatedInterphaseTicks =
          calculatedDeadTicks - (2UL * calculatedDriverDeadTicks);
    }

    // Все отдельные интервалы должны помещаться в 16-битный Timer1.
    if (halfPeriodTicks <= 65535UL &&
        calculatedPeriodTicks <= 65535UL &&
        calculatedActiveTicks > 0 &&
        calculatedActiveTicks <= 65535UL &&
        calculatedDeadTicks <= 65535UL &&
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

  // Внутренняя опора АЦП ATmega328P — INTERNAL (номинально 1,1 В).
  analogReference(INTERNAL);
  delay(5);

  // Первые преобразования после переключения опоры отбрасываем.
  for (uint8_t i = 0; i < 4; i++) {
    analogRead(BATTERY_PIN);
  }

  // Стартовая выдержка перед первой оценкой АКБ и RUN.
  delay(UPS_STARTUP_DELAY_MS);

  measureBatteryVoltage();
  lastBatteryMeasureMs = millis();
  updateRunStatus(lastBatteryMeasureMs);

  // Timer1 вызывает автомат на границах и в моменты переключения выходов.
  // Первое событие через один тик начинает первый период без лишней паузы.
  if (generatorTimingValid) {
    const uint8_t startSREG = SREG;
    cli();

    TCNT1 = 0;
    OCR1A = 1;
    TIFR1 = _BV(OCF1A) | _BV(TOV1);
    generatorPhase = GEN_PERIOD_BOUNDARY;
    generatorRunning = false;

    TIMSK1 = _BV(OCIE1A);
    TCCR1B = _BV(CS11);  // Normal mode, предделитель 8.

    lastTimer1Heartbeat = timer1Heartbeat;
    lastTimer1HeartbeatMs = millis();

    SREG = startSREG;
  }
}

// ======================================================
// ОСНОВНОЙ ЦИКЛ
// ======================================================

void loop() {
  const uint32_t now = millis();

  // Обновляем только программные переменные RUN; драйверы физический RUN
  // непосредственно не читают.
  updateRunStatus(now);

  // Контроль таймера и возврат в нормальное состояние при отсутствии событий.
  superviseTimer(now);

  // Измеряем напряжение АКБ раз в секунду, не блокируя Timer1.
  if (now - lastBatteryMeasureMs >= BATTERY_MEASURE_INTERVAL_MS) {
    lastBatteryMeasureMs = now;
    measureBatteryVoltage();
  }

  // Снять батарейную защёлку можно только при подтверждённой сети
  // и фактическом достижении напряжения восстановления.
  if (mainsPresentConfirmed &&
      BATTERY_GENERATOR_STOP_CV > 0 &&
      BATTERY_GENERATOR_RECOVERY_CV > BATTERY_GENERATOR_STOP_CV &&
      batteryVoltageCentiV >= BATTERY_GENERATOR_RECOVERY_CV) {
    generatorBlockFlags &= static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
  }
}

// ======================================================
// ПРЕРЫВАНИЕ TIMER1 — ЕДИНЫЙ АВТОМАТ ЧЕТЫРЁХ ВЫХОДОВ
// ======================================================

ISR(TIMER1_COMPA_vect) {
  ++timer1Heartbeat;
  driverControlStep();
}
