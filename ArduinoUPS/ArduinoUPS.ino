/*
  ArduinoUPS — испытательный генератор и контроль АКБ
  Плата: Arduino Nano, ATmega328P, 16 МГц.

  A0 / PC0 — RUN: HIGH = генерация нужна, LOW = сеть появилась
  A6 / ADC6 — измерение напряжения аккумулятора через делитель
  D5 / PD5 — DRIVER_EN: общее разрешение драйверов, активный HIGH
  D6 / PD6 — OUT_A: первое плечо, активный HIGH
  D7 / PD7 — OUT_B: второе плечо, активный HIGH
  D8 / PB0 — CHARGE_RELAY: управление реле зарядки, активный HIGH

  На D5, D6, D7 и D8 обязательны внешние подтяжки к GND.
  До подключения силовой части проверить все сигналы осциллографом.
*/

// ======================================================
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
// ======================================================

// Настройки генератора
#define UPS_FREQUENCY_HZ                    50  // Частота выходного сигнала, Гц
#define UPS_DEADTIME_PERCENT                10  // Мёртвое время, % от полупериода
#define UPS_STARTUP_DELAY_MS              1000  // Пауза после включения МК, мс
#define UPS_RUN_OFF_DELAY_MS              5000  // Время устойчивого RUN=LOW до остановки, мс
#define UPS_DRIVER_ENABLE_DELAY_MS            1  // Пауза после разрешения драйверов, мс

// Настройки измерения аккумулятора
#define BATTERY_MEASURE_INTERVAL_MS       1000  // Период измерения напряжения АКБ, мс
#define BATTERY_ADC_SAMPLES                 16  // Количество измерений АЦП для усреднения
#define BATTERY_ADC_REFERENCE_V           1.10f // Номинал INTERNAL reference; уточнить калибровкой, В
#define BATTERY_DIVIDER_TOP_OHM         330000UL // Верхний резистор делителя: АКБ+ -> A6, Ом
#define BATTERY_DIVIDER_BOTTOM_OHM       10000UL // Нижний резистор делителя: A6 -> GND, Ом

// Пороговые напряжения АКБ — позднее будут настраиваться из меню
#define BATTERY_GENERATOR_STOP_VOLTAGE      0.0f // Запрет генерации ниже этого напряжения; 0 = отсечка отключена
#define BATTERY_GENERATOR_RECOVERY_VOLTAGE  0.0f // Снятие запрета выше этого напряжения; 0 = отсечка отключена
#define BATTERY_CHARGE_ON_VOLTAGE           0.0f // Включить заряд ниже этого напряжения; пока не используется
#define BATTERY_CHARGE_OFF_VOLTAGE          0.0f // Выключить заряд выше этого напряжения; пока не используется

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

// ======================================================
// ПИНЫ И РАБОЧИЕ ПЕРЕМЕННЫЕ
// ======================================================

#define RUN_MASK          _BV(PC0)  // A0: вход датчика сети / RUN
#define BATTERY_PIN       A6        // A6: аналоговый вход измерения АКБ
#define DRIVER_EN_MASK    _BV(PD5)  // D5: разрешение драйверов
#define OUT_A_MASK        _BV(PD6)  // D6: первое плечо
#define OUT_B_MASK        _BV(PD7)  // D7: второе плечо
#define CHARGE_RELAY_MASK _BV(PB0)  // D8: выход управления реле зарядки

#define OUTPUT_MASK       (DRIVER_EN_MASK | OUT_A_MASK | OUT_B_MASK)
#define BRIDGE_MASK       (OUT_A_MASK | OUT_B_MASK)

// Время между событиями таймера. Эти значения рассчитываются
// при инициализации из частоты и процента dead time.
volatile uint16_t activeTicks = 0;
volatile uint16_t deadTimeTicks = 0;

bool generatorTimingValid = false;
volatile bool generatorRunning = false;

// Общий байт причин запрета: 0 — причин запрета нет, любое ненулевое
// значение запрещает запуск. Каждый модуль меняет только свой бит.
volatile uint8_t generatorBlockFlags = BLOCK_BATTERY_LOW;

// Последнее измеренное напряжение аккумулятора и время последнего измерения.
float batteryVoltage = 0.0f;
uint32_t lastBatteryMeasureMs = 0;

// Состояние последовательности событий Timer1.
enum TimerPhase : uint8_t {
  EVENT_END_A,    // Окончание импульса A: выключить оба плеча, начать DT.
  EVENT_START_B,  // Окончание DT: включить плечо B.
  EVENT_END_B,    // Окончание импульса B: выключить оба плеча, начать DT.
  EVENT_START_A   // Окончание DT: остановиться или включить плечо A.
};

volatile TimerPhase timerPhase = EVENT_END_A;

// Подтверждение присутствия сети: RUN=LOW должен быть непрерывным 5 секунд.
bool runLowTiming = false;
bool mainsPresentConfirmed = false;
uint32_t runLowSince = 0;

// Запрос остановки фиксируется до завершения полного цикла A + B.
volatile bool stopRequested = false;

// ======================================================
// ЗАПУСК И ОСТАНОВКА ГЕНЕРАТОРА
// ======================================================

void stopGenerator() {
  const uint8_t savedSREG = SREG;
  cli();

  // Остановить Timer1 и одновременно выключить DRIVER_EN, OUT_A и OUT_B.
  TCCR1B = 0;
  TIMSK1 = 0;
  PORTD &= static_cast<uint8_t>(~OUTPUT_MASK);
  TIFR1 = _BV(OCF1A);

  stopRequested = false;
  generatorRunning = false;
  timerPhase = EVENT_END_A;

  SREG = savedSREG;
}

void startGenerator() {
  if (!generatorTimingValid || generatorRunning || generatorBlockFlags != 0) {
    return;
  }

  // Сначала разрешаем драйверы при закрытых плечах.
  const uint8_t savedSREG = SREG;
  cli();

  TCCR1B = 0;
  TIMSK1 = 0;
  PORTD &= static_cast<uint8_t>(~BRIDGE_MASK);
  PORTD |= DRIVER_EN_MASK;

  SREG = savedSREG;

  delay(UPS_DRIVER_ENABLE_DELAY_MS);

  // Если RUN сняли или появился запрет во время выдержки, не запускаемся.
  if ((PINC & RUN_MASK) == 0 || generatorBlockFlags != 0) {
    stopGenerator();
    return;
  }

  const uint8_t startSREG = SREG;
  cli();

  // Каждый запуск начинается с плеча A.
  TCNT1 = 0;
  OCR1A = activeTicks;
  TIFR1 = _BV(OCF1A) | _BV(TOV1);

  timerPhase = EVENT_END_A;
  stopRequested = false;

  PORTD &= static_cast<uint8_t>(~BRIDGE_MASK);
  PORTD |= OUT_A_MASK;

  TIMSK1 = _BV(OCIE1A);
  TCCR1B = _BV(CS11);  // Timer1: Normal mode, предделитель 8.

  generatorRunning = true;
  runLowTiming = false;

  SREG = startSREG;
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

  const float adcAverage =
      static_cast<float>(adcSum) / static_cast<float>(BATTERY_ADC_SAMPLES);

  // INTERNAL reference номинально 1,1 В; коэффициент уточняется калибровкой.
  const float adcPinVoltage =
      adcAverage * BATTERY_ADC_REFERENCE_V / 1023.0f;

  const float dividerRatio =
      static_cast<float>(BATTERY_DIVIDER_TOP_OHM + BATTERY_DIVIDER_BOTTOM_OHM) /
      static_cast<float>(BATTERY_DIVIDER_BOTTOM_OHM);

  batteryVoltage = adcPinVoltage * dividerRatio;

  // Если оба порога равны нулю, батарейная отсечка пока отключена для макета.
  if (BATTERY_GENERATOR_STOP_VOLTAGE <= 0.0f &&
      BATTERY_GENERATOR_RECOVERY_VOLTAGE <= 0.0f) {
    generatorBlockFlags &= static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
    return;
  }

  // Если пороги заданы неправильно, оставляем генерацию запрещённой.
  if (BATTERY_GENERATOR_STOP_VOLTAGE <= 0.0f ||
      BATTERY_GENERATOR_RECOVERY_VOLTAGE <= BATTERY_GENERATOR_STOP_VOLTAGE) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
    return;
  }

  // Низкое напряжение защёлкивает запрет. Самостоятельное повышение
  // напряжения после снятия нагрузки не снимает BLOCK_BATTERY_LOW.
  // Сброс разрешается отдельно в loop() только после подтверждения сети.
  if (batteryVoltage <= BATTERY_GENERATOR_STOP_VOLTAGE) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
  }
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ
// ======================================================

void setup() {
  // Самое первое действие — принудительно установить D5, D6 и D7 в LOW.
  const uint8_t savedSREG = SREG;
  cli();

  PORTD &= static_cast<uint8_t>(~OUTPUT_MASK);
  DDRD |= OUTPUT_MASK;

  // D8 — выход реле зарядки. Пока зарядное управление не реализовано,
  // реле всегда остаётся выключенным.
  PORTB &= static_cast<uint8_t>(~CHARGE_RELAY_MASK);
  DDRB |= CHARGE_RELAY_MASK;

  // Timer1 полностью остановлен до отдельной команды запуска.
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

  // Рассчитываем интервалы Timer1.
  // 16 МГц / 8 = 2 МГц; один отсчёт равен 0,5 мкс.
  const uint32_t timerClockHz = F_CPU / 8UL;
  uint32_t halfPeriodTicks = 0;
  uint32_t calculatedDeadTicks = 0;
  uint32_t calculatedActiveTicks = 0;

  if (UPS_FREQUENCY_HZ > 0 &&
      UPS_DEADTIME_PERCENT > 0 &&
      UPS_DEADTIME_PERCENT < 100) {

    halfPeriodTicks = timerClockHz / (2UL * UPS_FREQUENCY_HZ);
    calculatedDeadTicks =
        (halfPeriodTicks * UPS_DEADTIME_PERCENT) / 100UL;
    calculatedActiveTicks = halfPeriodTicks - calculatedDeadTicks;

    // Каждый интервал должен помещаться в 16-битный Timer1.
    if (halfPeriodTicks <= 65535UL &&
        calculatedDeadTicks >= 100UL &&
        calculatedActiveTicks >= 100UL &&
        calculatedDeadTicks <= 65535UL &&
        calculatedActiveTicks <= 65535UL) {

      deadTimeTicks = static_cast<uint16_t>(calculatedDeadTicks);
      activeTicks = static_cast<uint16_t>(calculatedActiveTicks);
      generatorTimingValid = true;
    }
  }

  // Выбираем внутреннюю опору АЦП ATmega328P — INTERNAL (номинально 1,1 В).
  analogReference(INTERNAL);
  delay(5);  // Время на установление внутренней опоры.

  // Первые преобразования после переключения опоры отбрасываем.
  for (uint8_t i = 0; i < 4; i++) {
    analogRead(BATTERY_PIN);
  }

  // Стартовая выдержка перед первой оценкой состояния АКБ и RUN.
  delay(UPS_STARTUP_DELAY_MS);

  measureBatteryVoltage();
  lastBatteryMeasureMs = millis();
}

// ======================================================
// ОСНОВНОЙ ЦИКЛ
// ======================================================

void loop() {
  const uint32_t now = millis();
  const bool runIsHigh = (PINC & RUN_MASK) != 0;

  if (!generatorTimingValid) {
    return;
  }

  // Подтверждаем появление сети только после 5 секунд непрерывного RUN=LOW.
  // Это же подтверждение используется для разрешения восстановления защёлки АКБ.
  if (!runIsHigh) {
    if (!runLowTiming) {
      runLowTiming = true;
      runLowSince = now;
    } else if (now - runLowSince >= UPS_RUN_OFF_DELAY_MS) {
      mainsPresentConfirmed = true;
    }
  } else {
    // При пропадании сети подтверждение сбрасывается, защёлка АКБ сохраняется.
    runLowTiming = false;
    mainsPresentConfirmed = false;
  }

  // Измеряем напряжение АКБ раз в секунду, не блокируя Timer1.
  if (now - lastBatteryMeasureMs >= BATTERY_MEASURE_INTERVAL_MS) {
    lastBatteryMeasureMs = now;
    measureBatteryVoltage();
  }

  // Снять батарейную защёлку можно только при подтверждённой сети
  // и фактическом достижении напряжения восстановления.
  if (mainsPresentConfirmed &&
      BATTERY_GENERATOR_STOP_VOLTAGE > 0.0f &&
      BATTERY_GENERATOR_RECOVERY_VOLTAGE > BATTERY_GENERATOR_STOP_VOLTAGE &&
      batteryVoltage >= BATTERY_GENERATOR_RECOVERY_VOLTAGE) {
    generatorBlockFlags &= static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
  }

  if (generatorRunning) {
    // Любая причина из общего байта запрещает генерацию без задержки RUN.
    if (generatorBlockFlags != 0 || mainsPresentConfirmed) {
      stopRequested = true;
    }

    if (stopRequested) {
      return;
    }
    return;
  }

  // RUN=HIGH включает генерацию сразу, если нет ни одной причины запрета.
  if (runIsHigh && generatorBlockFlags == 0) {
    startGenerator();
  }
}

// ======================================================
// ПРЕРЫВАНИЕ TIMER1 — ЧЕРЕДОВАНИЕ ПЛЕЧ
// ======================================================

ISR(TIMER1_COMPA_vect) {
  switch (timerPhase) {
    case EVENT_END_A:
      // Окончился импульс A: выключаем оба плеча и выдерживаем DT.
      PORTD &= static_cast<uint8_t>(~BRIDGE_MASK);
      OCR1A += deadTimeTicks;
      timerPhase = EVENT_START_B;
      break;

    case EVENT_START_B:
      // DT после A завершён: включаем плечо B.
      PORTD |= OUT_B_MASK;
      OCR1A += activeTicks;
      timerPhase = EVENT_END_B;
      break;

    case EVENT_END_B:
      // Окончился импульс B: выключаем оба плеча и выдерживаем DT.
      PORTD &= static_cast<uint8_t>(~BRIDGE_MASK);
      OCR1A += deadTimeTicks;
      timerPhase = EVENT_START_A;
      break;

    case EVENT_START_A:
      // DT после B завершён: здесь можно завершить полный цикл A+B.
      if (stopRequested) {
        TCCR1B = 0;
        TIMSK1 = 0;

        // Все сигналы генератора выключаются только после полного цикла A+B.
        PORTD &= static_cast<uint8_t>(~OUTPUT_MASK);

        timerPhase = EVENT_END_A;
        stopRequested = false;
        generatorRunning = false;
      } else {
        PORTD |= OUT_A_MASK;
        OCR1A += activeTicks;
        timerPhase = EVENT_END_A;
      }
      break;
  }
}
