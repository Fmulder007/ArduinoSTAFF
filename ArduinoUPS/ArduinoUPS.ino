/*
  ArduinoUPS — первый испытательный генератор
  Плата: Arduino Nano, ATmega328P, 16 МГц.

  D5 / PD5 — DRIVER_EN: разрешение драйверов, активный HIGH
  D6 / PD6 — OUT_A: первое плечо, активный HIGH
  D7 / PD7 — OUT_B: второе плечо, активный HIGH
  A0 / PC0 — RUN: фиксируемая команда работы, HIGH = ВКЛ, LOW = ВЫКЛ
  A6 / ADC6 — измерение напряжения АКБ через резистивный делитель

  До подключения драйверов проверить сигналы осциллографом.
  На D5, D6 и D7 обязательны внешние подтяжки к GND.
*/

// ======================================================
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
// ======================================================

#define UPS_FREQUENCY_HZ              50  // Частота выходного сигнала, Гц
#define UPS_DEADTIME_PERCENT          10  // Мёртвое время, % от полупериода
#define UPS_STARTUP_DELAY_MS        1000  // Пауза после включения, мс
#define UPS_RUN_FILTER_MS             30  // Фильтр переключения RUN, мс
#define UPS_DRIVER_ENABLE_DELAY_MS     1  // Пауза после разрешения драйвера, мс

#define BATTERY_MEASURE_INTERVAL_MS 1000  // Период измерения напряжения АКБ, мс
#define BATTERY_ADC_SAMPLES           16  // Количество измерений АЦП для усреднения
#define BATTERY_ADC_REFERENCE_V     5.00f // Опорное напряжение АЦП (AVcc), В
#define BATTERY_DIVIDER_TOP_OHM   100000UL // Верхний резистор делителя: АКБ+ -> A6, Ом
#define BATTERY_DIVIDER_BOTTOM_OHM 10000UL // Нижний резистор делителя: A6 -> GND, Ом
#define BATTERY_MIN_VOLTAGE          0.0f // Нижний порог АКБ, В; 0 = порог не настроен, генерация заблокирована
#define BATTERY_RECOVERY_MARGIN_V   0.5f // Запас восстановления выше нижнего порога, В

// Коды причин запрета генерации. Каждому модулю принадлежит свой бит.
#define BLOCK_BATTERY_LOW             0x01 // Бит 0: низкое напряжение аккумулятора
#define BLOCK_OVERHEAT                0x02 // Бит 1: перегрев
#define BLOCK_OVERLOAD                0x04 // Бит 2: перегрузка
#define BLOCK_MAINS_FAULT             0x08 // Бит 3: неисправность/недопустимое состояние сети
#define BLOCK_SENSOR_FAULT            0x10 // Бит 4: неисправность датчика
#define BLOCK_RESERVED_5              0x20 // Бит 5: резерв
#define BLOCK_RESERVED_6              0x40 // Бит 6: резерв
#define BLOCK_RESERVED_7              0x80 // Бит 7: резерв

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// ======================================================
// ПИНЫ И РАБОЧИЕ ПЕРЕМЕННЫЕ
// ======================================================

#define RUN_MASK          _BV(PC0)  // A0: вход RUN
#define BATTERY_PIN       A6        // A6: аналоговый вход измерения АКБ
#define DRIVER_EN_MASK    _BV(PD5)  // D5: разрешение драйверов
#define OUT_A_MASK        _BV(PD6)  // D6: первое плечо
#define OUT_B_MASK        _BV(PD7)  // D7: второе плечо

#define OUTPUT_MASK       (DRIVER_EN_MASK | OUT_A_MASK | OUT_B_MASK)
#define BRIDGE_MASK       (OUT_A_MASK | OUT_B_MASK)

// Время между событиями таймера. Эти значения рассчитываются
// при запуске из UPS_FREQUENCY_HZ и UPS_DEADTIME_PERCENT.
volatile uint16_t activeTicks = 0;
volatile uint16_t deadTimeTicks = 0;

bool generatorTimingValid = false;
volatile bool generatorRunning = false;

// Общий байт причин запрета: 0 — генерация разрешена, любое ненулевое
// значение — запрещена. Каждый модуль меняет только принадлежащий ему бит.
// До первого корректного измерения АКБ запуск запрещён.
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

// Фильтр устойчивых уровней RUN.
bool runHighTiming = false;
bool runLowTiming = false;
uint32_t runHighSince = 0;
uint32_t runLowSince = 0;

// Запрос остановки фиксируется до завершения полного цикла A + B.
volatile bool stopRequested = false;

// ======================================================
// ЗАПУСК И ОСТАНОВКА ГЕНЕРАТОРА
// ======================================================

void stopGenerator() {
  const uint8_t savedSREG = SREG;
  cli();

  // Остановить Timer1 и одновременно выключить все три управляющих выхода.
  TCCR1B = 0;
  TIMSK1 = 0;
  PORTD &= static_cast<uint8_t>(~OUTPUT_MASK);

  // Удалить ожидающее событие сравнения.
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

  // Перед включением драйверов оба плеча гарантированно выключены.
  const uint8_t savedSREG = SREG;
  cli();

  TCCR1B = 0;
  TIMSK1 = 0;
  PORTD &= static_cast<uint8_t>(~BRIDGE_MASK);
  PORTD |= DRIVER_EN_MASK;

  SREG = savedSREG;

  // Даём драйверам включиться, пока оба силовых плеча закрыты.
  delay(UPS_DRIVER_ENABLE_DELAY_MS);

  // Повторно проверяем RUN и общие запреты после выдержки драйверов.
  if ((PINC & RUN_MASK) == 0 || generatorBlockFlags != 0) {
    stopGenerator();
    return;
  }

  const uint8_t startSREG = SREG;
  cli();

  // Начинаем новый период с плеча A.
  TCNT1 = 0;
  OCR1A = activeTicks;
  TIFR1 = _BV(OCF1A) | _BV(TOV1);

  timerPhase = EVENT_END_A;
  stopRequested = false;

  PORTD &= static_cast<uint8_t>(~BRIDGE_MASK);
  PORTD |= OUT_A_MASK;

  TIMSK1 = _BV(OCIE1A);  // Разрешить прерывание сравнения A.
  TCCR1B = _BV(CS11);    // Запустить Timer1: Normal mode, делитель 8.

  generatorRunning = true;

  SREG = startSREG;
}

// ======================================================
// ИЗМЕРЕНИЕ НАПРЯЖЕНИЯ АККУМУЛЯТОРА
// ======================================================

void measureBatteryVoltage() {
  // Усредняем несколько измерений ADC6, чтобы уменьшить шум.
  uint32_t adcSum = 0;
  for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    adcSum += analogRead(BATTERY_PIN);
  }

  const float adcAverage =
      static_cast<float>(adcSum) / static_cast<float>(BATTERY_ADC_SAMPLES);

  // Пересчитываем напряжение на A6 в напряжение аккумулятора
  // с учётом фактического делителя.
  const float adcPinVoltage =
      adcAverage * BATTERY_ADC_REFERENCE_V / 1023.0f;

  const float dividerRatio =
      static_cast<float>(BATTERY_DIVIDER_TOP_OHM + BATTERY_DIVIDER_BOTTOM_OHM) /
      static_cast<float>(BATTERY_DIVIDER_BOTTOM_OHM);

  batteryVoltage = adcPinVoltage * dividerRatio;

  // Если порог ещё не задан, батарейный модуль сохраняет запрет.
  // Это не позволяет случайно запустить инвертор с неизвестными настройками АКБ.
  if (BATTERY_MIN_VOLTAGE <= 0.0f) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
    return;
  }

  // Гистерезис: запрет включается ниже нижнего порога,
  // а снимается только после восстановления напряжения с запасом.
  if (batteryVoltage < BATTERY_MIN_VOLTAGE) {
    generatorBlockFlags |= BLOCK_BATTERY_LOW;
  } else if (batteryVoltage >=
             BATTERY_MIN_VOLTAGE + BATTERY_RECOVERY_MARGIN_V) {
    generatorBlockFlags &= static_cast<uint8_t>(~BLOCK_BATTERY_LOW);
  }
}

// ======================================================
// ИНИЦИАЛИЗАЦИЯ
// ======================================================

void setup() {
  // Самое первое действие — принудительно установить D5, D6 и D7 в LOW.
  const uint8_t savedSREG = SREG;
  cli();

  PORTD &= static_cast<uint8_t>(~OUTPUT_MASK); // Сначала выходные защёлки в LOW.
  DDRD |= OUTPUT_MASK;                         // Затем включить режим OUTPUT.

  // Timer1 должен быть остановлен до отдельной команды запуска.
  TIMSK1 = 0;
  TCCR1A = 0;  // Normal mode, аппаратные выходы сравнения отключены.
  TCCR1B = 0;  // Нет источника тактирования — таймер остановлен.
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  TIFR1 = _BV(ICF1) | _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);

  // A0 / PC0 — вход RUN, внутренняя подтяжка отключена.
  DDRC &= static_cast<uint8_t>(~RUN_MASK);
  PORTC &= static_cast<uint8_t>(~RUN_MASK);

  SREG = savedSREG;

  // Рассчитать временные интервалы для Timer1.
  // Nano 16 МГц / предделитель 8 = 2 МГц, один отсчёт 0,5 мкс.
  uint32_t timerClockHz = F_CPU / 8UL;
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

    // Каждый отдельный интервал должен помещаться в 16-битный Timer1.
    // Минимум 100 отсчётов оставляет достаточный запас для ISR.
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

  // После стартовой выдержки измеряем АКБ до разрешения запуска.
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
    // Неверные настройки времени: генератор остаётся выключенным.
    return;
  }

  // Обновляем напряжение аккумулятора раз в секунду.
  if (now - lastBatteryMeasureMs >= BATTERY_MEASURE_INTERVAL_MS) {
    lastBatteryMeasureMs = now;
    measureBatteryVoltage();
  }

  if (generatorRunning) {
    // Любой установленный бит требует штатной остановки.
    // Запрос защёлкивается до окончания полного цикла A + B.
    if (generatorBlockFlags != 0) {
      stopRequested = true;
    }

    if (stopRequested) {
      return;
    }

    if (!runIsHigh) {
      // Фильтруем LOW, чтобы дребезг выключателя не создавал STOP.
      if (!runLowTiming) {
        runLowTiming = true;
        runLowSince = now;
      } else if (now - runLowSince >= UPS_RUN_FILTER_MS) {
        stopRequested = true;
        runLowTiming = false;
      }
    } else {
      runLowTiming = false;
    }
    return;
  }

  // Запуск возможен только при RUN=HIGH и полном отсутствии запретов.
  if (generatorBlockFlags != 0 || !runIsHigh) {
    runHighTiming = false;
    runLowTiming = false;
    return;
  }

  // RUN — фиксируемая команда уровня. HIGH может присутствовать ещё
  // до подачи питания на Arduino; предварительный переход LOW->HIGH не нужен.
  if (!runHighTiming) {
    runHighTiming = true;
    runHighSince = now;
  } else if (now - runHighSince >= UPS_RUN_FILTER_MS) {
    runHighTiming = false;
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
      // Если поступил STOP, завершаем работу до начала нового импульса A.
      if (stopRequested) {
        TCCR1B = 0;  // Остановить Timer1.
        TIMSK1 = 0;  // Запретить дальнейшие прерывания таймера.

        // Оба плеча и разрешение драйверов выключаем.
        PORTD &= static_cast<uint8_t>(~OUTPUT_MASK);

        timerPhase = EVENT_END_A;
        stopRequested = false;
        generatorRunning = false;
      } else {
        // Остановка не запрошена — начинаем следующий импульс A.
        PORTD |= OUT_A_MASK;
        OCR1A += activeTicks;
        timerPhase = EVENT_END_A;
      }
      break;
  }
}
