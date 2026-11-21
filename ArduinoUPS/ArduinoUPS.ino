/*
  ArduinoUPS — первый испытательный генератор
  Плата: Arduino Nano, ATmega328P, 16 МГц.

  D5 / PD5 — DRIVER_EN: разрешение драйверов, активный HIGH
  D6 / PD6 — OUT_A: первое плечо, активный HIGH
  D7 / PD7 — OUT_B: второе плечо, активный HIGH
  A0 / PC0 — RUN: фиксируемая команда работы, HIGH = ВКЛ, LOW = ВЫКЛ

  До подключения драйверов проверить сигналы осциллографом.
  На D5, D6 и D7 обязательны внешние подтяжки к GND.
*/

// ======================================================
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
// ======================================================

#define UPS_FREQUENCY_HZ              50  // Частота выходного сигнала, Гц
#define UPS_DEADTIME_PERCENT          10  // Мёртвое время, % от полупериода
#define UPS_STARTUP_DELAY_MS        1000  // Пауза после включения, мс
#define UPS_RUN_FILTER_MS             30  // Время подтверждения команды RUN, мс
#define UPS_DRIVER_ENABLE_DELAY_MS     1  // Пауза после разрешения драйвера, мс

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// ======================================================
// ПИНЫ И РАБОЧИЕ ПЕРЕМЕННЫЕ
// ======================================================

#define RUN_MASK          _BV(PC0)  // A0: вход RUN
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
  if (!generatorTimingValid || generatorRunning) {
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

  // Если RUN успели снять во время задержки, не начинаем генерацию.
  if ((PINC & RUN_MASK) == 0) {
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

  // После стартовой выдержки loop() проверяет текущий уровень RUN.
  // Если RUN уже HIGH, генератор запустится автоматически после фильтрации.
  delay(UPS_STARTUP_DELAY_MS);
}

// ======================================================
// ОСНОВНОЙ ЦИКЛ
// ======================================================

void loop() {
  const bool runIsHigh = (PINC & RUN_MASK) != 0;
  const uint32_t now = millis();

  if (!generatorTimingValid) {
    // Неверные настройки времени: генератор остаётся выключенным.
    return;
  }

  if (generatorRunning) {
    // После принятия STOP не отменяем его: заканчиваем пару A + B.
    if (stopRequested) {
      return;
    }

    if (!runIsHigh) {
      // Фильтруем LOW, чтобы дребезг выключателя не останавливал генератор.
      if (!runLowTiming) {
        runLowTiming = true;
        runLowSince = now;
      } else if (now - runLowSince >= UPS_RUN_FILTER_MS) {
        // Просим Timer1 остановиться только после полного импульса B
        // и следующего за ним интервала dead time.
        stopRequested = true;
        runLowTiming = false;
      }
    } else {
      runLowTiming = false;
    }
    return;
  }

  // Генератор остановлен: RUN работает как обычный фиксируемый выключатель.
  // Устойчивый HIGH запускает генератор даже если присутствовал при старте Nano.
  if (runIsHigh) {
    runLowTiming = false;

    if (!runHighTiming) {
      runHighTiming = true;
      runHighSince = now;
    } else if (now - runHighSince >= UPS_RUN_FILTER_MS) {
      runHighTiming = false;
      startGenerator();
    }
  } else {
    runHighTiming = false;
    runLowTiming = false;
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
