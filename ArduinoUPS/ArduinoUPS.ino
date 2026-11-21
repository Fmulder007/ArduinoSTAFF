/*
  ArduinoUPS — безопасная начальная заготовка
  Плата: классическая Arduino Nano, ATmega328P, 16 МГц.

  D5 / PD5 — DRIVER_EN
  D6 / PD6 — OUT_A
  D7 / PD7 — OUT_B

  На всех трёх выходах обязательны внешние подтяжки к GND.
  Генерация импульсов пока не реализована.
*/

// ======================================================
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ ГЕНЕРАТОРА
// При будущем добавлении меню эти значения будут изменяться
// в оперативной памяти без перекомпиляции прошивки.
// ======================================================

#define UPS_FREQUENCY_HZ       50  // Частота выходного напряжения, Гц
#define UPS_DEADTIME_PERCENT   10  // Dead time, % от длительности полупериода

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Маска только трёх управляющих выходов: PD5, PD6 и PD7.
const uint8_t UPS_OUTPUT_MASK =
    _BV(PD5) | _BV(PD6) | _BV(PD7);

// Исходные настройки хранятся в обычных переменных RAM.
// В дальнейшем меню будет менять именно эти переменные.
struct GeneratorSettings {
  uint16_t frequencyHz;
  uint8_t deadTimePercent;
};

GeneratorSettings generatorSettings = {
  UPS_FREQUENCY_HZ,
  UPS_DEADTIME_PERCENT
};

// Все производные временные параметры также являются обычными
// переменными RAM и рассчитываются при инициализации Timer1.
struct GeneratorTiming {
  uint32_t halfPeriodTicks;
  uint16_t activeTicks;
  uint16_t deadTimeTicks;
};

GeneratorTiming generatorTiming = {0, 0, 0};
bool generatorTimingValid = false;

// Останавливает и подготавливает Timer1, а также рассчитывает
// временные параметры из текущих значений generatorSettings.
// Вызывать только при остановленном генераторе и выключенных выходах.
bool timer1Initialize() {
  // Атомарно останавливаем Timer1 и запрещаем его прерывания.
  const uint8_t savedSREG = SREG;
  cli();

  TIMSK1 = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  TIFR1 = _BV(ICF1) | _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);

  // Таймер уже остановлен, поэтому дальнейший расчёт можно выполнять
  // с восстановленными глобальными прерываниями.
  SREG = savedSREG;

  // Тактовая частота Timer1: 16 МГц / 8 = 2 МГц.
  // Делитель /8 — внутренняя настройка реализации, не пользовательский параметр.
  uint32_t timerClockHz = F_CPU / 8UL;

  // Проверяем исходные настройки до выполнения деления.
  if (generatorSettings.frequencyHz == 0 ||
      generatorSettings.deadTimePercent == 0 ||
      generatorSettings.deadTimePercent >= 100) {
    generatorTimingValid = false;
    return false;
  }

  // Рассчитываем длительность полупериода в отсчётах Timer1.
  uint32_t halfPeriodTicks =
      timerClockHz / (2UL * generatorSettings.frequencyHz);

  // Timer1 — 16-битный. Один интервал между событиями должен
  // помещаться в диапазон счётчика.
  if (halfPeriodTicks == 0 || halfPeriodTicks > 65535UL) {
    generatorTimingValid = false;
    return false;
  }

  // Рассчитываем dead time и активное время одного плеча.
  uint32_t deadTimeTicks =
      (halfPeriodTicks * generatorSettings.deadTimePercent) / 100UL;

  uint32_t activeTicks = halfPeriodTicks - deadTimeTicks;

  if (deadTimeTicks == 0 || activeTicks == 0 ||
      deadTimeTicks > 65535UL || activeTicks > 65535UL) {
    generatorTimingValid = false;
    return false;
  }

  // Сохраняем результаты в рабочие переменные RAM.
  generatorTiming.halfPeriodTicks = halfPeriodTicks;
  generatorTiming.deadTimeTicks = static_cast<uint16_t>(deadTimeTicks);
  generatorTiming.activeTicks = static_cast<uint16_t>(activeTicks);

  generatorTimingValid = true;
  return true;
}

void setup() {
  // Самое первое действие программы — безопасные уровни выходов.
  const uint8_t savedSREG = SREG;
  cli();

  // Сначала записываем LOW, пока выводы ещё находятся в режиме INPUT.
  PORTD &= static_cast<uint8_t>(~UPS_OUTPUT_MASK);

  // Затем переводим D5, D6 и D7 в режим OUTPUT.
  DDRD |= UPS_OUTPUT_MASK;

  // Восстанавливаем состояние глобальных прерываний.
  SREG = savedSREG;

  // Останавливаем Timer1 и рассчитываем его рабочие интервалы.
  // До отдельной команды запуска счётчик остаётся остановленным.
  generatorTimingValid = timer1Initialize();
}

void loop() {
  // Генератор и обработка входов пока не реализованы.
}
