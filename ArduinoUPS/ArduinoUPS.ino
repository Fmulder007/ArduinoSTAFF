/*
  ArduinoUPS — early safe-start skeleton
  Target: classic Arduino Nano, ATmega328P, 16 MHz.

  Pin map:
    D5 / PD5 — DRIVER_EN (active HIGH)
    D6 / PD6 — OUT_A     (active HIGH)
    D7 / PD7 — OUT_B     (active HIGH)

  External pull-down resistors to GND are required on all three signals.
  This file intentionally does not generate pulses yet.
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

constexpr uint8_t UPS_OUTPUT_MASK =
    _BV(PD5) | _BV(PD6) | _BV(PD7);

void setup() {
  // Preserve the current interrupt state, then make the first GPIO
  // operations force all UPS control outputs LOW.
  const uint8_t savedSREG = SREG;
  cli();

  // First clear the output latches while the pins are still inputs.
  PORTD &= static_cast<uint8_t>(~UPS_OUTPUT_MASK);

  // Then enable the output drivers. The pins now actively drive LOW.
  DDRD |= UPS_OUTPUT_MASK;

  // Keep Timer1 fully stopped and unarmed until the generator is
  // deliberately implemented and explicitly started.
  TIMSK1 = 0;  // Disable all Timer1 interrupts.
  TCCR1A = 0;  // Normal mode; disconnect compare outputs.
  TCCR1B = 0;  // Clock select = 0: Timer1 stopped.
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;

  // Timer flags are cleared by writing logic one to the corresponding bits.
  TIFR1 = _BV(ICF1) | _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);

  // Restore the interrupt state that was present on entry to setup().
  SREG = savedSREG;

  // Further initialization will be added only after the startup
  // sequence and remaining logic are agreed.
}

void loop() {
  // Generator and input handling are not implemented yet.
}
