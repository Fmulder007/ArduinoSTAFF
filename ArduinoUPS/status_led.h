/*
  Автономная индикация состояния ArduinoUPS для Waveshare RP2040-Zero.

  Встроенный WS2812 подключён к GP16. Для него используется PIO1,
  отдельный от PIO0, который формирует сигналы силовых драйверов.

  Функция updateStatusLed() только читает generatorBlockFlags.
  Она не изменяет блокировки и не управляет генератором.
*/

#pragma once

#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>

extern volatile uint8_t generatorBlockFlags;

constexpr uint8_t STATUS_LED_PIN = 16;
constexpr uint32_t STATUS_LED_PIO_FREQUENCY_HZ = 8000000UL;

// WS2812: 800 кбит/с, 10 тактов PIO на один бит.
static const uint16_t statusLedWs2812Instructions[] = {
  0x6321,  // out x, 1 side 0 [3]
  0x1223,  // jmp !x, 3 side 1 [2]
  0x1200,  // jmp 0 side 1 [2]
  0xA242   // nop side 0 [2]
};

static const struct pio_program statusLedWs2812Program = {
  statusLedWs2812Instructions,
  4,
  -1,
  0
};

enum StatusLedPhase : uint8_t {
  STATUS_LED_GREEN_ON,
  STATUS_LED_GREEN_OFF,
  STATUS_LED_ERROR_ON,
  STATUS_LED_ERROR_PULSE_GAP,
  STATUS_LED_ERROR_CODE_GAP,
  STATUS_LED_ERROR_CYCLE_GAP
};

static PIO statusLedPio = pio1;
static int statusLedSm = -1;
static uint statusLedProgramOffset = 0;
static bool statusLedReady = false;

static bool statusLedStateInitialized = false;
static uint8_t statusLedLastFlags = 0;
static StatusLedPhase statusLedPhase = STATUS_LED_GREEN_ON;
static uint32_t statusLedPhaseStartedMs = 0;

static uint8_t statusLedCurrentBit = 0;
static uint8_t statusLedNextBit = 0;
static uint8_t statusLedPulsesDone = 0;

// Цвет передаётся в формате GRB, который ожидает WS2812.
static constexpr uint8_t STATUS_LED_BRIGHTNESS = 32;
static constexpr uint32_t STATUS_LED_COLOR_OFF = 0x000000UL;
static constexpr uint32_t STATUS_LED_COLOR_GREEN =
    (static_cast<uint32_t>(STATUS_LED_BRIGHTNESS) << 16);
static constexpr uint32_t STATUS_LED_COLOR_RED =
    (static_cast<uint32_t>(STATUS_LED_BRIGHTNESS) << 8);

static uint32_t statusLedLastSentColor = 0;
static uint32_t statusLedPendingColor = 0;
static uint32_t statusLedLastSendUs = 0;
static bool statusLedHasSentColor = false;
static bool statusLedColorPending = false;

static void statusLedRequestColor(uint32_t grbColor) {
  if (statusLedHasSentColor && grbColor == statusLedLastSentColor) {
    statusLedColorPending = false;
    return;
  }

  statusLedPendingColor = grbColor;
  statusLedColorPending = true;
}

// Отправка кадра WS2812 выполняется не чаще одного раза в 100 мкс.
// Это гарантирует необходимую низкую паузу для фиксации предыдущего цвета.
static void statusLedServiceOutput() {
  if (!statusLedReady || !statusLedColorPending) {
    return;
  }

  const uint32_t nowUs = micros();

  if (statusLedHasSentColor &&
      static_cast<uint32_t>(nowUs - statusLedLastSendUs) < 100UL) {
    return;
  }

  // В FIFO передаётся 24-битное слово GRB в старших 24 битах.
  pio_sm_put_blocking(
      statusLedPio,
      static_cast<uint>(statusLedSm),
      statusLedPendingColor << 8);

  statusLedLastSendUs = micros();
  statusLedLastSentColor = statusLedPendingColor;
  statusLedHasSentColor = true;
  statusLedColorPending = false;
}

static uint8_t statusLedFindNextBit(uint8_t flags, uint8_t firstBit) {
  for (uint8_t bit = firstBit; bit < 8; ++bit) {
    if (flags & static_cast<uint8_t>(1U << bit)) {
      return bit;
    }
  }

  return 8;
}

static void statusLedStartPattern(uint8_t flags, uint32_t nowMs) {
  statusLedPhaseStartedMs = nowMs;

  if (flags == 0) {
    statusLedPhase = STATUS_LED_GREEN_ON;
    statusLedRequestColor(STATUS_LED_COLOR_GREEN);
    return;
  }

  statusLedCurrentBit = statusLedFindNextBit(flags, 0);
  statusLedPulsesDone = 0;
  statusLedPhase = STATUS_LED_ERROR_ON;
  statusLedRequestColor(STATUS_LED_COLOR_RED);
}

static bool initializeStatusLed() {
  // Делитель PIO рассчитывается целочисленно в формате 16.8.
  const uint64_t divider256 =
      (static_cast<uint64_t>(clock_get_hz(clk_sys)) * 256ULL +
       (STATUS_LED_PIO_FREQUENCY_HZ / 2UL)) /
      STATUS_LED_PIO_FREQUENCY_HZ;

  const uint32_t dividerInteger =
      static_cast<uint32_t>(divider256 / 256ULL);

  const uint8_t dividerFraction =
      static_cast<uint8_t>(divider256 % 256ULL);

  if (dividerInteger == 0 || dividerInteger > 65535UL) {
    return false;
  }

  if (!pio_can_add_program(statusLedPio, &statusLedWs2812Program)) {
    return false;
  }

  statusLedSm = pio_claim_unused_sm(statusLedPio, false);
  if (statusLedSm < 0) {
    return false;
  }

  statusLedProgramOffset =
      pio_add_program(statusLedPio, &statusLedWs2812Program);

  pio_gpio_init(statusLedPio, STATUS_LED_PIN);
  pio_sm_set_consecutive_pindirs(
      statusLedPio,
      static_cast<uint>(statusLedSm),
      STATUS_LED_PIN,
      1,
      true);

  pio_sm_config config = pio_get_default_sm_config();

  sm_config_set_wrap(
      &config,
      statusLedProgramOffset,
      statusLedProgramOffset + 3);

  sm_config_set_sideset(&config, 1, false, false);
  sm_config_set_sideset_pins(&config, STATUS_LED_PIN);
  sm_config_set_out_shift(&config, false, true, 24);
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

  sm_config_set_clkdiv_int_frac8(
      &config,
      static_cast<uint16_t>(dividerInteger),
      dividerFraction);

  pio_sm_init(
      statusLedPio,
      static_cast<uint>(statusLedSm),
      statusLedProgramOffset,
      &config);

  pio_sm_set_enabled(
      statusLedPio,
      static_cast<uint>(statusLedSm),
      true);

  statusLedReady = true;
  return true;
}

// Вызывать из loop() постоянно. Все интервалы неблокирующие.
static void updateStatusLed() {
  if (!statusLedReady) {
    return;
  }

  const uint32_t nowMs = millis();
  const uint8_t flags = generatorBlockFlags;

  // Любое изменение маски немедленно перезапускает соответствующий рисунок.
  if (!statusLedStateInitialized || flags != statusLedLastFlags) {
    statusLedStateInitialized = true;
    statusLedLastFlags = flags;
    statusLedStartPattern(flags, nowMs);
    statusLedServiceOutput();
    return;
  }

  switch (statusLedPhase) {
    case STATUS_LED_GREEN_ON:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 100UL) {
        statusLedRequestColor(STATUS_LED_COLOR_OFF);
        statusLedPhase = STATUS_LED_GREEN_OFF;
        statusLedPhaseStartedMs = nowMs;
      }
      break;

    case STATUS_LED_GREEN_OFF:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 900UL) {
        statusLedRequestColor(STATUS_LED_COLOR_GREEN);
        statusLedPhase = STATUS_LED_GREEN_ON;
        statusLedPhaseStartedMs = nowMs;
      }
      break;

    case STATUS_LED_ERROR_ON:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 100UL) {
        statusLedRequestColor(STATUS_LED_COLOR_OFF);
        statusLedPulsesDone++;
        statusLedPhaseStartedMs = nowMs;

        if (statusLedPulsesDone < static_cast<uint8_t>(statusLedCurrentBit + 1)) {
          statusLedPhase = STATUS_LED_ERROR_PULSE_GAP;
        } else {
          statusLedNextBit = statusLedFindNextBit(
              flags,
              static_cast<uint8_t>(statusLedCurrentBit + 1));

          if (statusLedNextBit < 8) {
            statusLedPhase = STATUS_LED_ERROR_CODE_GAP;
          } else {
            statusLedPhase = STATUS_LED_ERROR_CYCLE_GAP;
          }
        }
      }
      break;

    case STATUS_LED_ERROR_PULSE_GAP:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 200UL) {
        statusLedRequestColor(STATUS_LED_COLOR_RED);
        statusLedPhase = STATUS_LED_ERROR_ON;
        statusLedPhaseStartedMs = nowMs;
      }
      break;

    case STATUS_LED_ERROR_CODE_GAP:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 600UL) {
        statusLedCurrentBit = statusLedNextBit;
        statusLedPulsesDone = 0;
        statusLedRequestColor(STATUS_LED_COLOR_RED);
        statusLedPhase = STATUS_LED_ERROR_ON;
        statusLedPhaseStartedMs = nowMs;
      }
      break;

    case STATUS_LED_ERROR_CYCLE_GAP:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 1500UL) {
        statusLedCurrentBit = statusLedFindNextBit(flags, 0);
        statusLedPulsesDone = 0;
        statusLedRequestColor(STATUS_LED_COLOR_RED);
        statusLedPhase = STATUS_LED_ERROR_ON;
        statusLedPhaseStartedMs = nowMs;
      }
      break;
  }

  statusLedServiceOutput();
}
