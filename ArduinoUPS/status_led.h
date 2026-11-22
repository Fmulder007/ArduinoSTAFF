/*
  Автономная индикация состояния ArduinoUPS для Waveshare RP2040-Zero.

  Встроенный WS2812 подключён к GP16 (UART0 TX).
  Передача выполняется UART0 + DMA с инверсией выхода GPIO:
  PIO0 и PIO1 полностью свободны для силовой автоматики.

  millis() задаёт моменты смены цвета. Передача 24 бит запускается DMA
  асинхронно; процессор не формирует импульсы и не ждёт окончания кадра.

  Функция updateStatusLed() читает generatorBlockFlags и generatorIsRunning.
  Она не изменяет эти флаги и не управляет генератором.
*/

#pragma once

#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>

extern volatile uint8_t generatorBlockFlags;
extern volatile bool generatorIsRunning;

constexpr uint8_t STATUS_LED_PIN = 16;
constexpr uint32_t STATUS_LED_LATCH_TIME_US = 300UL;
constexpr uint32_t STATUS_LED_FRAME_GUARD_US = 50UL;
constexpr uint8_t STATUS_LED_DATA_BITS = 24;

// UART работает в формате 8N1, а выход GP16 инвертирован.
// Один UART-символ (start + 8 data + stop = 10 бит) формирует один бит WS2812.
// При clk_peri=125 МГц UART baud = 7,8125 Мбит/с, один UART-бит = 128 нс.
//
// После инверсии:
//   0xFC (UART data LSB first): HIGH 3 бита = 384 нс, LOW 7 = 896 нс.
//   0xE0 (UART data LSB first): HIGH 6 бит = 768 нс, LOW 4 = 512 нс.
// Общая длительность каждого бита WS2812 — 1,28 мкс.
//
// UART idle/stop после инверсии удерживает линию LOW. DMA заполняет TX FIFO
// без участия CPU между отдельными байтами.
constexpr uint8_t STATUS_LED_UART_ZERO = 0xFC;
constexpr uint8_t STATUS_LED_UART_ONE = 0xE0;

enum StatusLedPhase : uint8_t {
  STATUS_LED_NORMAL_ON,
  STATUS_LED_NORMAL_OFF,
  STATUS_LED_ERROR_ON,
  STATUS_LED_ERROR_PULSE_GAP,
  STATUS_LED_ERROR_CODE_GAP,
  STATUS_LED_ERROR_CYCLE_GAP
};

static bool statusLedReady = false;
static int statusLedDmaChannel = -1;
static dma_channel_config statusLedDmaConfig;

static uint8_t statusLedTxBuffer[STATUS_LED_DATA_BITS];

static bool statusLedDmaActive = false;
static bool statusLedWaitForFrameEnd = false;
static bool statusLedLatchActive = false;
static uint32_t statusLedFrameStartedUs = 0;
static uint32_t statusLedLatchStartedUs = 0;

static bool statusLedStateInitialized = false;
static uint8_t statusLedLastFlags = 0;
static bool statusLedLastGeneratorRunning = false;
static StatusLedPhase statusLedPhase = STATUS_LED_NORMAL_ON;
static uint32_t statusLedPhaseStartedMs = 0;

static uint8_t statusLedCurrentBit = 0;
static uint8_t statusLedNextBit = 0;
static uint8_t statusLedPulsesDone = 0;

// Цвет хранится в порядке GRB, который ожидает WS2812.
static constexpr uint8_t STATUS_LED_BRIGHTNESS = 32;
static constexpr uint32_t STATUS_LED_COLOR_OFF = 0x000000UL;
static constexpr uint32_t STATUS_LED_COLOR_GREEN =
    (static_cast<uint32_t>(STATUS_LED_BRIGHTNESS) << 16);
static constexpr uint32_t STATUS_LED_COLOR_RED =
    (static_cast<uint32_t>(STATUS_LED_BRIGHTNESS) << 8);
static constexpr uint32_t STATUS_LED_COLOR_YELLOW =
    (20UL << 16) | (32UL << 8);

static uint32_t statusLedNormalColor = STATUS_LED_COLOR_GREEN;

static uint32_t statusLedLastSentColor = 0;
static uint32_t statusLedPendingColor = 0;
static bool statusLedHasSentColor = false;
static bool statusLedColorPending = false;

static void statusLedRequestColor(uint32_t grbColor) {
  grbColor &= 0x00FFFFFFUL;

  if (statusLedHasSentColor && grbColor == statusLedLastSentColor) {
    statusLedColorPending = false;
    return;
  }

  statusLedPendingColor = grbColor;
  statusLedColorPending = true;
}

static void statusLedEncodeFrame(uint32_t grbColor) {
  for (uint8_t i = 0; i < STATUS_LED_DATA_BITS; ++i) {
    const uint8_t bitPosition =
        static_cast<uint8_t>(23U - i);

    statusLedTxBuffer[i] =
        (grbColor & (1UL << bitPosition))
            ? STATUS_LED_UART_ONE
            : STATUS_LED_UART_ZERO;
  }
}

// Запуск кадра и контроль его окончания полностью неблокирующие.
// После передачи UART оставляет инвертированный TX в LOW для reset/latch.
static void statusLedServiceOutput() {
  if (!statusLedReady) {
    return;
  }

  const uint32_t nowUs = micros();

  if (statusLedDmaActive) {
    if (dma_channel_is_busy(static_cast<uint>(statusLedDmaChannel))) {
      return;
    }

    statusLedDmaActive = false;
    statusLedWaitForFrameEnd = true;
  }

  if (statusLedWaitForFrameEnd) {
    // DMA мог заполнить TX FIFO раньше, чем UART физически выдал весь кадр.
    // Выдерживаем время полного кадра с запасом, не блокируя CPU.
    if (static_cast<uint32_t>(nowUs - statusLedFrameStartedUs) <
        STATUS_LED_FRAME_GUARD_US) {
      return;
    }

    statusLedWaitForFrameEnd = false;
    statusLedLatchActive = true;
    statusLedLatchStartedUs = nowUs;
    return;
  }

  if (statusLedLatchActive) {
    if (static_cast<uint32_t>(nowUs - statusLedLatchStartedUs) <
        STATUS_LED_LATCH_TIME_US) {
      return;
    }

    statusLedLatchActive = false;
  }

  if (!statusLedColorPending) {
    return;
  }

  statusLedEncodeFrame(statusLedPendingColor);

  // 24 однобитовых UART-символа передаются DMA в TX FIFO UART0.
  // DREQ UART0_TX автоматически ограничивает скорость заполнения FIFO.
  dma_channel_configure(
      static_cast<uint>(statusLedDmaChannel),
      &statusLedDmaConfig,
      &uart_get_hw(uart0)->dr,
      statusLedTxBuffer,
      STATUS_LED_DATA_BITS,
      true);

  statusLedFrameStartedUs = nowUs;
  statusLedDmaActive = true;

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

static void statusLedStartPattern(
    uint8_t flags,
    bool generatorRunning,
    uint32_t nowMs) {

  statusLedPhaseStartedMs = nowMs;

  if (flags == 0) {
    statusLedNormalColor = generatorRunning
        ? STATUS_LED_COLOR_YELLOW
        : STATUS_LED_COLOR_GREEN;
    statusLedPhase = STATUS_LED_NORMAL_ON;
    statusLedRequestColor(statusLedNormalColor);
    return;
  }

  statusLedCurrentBit = statusLedFindNextBit(flags, 0);
  statusLedPulsesDone = 0;
  statusLedPhase = STATUS_LED_ERROR_ON;
  statusLedRequestColor(STATUS_LED_COLOR_RED);
}

static bool initializeStatusLed() {
  if (statusLedReady) {
    return true;
  }

  // UART0 работает на GP16. Его тактовая частота должна позволять
  // сформировать 10 UART-битов примерно за 1,25 мкс.
  const uint32_t peripheralClockHz = clock_get_hz(clk_peri);
  if (peripheralClockHz == 0) {
    return false;
  }

  const uint32_t uartBaud = peripheralClockHz / 16UL;
  const uint64_t wsBitCellNs =
      (160000000000ULL + (peripheralClockHz / 2UL)) /
      peripheralClockHz;

  // Допускаем стандартное окно длительности бита WS2812: 1,25 мкс ± 0,15 мкс.
  if (wsBitCellNs < 1100ULL || wsBitCellNs > 1400ULL) {
    return false;
  }

  statusLedDmaChannel = dma_claim_unused_channel(false);
  if (statusLedDmaChannel < 0) {
    return false;
  }

  const uint32_t actualBaud = uart_init(uart0, uartBaud);
  if (actualBaud == 0) {
    dma_channel_unclaim(static_cast<uint>(statusLedDmaChannel));
    statusLedDmaChannel = -1;
    return false;
  }

  uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(uart0, true);

  // Сначала удерживаем вывод LOW, затем передаём его UART0.
  // Инверсия превращает UART start/data/stop в импульсы WS2812.
  gpio_init(STATUS_LED_PIN);
  gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
  gpio_put(STATUS_LED_PIN, false);
  gpio_set_outover(STATUS_LED_PIN, GPIO_OVERRIDE_LOW);
  gpio_set_function(STATUS_LED_PIN, GPIO_FUNC_UART);
  gpio_set_outover(STATUS_LED_PIN, GPIO_OVERRIDE_INVERT);

  statusLedDmaConfig =
      dma_channel_get_default_config(
          static_cast<uint>(statusLedDmaChannel));

  channel_config_set_transfer_data_size(
      &statusLedDmaConfig,
      DMA_SIZE_8);

  channel_config_set_read_increment(
      &statusLedDmaConfig,
      true);

  channel_config_set_write_increment(
      &statusLedDmaConfig,
      false);

  channel_config_set_dreq(
      &statusLedDmaConfig,
      uart_get_dreq(uart0, true));

  statusLedReady = true;
  return true;
}

// Вызывать из loop() постоянно. millis() управляет рисунком,
// а UART0 + DMA отправляют каждый кадр асинхронно.
static void updateStatusLed() {
  if (!statusLedReady) {
    return;
  }

  const uint32_t nowMs = millis();
  const uint8_t flags = generatorBlockFlags;
  const bool generatorRunning = generatorIsRunning;

  // Красный код зависит только от маски блокировок.
  // Изменение generatorIsRunning не сбрасывает красную последовательность.
  const bool flagsChanged =
      !statusLedStateInitialized || flags != statusLedLastFlags;
  const bool normalStateChanged =
      statusLedStateInitialized &&
      flags == 0 &&
      statusLedLastFlags == 0 &&
      generatorRunning != statusLedLastGeneratorRunning;

  if (flagsChanged || normalStateChanged) {
    statusLedStateInitialized = true;
    statusLedLastFlags = flags;
    statusLedLastGeneratorRunning = generatorRunning;
    statusLedStartPattern(flags, generatorRunning, nowMs);
    statusLedServiceOutput();
    return;
  }

  // Во время красной последовательности изменение флага генератора
  // запоминается, но не меняет рисунок красных вспышек.
  statusLedLastGeneratorRunning = generatorRunning;

  switch (statusLedPhase) {
    case STATUS_LED_NORMAL_ON:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 100UL) {
        statusLedRequestColor(STATUS_LED_COLOR_OFF);
        statusLedPhase = STATUS_LED_NORMAL_OFF;
        statusLedPhaseStartedMs = nowMs;
      }
      break;

    case STATUS_LED_NORMAL_OFF:
      if (static_cast<uint32_t>(nowMs - statusLedPhaseStartedMs) >= 900UL) {
        statusLedRequestColor(statusLedNormalColor);
        statusLedPhase = STATUS_LED_NORMAL_ON;
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
