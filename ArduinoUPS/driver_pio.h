/*
  PIO0 SM0: двухтактный генератор.

  Программа занимает 17 инструкций по адресу 0..16.
  SM0 использует обязательный side-set4 на GP0..GP3.
  Таймерные значения загружаются в Y, ISR и OSR из C++ через EXEC,
  поэтому команды инициализации не расходуют память инструкций PIO.
*/
#pragma once

#include <hardware/pio.h>

#define UPS_DRIVER_PIO_WRAP_TARGET 0
#define UPS_DRIVER_PIO_WRAP        16
#define UPS_DRIVER_PIO_LENGTH      17

static const uint16_t ups_driver_program_instructions[] = {
    0x3440, //  0: wait 0 irq 0             side 1010
    0xB026, //  1: mov x, isr               side 1000
    0x1042, //  2: jmp x--, 2               side 1000
    0xB222, //  3: mov x, y                 side 1001
    0x1244, //  4: jmp x--, 4               side 1001
    0xB026, //  5: mov x, isr               side 1000
    0x1046, //  6: jmp x--, 6               side 1000
    0xB527, //  7: mov x, osr               side 1010 [1]
    0x1448, //  8: jmp x--, 8               side 1010
    0xA426, //  9: mov x, isr               side 0010
    0x044A, // 10: jmp x--, 10              side 0010
    0xAC22, // 11: mov x, y                 side 0110
    0x0C4C, // 12: jmp x--, 12              side 0110
    0xA426, // 13: mov x, isr               side 0010
    0x044E, // 14: jmp x--, 14              side 0010
    0xB427, // 15: mov x, osr               side 1010
    0x1450  // 16: jmp x--, 16              side 1010 (wrap)
};

static const pio_program_t ups_driver_program = {
    ups_driver_program_instructions,
    UPS_DRIVER_PIO_LENGTH,
    0,
    0
};
