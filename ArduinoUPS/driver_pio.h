// Машинный код для ups_driver из driver_pio.pio.
// В Arduino-Pico .pio не ассемблируется автоматически, поэтому
// предварительно собранная таблица инструкций хранится рядом со скетчем.
#pragma once

#include <hardware/pio.h>

#define UPS_DRIVER_PIO_WRAP_TARGET 6
#define UPS_DRIVER_PIO_WRAP       31

static const uint16_t ups_driver_program_instructions[] = {
    0x80A0, //  0: pull block
    0xA047, //  1: mov y, osr
    0x80A0, //  2: pull block
    0xA0C7, //  3: mov isr, osr
    0x80A0, //  4: pull block
    0xE00A, //  5: set pins, 0b1010

    0x2040, //  6: wait 0 irq 0 (wrap target)

    0xE008, //  7: set pins, 0b1000
    0xA026, //  8: mov x, isr
    0x0049, //  9: jmp x--, 9
    0xE009, // 10: set pins, 0b1001
    0xA022, // 11: mov x, y
    0x004C, // 12: jmp x--, 12
    0xE008, // 13: set pins, 0b1000
    0xA026, // 14: mov x, isr
    0x004F, // 15: jmp x--, 15
    0xE00A, // 16: set pins, 0b1010
    0xA042, // 17: nop (mov y, y)
    0xA027, // 18: mov x, osr
    0x0053, // 19: jmp x--, 19
    0xE002, // 20: set pins, 0b0010
    0xA026, // 21: mov x, isr
    0x0056, // 22: jmp x--, 22
    0xE006, // 23: set pins, 0b0110
    0xA022, // 24: mov x, y
    0x0059, // 25: jmp x--, 25
    0xE002, // 26: set pins, 0b0010
    0xA026, // 27: mov x, isr
    0x005C, // 28: jmp x--, 28
    0xE00A, // 29: set pins, 0b1010
    0xA027, // 30: mov x, osr
    0x005F  // 31: jmp x--, 31
};

static const pio_program_t ups_driver_program = {
    ups_driver_program_instructions,
    32,
    0,
    0
};
