/*
  PIO0: двухтактный генератор с аппаратным контролем DRAIN_FAULT.

  GP0..GP3 передаются обязательным side-set шириной 4 бита.
  GP5 — аварийная линия от PIO1 drain_monitor.

  PIO0 проверяет GP5 в каждом активном импульсе. При GP5=1
  аварийная ветвь одновременно устанавливает NORMAL=1010 и IRQ1,
  затем бесконечно удерживает безопасное состояние.

  Программа занимает 30 из 32 инструкций PIO0.
*/
#pragma once

#include <hardware/pio.h>

#define UPS_DRIVER_PIO_WRAP_TARGET 6
#define UPS_DRIVER_PIO_WRAP        27
#define UPS_DRIVER_PIO_FAULT_PC    28

static const uint16_t ups_driver_program_instructions[] = {
    0x94A0, //  0: pull block              side 1010
    0xB447, //  1: mov y, osr              side 1010
    0x94A0, //  2: pull block              side 1010
    0xB4C7, //  3: mov isr, osr            side 1010
    0x94A0, //  4: pull block              side 1010
    0xF40A, //  5: set pins, 1010          side 1010

    0x3440, //  6: wait 0 irq 0            side 1010 (wrap target)

    0xB026, //  7: mov x, isr              side 1000
    0x1048, //  8: jmp x--, 8              side 1000

    0xB222, //  9: mov x, y               side 1001
    0xB242, // 10: nop                     side 1001
    0x12DC, // 11: jmp pin, 28             side 1001
    0x124B, // 12: jmp x--, 11             side 1001

    0xB026, // 13: mov x, isr              side 1000
    0x104E, // 14: jmp x--, 14             side 1000

    0xB442, // 15: nop                     side 1010
    0xB427, // 16: mov x, osr              side 1010
    0x1451, // 17: jmp x--, 17             side 1010

    0xA426, // 18: mov x, isr              side 0010
    0x0453, // 19: jmp x--, 19             side 0010

    0xAC22, // 20: mov x, y               side 0110
    0xAC42, // 21: nop                     side 0110
    0x0CDC, // 22: jmp pin, 28             side 0110
    0x0C56, // 23: jmp x--, 22             side 0110

    0xA426, // 24: mov x, isr              side 0010
    0x0459, // 25: jmp x--, 25             side 0010
    0xB427, // 26: mov x, osr              side 1010
    0x145B, // 27: jmp x--, 27             side 1010 (wrap top)

    0xD401, // 28: irq set 1               side 1010 (NORMAL first)
    0x141D  // 29: jmp 29                  side 1010 (hold NORMAL)
};

static const pio_program_t ups_driver_program = {
    ups_driver_program_instructions,
    30,
    0,
    0
};
