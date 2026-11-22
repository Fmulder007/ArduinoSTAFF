/*
  PIO0: двухтактный генератор с контролем внешнего сигнала DRAIN_FAULT.

  Четыре выхода GP0..GP3 передаются одним обязательным side-set шириной 4 бита.
  Благодаря этому генератор сохраняет отдельные временные интервалы, но
  внутри активных импульсов может проверять вход DRAIN_FAULT (GP5).

  Если PIO1 обнаруживает ошибку датчиков стоков, он поднимает GP5.
  PIO0 проверяет GP5 в цикле активного импульса и переходит в аварийную
  ветвь: NORMAL=0b1010, затем IRQ1 и бесконечное удержание NORMAL.

  Программа занимает 29 из 32 инструкций PIO0.
*/

#pragma once

#include <hardware/pio.h>

#define UPS_DRIVER_PIO_WRAP_TARGET 6
#define UPS_DRIVER_PIO_WRAP        26
#define UPS_DRIVER_PIO_FAULT_PC    27

static const uint16_t ups_driver_program_instructions[] = {
    0x94A0, //  0: pull block             side 1010
    0xB447, //  1: mov y, osr             side 1010
    0x94A0, //  2: pull block             side 1010
    0xB4C7, //  3: mov isr, osr           side 1010
    0x94A0, //  4: pull block             side 1010
    0xF40A, //  5: set pins, 1010         side 1010

    0x3440, //  6: wait 0 irq 0           side 1010 (wrap target)

    0xB026, //  7: mov x, isr             side 1000
    0x1048, //  8: jmp x--, 8             side 1000

    0xB222, //  9: mov x, y              side 1001
    0xB242, // 10: nop                    side 1001
    0x12DB, // 11: jmp pin, 27            side 1001
    0x124B, // 12: jmp x--, 11            side 1001

    0xB026, // 13: mov x, isr             side 1000
    0x104D, // 14: jmp x--, 13            side 1000
    0xB427, // 15: mov x, osr             side 1010
    0x144F, // 16: jmp x--, 15            side 1010

    0xA426, // 17: mov x, isr             side 0010
    0x0451, // 18: jmp x--, 17            side 0010

    0xAC22, // 19: mov x, y              side 0110
    0xAC42, // 20: nop                    side 0110
    0x0CDB, // 21: jmp pin, 27            side 0110
    0x0C55, // 22: jmp x--, 21            side 0110

    0xA426, // 23: mov x, isr             side 0010
    0x0457, // 24: jmp x--, 23            side 0010
    0xB427, // 25: mov x, osr             side 1010
    0x1459, // 26: jmp x--, 25            side 1010 (wrap top)

    0xD401, // 27: irq set 1              side 1010 (fault: NORMAL first)
    0x141C  // 28: jmp 28                 side 1010 (hold NORMAL)
};

static const pio_program_t ups_driver_program = {
    ups_driver_program_instructions,
    29,
    0,
    0
};
