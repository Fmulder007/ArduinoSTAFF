/*
  PIO1 SM0: независимый монитор двух датчиков стоков в режиме GENERATION.

  Монитор ждёт фактического фронта команды A (GP0), через четверть периода
  считывает GP27/GP28 одной командой IN PINS, 2 и проверяет комбинацию 00.
  Затем аналогично ждёт команду B (GP2).

  При 00 PIO1 поднимает GP5 (внутреннюю линию PIO1 -> PIO0).
  PIO0 обнаруживает этот уровень непосредственно в своём активном цикле,
  переводит GP0..GP3 в NORMAL и устанавливает собственный IRQ1.

  PIO1 программа занимает 20 инструкций. Все временные расчёты целочисленные.
*/

#pragma once

#include <hardware/pio.h>

#define DRAIN_MONITOR_WRAP_TARGET 0
#define DRAIN_MONITOR_WRAP        19
#define DRAIN_MONITOR_FAULT_PC    18

static const uint16_t drain_monitor_program_instructions[] = {
    0x80A0, //  0: pull block               (quarter-period loop count)
    0x2080, //  1: wait 1 gpio 0            (A gate command active)
    0xA027, //  2: mov x, osr
    0x0043, //  3: jmp x--, 3               (quarter-period delay)
    0xA0C3, //  4: mov isr, null
    0x4002, //  5: in pins, 2               (GP27, GP28)
    0xA026, //  6: mov x, isr
    0x0032, //  7: jmp !x, 18              (both sensors LOW -> fault)

    0x2000, //  8: wait 0 gpio 0            (A command ended)
    0x2082, //  9: wait 1 gpio 2            (B gate command active)
    0xA027, // 10: mov x, osr
    0x004B, // 11: jmp x--, 11              (quarter-period delay)
    0xA0C3, // 12: mov isr, null
    0x4002, // 13: in pins, 2               (GP27, GP28)
    0xA026, // 14: mov x, isr
    0x0032, // 15: jmp !x, 18              (both sensors LOW -> fault)

    0x2002, // 16: wait 0 gpio 2            (B command ended)
    0x0001, // 17: jmp 1                   (wait for next A command)

    0xE001, // 18: set pins, 1              (raise GP5 fault link)
    0x0013  // 19: jmp 19                  (latch fault level)
};

static const pio_program_t drain_monitor_program = {
    drain_monitor_program_instructions,
    20,
    0,
    0
};
