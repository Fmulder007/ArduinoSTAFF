/*
  PIO0 SM1: автономный монитор датчиков стоков.

  Программа занимает 13 инструкций и загружается в PIO0 по адресу 17.
  Общая память PIO0: генератор SM0 (17 слов) + монитор SM1 (13 слов)
  = 30 из 32 слов.

  На старте ISR получает quarterCheckLoopCount, Y — адрес wait_b (24).
  После выборки A SM1 переходит на wait_b; после выборки B — на wait_a (17).

  При 00 инструкция IRQ SET1 одновременно выдаёт NORMAL через optional
  side-set4. SM1 (номер 1) имеет приоритет над SM0 (номер 0) на GP0..GP3
  и удерживает NORMAL в fault_hold. GP5 и PIO1 для этой связи не нужны.
*/
#pragma once

#include <hardware/pio.h>

#define DRAIN_MONITOR_PROGRAM_OFFSET 17
#define DRAIN_MONITOR_WAIT_A_PC      17
#define DRAIN_MONITOR_WAIT_B_PC      24
#define DRAIN_MONITOR_WRAP_TARGET   17
#define DRAIN_MONITOR_WRAP          29
#define DRAIN_MONITOR_FAULT_PC      28
#define DRAIN_MONITOR_LENGTH        13

static const uint16_t drain_monitor_program_instructions[] = {
    0x2080, //  0: wait 1 gpio 0
    0xA026, //  1: mov x, isr
    0x0042, //  2: jmp x--, 2
    0xA0E0, //  3: mov osr, pins
    0x6022, //  4: out x, 2
    0x002B, //  5: jmp !x, 11
    0xA0A2, //  6: mov pc, y
    0x2082, //  7: wait 1 gpio 2
    0xE051, //  8: set y, 17 (absolute PIO0 address)
    0xA026, //  9: mov x, isr
    0x0002, // 10: jmp 2
    0xDA01, // 11: irq set 1             side-set optional 1010
    0x1A0C  // 12: jmp 12                side-set optional 1010
};

static const pio_program_t drain_monitor_program = {
    drain_monitor_program_instructions,
    DRAIN_MONITOR_LENGTH,
    -1,
    0
};
