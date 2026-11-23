/*
  PIO0 SM1: автономный монитор датчиков стоков.

  Программа занимает 15 инструкций и загружается в PIO0 по адресу 17.
  Общая память PIO0 полностью занята: SM0 (17 слов) + SM1 (15 слов) = 32.

  ISR SM1 хранит четверть периода. После фронта A или B автомат ждёт
  одинаковое число тактов, затем читает GP27/GP28 через MOV OSR,PINS
  и OUT X,2. Если оба бита нулевые, JMP !X переходит в fault.

  При 00 IRQ SET1 одновременно выдаёт NORMAL через optional side-set.
  SM1 (номер 1) имеет более высокий приоритет, чем SM0 (номер 0), поэтому
  аварийный NORMAL аппаратно перекрывает генератор. GP5 и PIO1 не нужны.
*/
#pragma once

#include <hardware/pio.h>

#define DRAIN_MONITOR_PROGRAM_OFFSET 17
#define DRAIN_MONITOR_WRAP_TARGET    17
#define DRAIN_MONITOR_WRAP           31
#define DRAIN_MONITOR_FAULT_PC       30
#define DRAIN_MONITOR_LENGTH         15

static const uint16_t drain_monitor_program_instructions[] = {
    0x2080, //  0: wait 1 gpio 0
    0xA026, //  1: mov x, isr
    0x0042, //  2: jmp x--, 2
    0xA0E0, //  3: mov osr, pins
    0x6022, //  4: out x, 2
    0x002D, //  5: jmp !x, 13
    0x2082, //  6: wait 1 gpio 2
    0xA026, //  7: mov x, isr
    0x0048, //  8: jmp x--, 8
    0xA0E0, //  9: mov osr, pins
    0x6022, // 10: out x, 2
    0x002D, // 11: jmp !x, 13
    0x0000, // 12: jmp 0
    0xDA01, // 13: irq set 1             side-set optional 1010
    0x1A0E  // 14: jmp 14                side-set optional 1010
};

static const pio_program_t drain_monitor_program = {
    drain_monitor_program_instructions,
    DRAIN_MONITOR_LENGTH,
    -1,
    0
};
