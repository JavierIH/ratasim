/* Forced into every file of Futura's firmware (fw/Makefile: -include fsim.h): its sources stay as they are.
 *   Millis      its millisecond counter (delay.c), read through fsim_millis(): its waiting loops poll it, and each
 *               read takes the simulated chip's time, as the loop does on the chip
 *   main        firmware_main, run by fchip.c in a coroutine
 *   printf      fsim_printf: formatted, then each character through its own fputc() (usart.c: USART1), which the
 *               Keil library it was built with calls; fputc and fgetc renamed so as not to be the host's */
#ifndef FSIM_H
#define FSIM_H
#include "stm32f4xx.h"
#define Millis (*fsim_millis())
#define main firmware_main
#define printf fsim_printf
#define fputc fw_fputc
#define fgetc fw_fgetc
#endif
