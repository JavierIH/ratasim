/* Forced into every file of Perenquen's firmware (and its aversive4dspic modules) built for the host: what XC16
 * gives a dsPIC program and the host's compiler does not.
 *
 * - The compiler's builtins the code calls (the oscillator's unlocked OSCCON writes): the simulated chip's.
 * - The two dsPIC instructions its C has inline, as GNU as macros for the host's assembler: `repeat #39` (a busy
 *   wait in sensor.c; '#' starts a comment on x86, so `repeat` takes no argument and does nothing: the simulated
 *   chip does not count cycles) and `Reset` (cmdline.c, "pop" on the command line): a call into the chip that
 *   never returns.
 * - Its interrupt attributes: on the host they would ask for x86 interrupt handlers; the chip calls the ISRs. */
#ifndef PDSPIC_H
#define PDSPIC_H
#include <stdint.h>

void sim_write_osccon(int high, unsigned value);
#define __builtin_write_OSCCONH(v) sim_write_osccon(1, (v))
#define __builtin_write_OSCCONL(v) sim_write_osccon(0, (v))
#define __builtin_nop() do {} while (0)

__asm__(".ifndef PDSPIC_MACROS\n"
        ".set PDSPIC_MACROS, 1\n"
        ".macro repeat\n.endm\n"
        ".macro Reset\ncall sim_cpu_reset@PLT\n.endm\n"
        ".endif\n");

#define __interrupt__ __used__
#define interrupt __used__
#define no_auto_psv __used__
#define auto_psv __used__
#endif
