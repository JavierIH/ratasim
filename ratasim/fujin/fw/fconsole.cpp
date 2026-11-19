// Fujin's console: what its firmware prints (std::printf; over USB on the robot) goes to the simulator (fchip.cpp).
// Defined here, in a file without <cstdio>: glibc's header has inline versions of some of these.
#include <cstdarg>
#include <cstring>

#include "fsim.h"

extern "C" {
int vsnprintf(char *s, size_t n, const char *fmt, va_list ap);

int vprintf(const char *fmt, va_list ap) {
    char b[1024];
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    fsim::console_out(b, n < (int)sizeof(b) ? (size_t)n : sizeof(b) - 1);
    return n;
}
int printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const int n = vprintf(fmt, ap);
    va_end(ap);
    return n;
}
int puts(const char *s) {
    fsim::console_out(s, strlen(s));
    fsim::console_out("\n", 1);
    return 1;
}
int putchar(int c) {
    const char ch = (char)c;
    fsim::console_out(&ch, 1);
    return c;
}
}
