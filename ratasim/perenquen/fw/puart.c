// Perenquen's serial line, UART1 to its HC-05 Bluetooth module, in place of aversive4dspic's dsPIC UART driver
// (modules/comm/dspic/uart): its API (uart.h) over the simulated chip. The driver reaches the UART's registers through
// pointers (uart_regs[]), which the simulated registers cannot see being read or written, so the line is simulated
// here, as the driver behaves: bytes received at the link's speed (921600 baud) into its 128-byte receive FIFO, the
// receive event (cmdline.c's emergency(): ctrl-C, "pop") called for each from its interrupt; bytes sent into its
// 128-byte transmit FIFO, out at the link's speed, uart_send() waiting while it is full (its log lines pace its loops).
// printf (stdio on the chip, aversive's write() to UART0) comes here through psim_printf.
#define _GNU_SOURCE
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <aversive.h>
#include <uart.h>

#define RX_FIFO 128
#define TX_FIFO 128         // UART0_TX_FIFO_SIZE: what is waiting to go out; uart_send() waits while it is full
#define RX_IPL 4
void psim_poll(void);
void psim_wait(void);
int psim_in_isr(void);

static void (*rx_event)(char), (*tx_event)(char);
static char rx_fifo[RX_FIFO];
static unsigned rx_head, rx_len;
static char pending[4096];          // sent by the simulator, not yet received by the chip
static unsigned pending_len;
static char tx[1 << 16];             // gone out: the simulator reads it
static unsigned tx_len, tx_queued;   // tx_queued: the last ones of tx still in the FIFO

void uart_init(void) {}
int8_t uart_setconf(uint8_t num, struct uart_config *u) { (void)num; (void)u; return 0; }
void uart_getconf(uint8_t num, struct uart_config *u) { (void)num; (void)u; }
void uart_register_tx_event(uint8_t num, void (*f)(char)) { if (num == 0) tx_event = f; }
void uart_register_rx_event(uint8_t num, void (*f)(char)) { if (num == 0) rx_event = f; }

int uart_send_nowait(uint8_t num, char c) {
  if (num != 0 || tx_queued >= TX_FIFO) return -1;
  if (tx_len < sizeof(tx)) {
    tx[tx_len++] = c;
    tx_queued++;
  }
  if (tx_event) tx_event(c);
  return (unsigned char)c;
}
int uart_send(uint8_t num, char c) {
  int r;
  while ((r = uart_send_nowait(num, c)) == -1) {
    if (psim_in_isr()) tx_queued--;    // in an interrupt the driver polls the UART: it goes out
    else psim_wait();
  }
  return r;
}

int uart_recv_nowait(uint8_t num) {
  psim_poll();
  if (num != 0 || !rx_len) return -1;
  char c = rx_fifo[rx_head];
  rx_head = (rx_head + 1) % RX_FIFO;
  rx_len--;
  return (unsigned char)c;
}
int uart_recv(uint8_t num) {
  int c;
  while ((c = uart_recv_nowait(num)) == -1) {}
  return c;
}

int psim_vfprintf(FILE *f, const char *fmt, va_list ap) {
  (void)f;
  char b[1024];
  int n = vsnprintf(b, sizeof(b), fmt, ap);
  for (int i = 0; i < n && i < (int)sizeof(b) - 1; i++) uart_send(0, b[i]);
  return n;
}
int psim_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int n = psim_vfprintf(NULL, fmt, ap);
  va_end(ap);
  return n;
}

// ---- the chip (pchip.c) ----
void puart_service(uint32_t bytes) {   // its transmit and receive interrupts: `bytes` each way this period
  tx_queued = tx_queued > bytes ? tx_queued - bytes : 0;
  unsigned ipl = ((volatile SRBITS *)&sim_SR)->IPL;
  if (ipl >= RX_IPL) return;
  for (uint32_t i = 0; i < bytes && pending_len; i++) {
    char c = pending[0];
    memmove(pending, pending + 1, --pending_len);
    if (rx_len < RX_FIFO) {
      rx_fifo[(rx_head + rx_len) % RX_FIFO] = c;
      rx_len++;
    }
    if (rx_event) rx_event(c);
  }
}
void puart_reset(void) { rx_len = pending_len = tx_len = tx_queued = 0; }

// ---- Simulator API ----
uint32_t psim_uart_rx(const char *data, uint32_t n) {   // bytes from the operator's terminal (over Bluetooth)
  if (n > sizeof(pending) - pending_len) n = sizeof(pending) - pending_len;
  memcpy(pending + pending_len, data, n);
  pending_len += n;
  return n;
}
uint32_t psim_uart_tx(char *buffer, uint32_t size) {   // bytes the chip sent
  uint32_t n = tx_len < size ? tx_len : size;
  memcpy(buffer, tx, n);
  memmove(tx, tx + n, tx_len - n);
  tx_len -= n;
  return n;
}
