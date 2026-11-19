// The ATmega328P's registers that mazerunner-core touches, simulated (uchip.cpp): each one an object whose reads
// and writes the chip sees (its timers, its ADC, its ports), so the firmware's own register code runs unchanged.
#pragma once
#include <stdint.h>

enum SimReg {
  SIM_PORTB, SIM_PORTC, SIM_PORTD, SIM_PINB, SIM_PINC, SIM_PIND, SIM_DDRB, SIM_DDRC, SIM_DDRD,
  SIM_TCCR1A, SIM_TCCR1B, SIM_TCCR2A, SIM_TCCR2B, SIM_OCR2A, SIM_TIMSK2, SIM_ADCSRA, SIM_ADCSRB, SIM_ADMUX,
  SIM_SREG, SIM_REGS
};
uint8_t sim_reg_read(int reg);
void sim_reg_write(int reg, uint8_t value);
uint16_t sim_adc_read(void);

struct SimReg8 {
  int id;
  operator uint8_t() const { return sim_reg_read(id); }
  SimReg8 &operator=(unsigned long v) { sim_reg_write(id, (uint8_t)v); return *this; }
  SimReg8 &operator=(const SimReg8 &o) { sim_reg_write(id, (uint8_t)o); return *this; }
  SimReg8 &operator|=(unsigned long v) { sim_reg_write(id, (uint8_t)(sim_reg_read(id) | v)); return *this; }
  SimReg8 &operator&=(unsigned long v) { sim_reg_write(id, (uint8_t)(sim_reg_read(id) & v)); return *this; }
  SimReg8 &operator^=(unsigned long v) { sim_reg_write(id, (uint8_t)(sim_reg_read(id) ^ v)); return *this; }
};
struct SimAdc16 {
  operator uint16_t() const { return sim_adc_read(); }
};
extern SimReg8 sim_regs[SIM_REGS];
extern SimAdc16 sim_adc16;

#define PORTB sim_regs[SIM_PORTB]
#define PORTC sim_regs[SIM_PORTC]
#define PORTD sim_regs[SIM_PORTD]
#define PINB sim_regs[SIM_PINB]
#define PINC sim_regs[SIM_PINC]
#define PIND sim_regs[SIM_PIND]
#define DDRB sim_regs[SIM_DDRB]
#define DDRC sim_regs[SIM_DDRC]
#define DDRD sim_regs[SIM_DDRD]
#define TCCR1A sim_regs[SIM_TCCR1A]
#define TCCR1B sim_regs[SIM_TCCR1B]
#define TCCR2A sim_regs[SIM_TCCR2A]
#define TCCR2B sim_regs[SIM_TCCR2B]
#define OCR2A sim_regs[SIM_OCR2A]
#define TIMSK2 sim_regs[SIM_TIMSK2]
#define ADCSRA sim_regs[SIM_ADCSRA]
#define ADCSRB sim_regs[SIM_ADCSRB]
#define ADMUX sim_regs[SIM_ADMUX]
#define SREG sim_regs[SIM_SREG]
#define ADC sim_adc16

// bits (ATmega328P datasheet)
#define CS10 0
#define CS11 1
#define CS12 2
#define WGM10 0
#define WGM11 1
#define WGM12 3
#define WGM13 4
#define WGM20 0
#define WGM21 1
#define WGM22 3
#define CS20 0
#define CS21 1
#define CS22 2
#define OCIE2A 1
#define OCIE2B 2
#define TOIE2 0
#define ADPS0 0
#define ADPS1 1
#define ADPS2 2
#define ADIE 3
#define ADIF 4
#define ADATE 5
#define ADSC 6
#define ADEN 7
#define MUX0 0
#define ADLAR 5
#define REFS0 6
#define REFS1 7
#define SREG_I 7
#define _BV(bit) (1 << (bit))
