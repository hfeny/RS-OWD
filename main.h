#ifndef __MAIN_H
#define __MAIN_H

#include "modbus.h"

#define F_CPU                           16000000

#define F_SYSCLK                        16000000                // system clock frequency 16mHz
#define EEPROM_BASE                     0x4000
#define EEPROM_END_ADDRESS              0x7F

#define FIRST_ADC_CH                    0x03                    // AIN3
#define LAST_ADC_CH                     0x04                    // AIN4

#define RS485_BAUD                      9600                    // скорость UART

#define UART_DIV                        (uint32_t)F_SYSCLK/(uint32_t)RS485_BAUD
#define UART_DIVM                       (uint8_t)((UART_DIV) & (uint8_t)0x0F)
#define UART_DIVL                       (uint8_t)((UART_DIV) >> 4)


#if RS485_BAUD < 38400
  #define T35  (uint8_t)(38500/RS485_BAUD)*4                     // для одного цикла таймера = 250мкс
#else
  #define T35  7
#endif

//extern volatile uint16_t tick_ms;

void RS485TX(MBUS_UART_DATA *uart);
//void eeprom_write(uint8_t addr,uint8_t value);
//uint8_t eeprom_read(uint8_t addr);

#endif /* __MAIN_H */