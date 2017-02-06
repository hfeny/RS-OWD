#ifndef __MODBUS_H
#define __MODBUS_H

#define MODBUS_REG_MAX                  16      // макс.кол-во регистров в проекте
/* Modbus definitions */
#define MBUS_ADU_MAX_LENGHT             256     //
#define MBUS_ADDRESS_DEFAULT            127     //
/* Modbus functions */
#define MBUS_F_NONE                     0x00    // 0x00
#define MBUS_F_READ_HOLDING_REGISTER    0x03    // 3
#define MBUS_F_WRITE_SINGLE_REGISTER    0x06    // 6
#define MBUS_F_WRITE_MULTIPLE_REGISTERS 0x10    // 16
#define MBUS_F_ERROR_MASK               0x80
/* Modbus exception code */
#define MBUS_EX_NONE                    0x00
#define MBUS_EX_ILLEGAL_FUNCTION        0x01
#define MBUS_EX_ILLEGAL_DATA_ADDRESS    0x02
#define MBUS_EX_ILLEGAL_DATA_VALUE      0x03
#define MBUS_EX_SLAVE_DEVICE_FAILURE    0x04
/* Modbus UART data structure */
typedef struct {
uint8_t ADU[MBUS_ADU_MAX_LENGHT];               // буфер modbus ADU
uint8_t rxtimer;                                // таймаут приема
uint8_t rxcnt;                                  // счетчик принятых символов
uint8_t rxlen;                                  // принято символов
uint8_t txcnt;                                  // счетчик переданных символов
uint8_t txlen;                                  // байт для отправки
uint8_t rxgap;                                  // окончание приема
uint8_t delay;                                  // задержка приема 3,5 символа
} MBUS_UART_DATA;
/* Modbus registers reservation */
extern uint16_t MODBUS_REG[MODBUS_REG_MAX];
extern uint8_t DEVICE_ADDR;
/* Modbus parse functions */
void modbus(MBUS_UART_DATA *DATA,uint8_t DEVICE_ADDR,uint16_t *R);
void modbus_F3(MBUS_UART_DATA *DATA, uint16_t *R);
void modbus_F6(MBUS_UART_DATA *DATA, uint16_t *R);
void modbus_F16(MBUS_UART_DATA *DATA, uint16_t *R);
void modbus_EX(MBUS_UART_DATA *DATA,uint8_t exception);
/* Common modbus function */
uint16_t CRC16(uint8_t *bufPtr, uint8_t len);
#endif /* __MODBUS_H */