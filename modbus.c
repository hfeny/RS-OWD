#include "stm8s.h"
#include "main.h"
#include "modbus.h"

void modbus(MBUS_UART_DATA *DATA,uint8_t DEVICE_ADDR,uint16_t *R)
{
  uint16_t CRC;
  // проверка адреса устройства
  if((DATA->ADU[0]==DEVICE_ADDR)&&(DATA->rxlen>5))
  {
    CRC = CRC16(DATA->ADU,DATA->rxlen-2);
    // проверка контрольной суммы
    if((DATA->ADU[DATA->rxlen-2]==(CRC&0x00FF))&&(DATA->ADU[DATA->rxlen-1]==(CRC>>8)))
    {
      // обрабатываем прин€тую функцию
      switch(DATA->ADU[1])
      {
        case MBUS_F_READ_HOLDING_REGISTER:
          modbus_F3(DATA,R);
          break;
        case MBUS_F_WRITE_SINGLE_REGISTER:
          modbus_F6(DATA,R);
          break;
        case MBUS_F_WRITE_MULTIPLE_REGISTERS:
          modbus_F16(DATA,R);
          break;
        default:
          modbus_EX(DATA,MBUS_EX_ILLEGAL_FUNCTION);
      }
      // вставл€ем контрольную сумму в ответ
      CRC = CRC16(DATA->ADU,DATA->txlen-2);
      DATA->ADU[DATA->txlen-2] = CRC;
      DATA->ADU[DATA->txlen-1] = CRC>>8;
    }
  }
  // очищаем флаги
  DATA->rxgap = 0;
  DATA->txcnt = 0;
}

void modbus_F3(MBUS_UART_DATA *DATA, uint16_t *R)
{
  // Modbus Function 03(0x03) Read holding registers
  // запрос
  // ADU[0] - device address
  // ADU[1] - function code
  // ADU[2,3] - starting address 0x0000-0xFFFF
  // ADU[4,5] - number of registers 0x0001-0x007B
  // ответ
  // ADU[0] - device address
  // ADU[1] - function code
  // ADU[2] - byte count
  // ADU[3] - start data blocks
  uint8_t i,j;
  uint16_t reg_start,reg_count;
  reg_start = ((DATA->ADU[2]<<8)+DATA->ADU[3]); // начачальный регистр
  reg_count = ((DATA->ADU[4]<<8)+DATA->ADU[5]); // кол-во регистров
  j = 3;                                        // смещение на начало блока данных
  if((reg_start<MODBUS_REG_MAX)&&((reg_start+reg_count)<MODBUS_REG_MAX+1))
  {
    for(i=0;i<reg_count;i++)
    {
      DATA->ADU[j]=R[i+reg_start]>>8;
      DATA->ADU[j+1]=R[i+reg_start];
      j = j + 2;
    }
    DATA->ADU[2] = reg_count*2;                   // кол-во байт данных дл€ отправки
    DATA->txlen = reg_count*2+5;                  // длина ADU фрейма дл€ отправки
  }
  else
  {
    modbus_EX(DATA,MBUS_EX_ILLEGAL_DATA_ADDRESS);
  }
}

void modbus_F6(MBUS_UART_DATA *DATA, uint16_t *R)
{
  // Modbus Function 06(0x06) Write single register
  // ADU[0] - device address
  // ADU[1] - function code
  // ADU[2,3] - register address 0x0000-0xFFFF
  // ADU[4,5] - register value 0x0000-0xFFFF
  // ответ
  // ADU[0] - device address
  // ADU[1] - function code
  // ADU[2,3] - register address 0x0000-0xFFFF
  // ADU[4,5] - register value 0x0000-0xFFFF
  uint16_t reg_addr;
  reg_addr = ((DATA->ADU[2]<<8)+DATA->ADU[3]);  // адрес регистра
  if(reg_addr<MODBUS_REG_MAX)
  {
    R[reg_addr] = (DATA->ADU[4]<<8)+DATA->ADU[5];
    DATA->txlen = DATA->rxlen-1;
  }
  else
  {
    modbus_EX(DATA,MBUS_EX_ILLEGAL_DATA_ADDRESS);
  }
}

void modbus_F16(MBUS_UART_DATA *DATA, uint16_t *R)
{
  // Modbus Function 16(0x10) Write multiple registers
  // ADU[0] - device address
  // ADU[1] - function code
  // ADU[2,3] - starting address 0x0000-0xFFFF
  // ADU[4,5] - number of registers 0x0001-0x007B
  // ADU[6] - byte count
  // ADU[7] - start data blocks
  // ответ
  // ADU[0] - device address
  // ADU[1] - function code
  // ADU[2,3] - starting address 0x0000-0xFFFF
  // ADU[4,5] - number of registers 0x0001-0x007B
  uint8_t i,j;
  uint16_t reg_start,reg_count;
  reg_start = ((DATA->ADU[2]<<8)+DATA->ADU[3]); // начачальный регистр
  reg_count = ((DATA->ADU[4]<<8)+DATA->ADU[5]); // кол-во регистров
  j = 7;                                        // смещение на начало блока данных
  if((reg_start<MODBUS_REG_MAX)&&((reg_start+reg_count)<MODBUS_REG_MAX+1))
  {
    for(i=0;i<reg_count;i++)
    {
      R[reg_start+i] = (DATA->ADU[j]<<8)+DATA->ADU[j+1];
      j = j + 2;
    }
    DATA->txlen = DATA->rxlen-1;
  }
  else
  {
    modbus_EX(DATA,MBUS_EX_ILLEGAL_DATA_ADDRESS);
  }
}

void modbus_EX(MBUS_UART_DATA *DATA,uint8_t exception)
{
  // Modbus Exception
  // ответ
  // ADU[0] - device address
  // ADU[1] - return function code and Modbus error mask(0x80)
  // ADU[2] - exception code
  // ADU[3,4] - CRC16
  // фиксированна€ длина 5 байт
  DATA->ADU[1] = DATA->ADU[1]|MBUS_F_ERROR_MASK;
  DATA->ADU[2] = exception;
  DATA->txlen = 5;
}

uint16_t CRC16(uint8_t *bufPtr, uint8_t len)
{
  uint16_t res=0;
  uint8_t i;
  if(bufPtr)
  {
    res=0xffffU;
    for(; len>0; len--)
    {
      res=(uint16_t)((res/256U)*256U+((res%256U)^(*bufPtr++)));
      for(i=0; i<8; i++)
      {
        if((res&0x01)==1) res=(uint16_t)((res>>1)^0xa001U);
        else res>>=1;
      }
    }
  }
  return res;
}
