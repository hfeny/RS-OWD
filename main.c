/* */ 
#include "stm8s.h"
#include "stm8s_uart1.h"
#include "main.h"

MBUS_UART_DATA UART_DATA;
uint16_t MODBUS_REG[MODBUS_REG_MAX];
uint8_t DEVICE_ADDR;

volatile uint16_t delay;

/*
static inline void delay_us(unsigned short __us)
{
  do{
    __no_operation();
    __no_operation();
    __no_operation();
    __us--;
  }while(__us);
}
*/

/* Private functions */
void SetupTIM1(void)
{
  TIM1->PSCRH = 0x00;
  TIM1->PSCRL = 0x00;
  
}

void SetupTIM2(void)
{
  TIM2->PSCR = 0x04;                                            // prescaler = 16
  TIM2->ARRH = 0x03;                                            // tick = 1ms 0x03DB
  TIM2->ARRL = 0xDB;                                            // tick = 0,1ms 0x0060
  TIM2->IER |= TIM2_IER_UIE;                                    // enable update interrupt
}

void SetupTIM4(void)
{                                                               // tick = 0,25ms
  TIM4->PSCR = 0x07;                                            // 2^0x07 = 128
  TIM4->ARR = 0x1E;                                             // counter = 30
  TIM4->IER |= (uint8_t)TIM4_IT_UPDATE;                         // enable update interrupt
  TIM4->CR1 |= (uint8_t)TIM4_CR1_CEN;                           // enable timer
}

void SetupUART(uint32_t UART_BAUD)
{
  UART1->CR1 = (uint8_t)UART1_WORDLENGTH_8D | (uint8_t)UART1_PARITY_NO;
  UART1->CR3 = (uint8_t)UART1_STOPBITS_1;
  UART1->BRR2 |= (uint8_t)UART_DIVM;
  UART1->BRR1 |= (uint8_t)UART_DIVL;
  UART1->CR2 |= (uint8_t)(UART1_CR2_TEN | UART1_CR2_REN);       // enable UART TX RX
  UART1->CR3 &= (uint8_t)(~UART1_CR3_CKEN);                     // disable SCLK
  UART1->CR1 &= (uint8_t)(~UART1_CR1_UARTD);                    // UART1 Enable
  // Setup TXE pin
  GPIOD->DDR |= GPIO_PIN_4;                                     // set pin to output
  GPIOD->CR1 |= GPIO_PIN_4;                                     // push-pull mode
  GPIOD->CR2 &= (uint8_t)~GPIO_PIN_4;                           // low speed mode
  GPIOD->ODR &= (uint8_t)~GPIO_PIN_4;                           // pin to low
}

void SetupADC()
{
  GPIOD->DDR &= (uint8_t)~GPIO_PIN_2 | (uint8_t)~GPIO_PIN_3;    // AIN3,AIN4 float input
  GPIOD->CR1 &= (uint8_t)~GPIO_PIN_2 | (uint8_t)~GPIO_PIN_3;
  ADC1->CR1 |= (uint8_t)(ADC1_PRESSEL_FCPU_D8);                 // Fmaster/8 (16MHz / 8 = 2MHz)
  ADC1->CR2 |= ADC1_CR2_ALIGN;                                  // right align
  ADC1->CR3 &= (uint8_t)(~ADC1_CR3_DBUF);                       // disable data buffer
  ADC1->TDRH |= 0x00;                                           // disable schmitt trigger on AIN3,AIN4 channel
  ADC1->TDRL |= (uint8_t)0x18;                                  // disable schmitt trigger on AIN3,AIN4 channel
  ADC1->CSR |= FIRST_ADC_CH;                                    // select channel first chanel
  ADC1->CSR |= ADC1_CSR_EOCIE;                                  // enable interrupt if end of conversion
  ADC1->CR1 |= ADC1_CR1_ADON;                                   // ADC1 On
}

// отправка данных modbus
void RS485TX(MBUS_UART_DATA *uart)
{
  if((uart->txlen > 0)&(uart->txcnt == 0))
  {
    GPIO_WriteHigh(GPIOD,GPIO_PIN_4);                           // ADM485 TXE to high
    UART1->CR2 &= (uint8_t)~0x20;                               // RX Int disable,by clear RIEN bit on CR2
    UART1->CR2 |= 0x40;                                         // TX Int enable,by set TCIEN bit on CR2
  }
}

void eeprom_write(uint8_t EEPROM_ADDR,uint8_t value)
{
  uint8_t *address = (uint8_t *)(EEPROM_BASE+EEPROM_ADDR);
  if(EEPROM_ADDR>EEPROM_END_ADDRESS)
    EEPROM_ADDR = 0x00;
  // включаем запись в EEPROM
  if(!(FLASH->IAPSR & FLASH_IAPSR_DUL))
  {
    FLASH->DUKR = 0xae;
    FLASH->DUKR = 0x56;
  }
  // пишем данные предварительно провер€€ что они отличаютс€ от имеющихс€
  if(*address != value)
    *address = value;
  // отключаем запись в EEPROM
  FLASH->DUKR &= (uint8_t)(FLASH_IAPSR_DUL);
}

uint8_t eeprom_read(uint8_t EEPROM_ADDR)
{
  if(EEPROM_ADDR>EEPROM_END_ADDRESS)
    EEPROM_ADDR = 0x00;
  uint8_t *address = (uint8_t *)(EEPROM_BASE+EEPROM_ADDR);
  return *address;
}

void device_setup(void)
{
  uint8_t msb,lsb;
  msb = MODBUS_REG[0]>>8;
  lsb = MODBUS_REG[0]&0x00FF;
  if((msb == EEPROM_END_ADDRESS)&&(lsb != DEVICE_ADDR))
    eeprom_write(msb,lsb);
  else
    MODBUS_REG[0] = (EEPROM_END_ADDRESS<<8)+eeprom_read(EEPROM_END_ADDRESS);
}

void delay_ms(uint16_t time)
{
  delay = time;                                                 // TIM2 tick = 0,1ms
  TIM2->CR1 |= TIM2_CR1_CEN;                                    // enable timer
  while(delay != 0);
  TIM2->CR1 &= ~TIM2_CR1_CEN;                                   // disable timer
}

uint16_t CAPmeter(void)
{
  //volatile uint16_t RCtime;
  //RCtime = 0;
  uint16_t CAP = 0;
  //
  GPIOC->DDR &= (uint8_t)~GPIO_PIN_4;                           // PC4 float input
  GPIOC->CR1 &= (uint8_t)~GPIO_PIN_4;
  
  /**/
  GPIOA->DDR |= GPIO_PIN_3;                                     // set PA3 pin to output
  GPIOA->CR1 &= (uint8_t)~GPIO_PIN_3;                           // open-drain mode
  GPIOA->CR2 &= (uint8_t)~GPIO_PIN_3;                           // low speed mode
  GPIOA->ODR &= (uint8_t)~GPIO_PIN_3;                           // pin to low
  /**/
  TIM1->CNTRH = 0x00;
  TIM1->CNTRL = 0x00;
  delay_ms(10);                                                 // wait for discharge
  //
  GPIOA->DDR ^= GPIO_PIN_3;                                     // set pin to float input
  TIM1->CR1 = 0x01;
  /**/
  while(!(GPIOA->IDR & GPIO_PIN_3))                             // wait for input high
  {
  }
  /**/
  GPIOA->DDR ^= GPIO_PIN_3;                                     // set pin to output
  TIM1->CR1 = 0x00;
  CAP = (unsigned int)TIM1->CNTRL;
  CAP |= (unsigned int)TIM1->CNTRH<<8;
  CAP = CAP>>4;
  return CAP;
}

/* main function */
void main(void)
{
  // Setup system clock
  CLK->ICKR = 0x00;                                             // Reset the Internal Clock Register
  CLK->ICKR |= CLK_ICKR_HSIEN;                                  // Enable the HSI
  CLK->CKDIVR = 0x00;                                           // Set divider to 1. Sysclock = 16mHz
  //
  SetupTIM1();
  SetupTIM2();
  SetupTIM4();
  SetupUART(RS485_BAUD);
  SetupADC();
  //
  UART_DATA.delay = T35;                                        // 4ms for 9600baud
  //
  DEVICE_ADDR = eeprom_read(EEPROM_END_ADDRESS);
  if(DEVICE_ADDR == 0)
    DEVICE_ADDR = MBUS_ADDRESS_DEFAULT;
  //
  __enable_interrupt();
    
  while (1)
  {
    if(UART_DATA.rxgap == 1)
    {
      modbus(&UART_DATA,DEVICE_ADDR,MODBUS_REG);                //modbus parsing function
      RS485TX(&UART_DATA);                                      //send modbus ADU
    }
      ADC1->CR1 |= ADC1_CR1_ADON;                               // start ADC conversion
      MODBUS_REG[6] = CAPmeter();

    //device_setup();
    /*
    if(MODBUS_REG[2]==0x01)
    {
      MODBUS_REG[2] = 0x00;                                     // debug way
      ADC1->CR1 |= ADC1_CR1_ADON;                               // start ADC conversion
      MODBUS_REG[6] = CAPmeter();
    }
    */
  }
  
}

INTERRUPT_HANDLER(ADC1_handler,ITC_IRQ_ADC1)
{
  unsigned char ADC_IN;
  unsigned short ADC_Value;
  if(ADC1->CSR & (uint8_t)ADC1_CSR_EOC)
  {
    ADC1->CSR &= (uint8_t)(~ADC1_CSR_EOC);                      // clear EOC flag
  }
  else
  {
    ADC1->CSR &= (uint8_t)(~ADC1_CSR_AWD);                      // clear AWD flag
  }
  ADC_IN = ADC1->CSR&0x0f;
  ADC1->CR1 &= (uint8_t)(~ADC1_CR1_ADON);                       // disable ADC
  // read LSB first if right aligment use
  ADC_Value = (unsigned int)ADC1->DRL;
  ADC_Value |= (unsigned int)ADC1->DRH<<8;
  //
  MODBUS_REG[ADC_IN] = ADC_Value;
  if(ADC_IN < LAST_ADC_CH)
  {
    ADC_IN++;
    ADC1->CSR &= 0xf0;
    ADC1->CSR |= ADC_IN;
    ADC1->CR1 |= ADC1_CR1_ADON;
    __no_operation();
    ADC1->CR1 |= ADC1_CR1_ADON;
  }
  else
  {
    ADC1->CSR &= 0xf0;
    ADC1->CSR |= FIRST_ADC_CH;
  }
  /*
  if(ADC_IN==0x03)
  {
    MODBUS_REG[ADC_IN] = ADC_Value;
    ADC1->CSR &= 0xf0;
    ADC1->CSR |= 0x04;
    ADC1->CR1 |= ADC1_CR1_ADON;
    __no_operation();
    ADC1->CR1 |= ADC1_CR1_ADON;
  }
  else
  {
    MODBUS_REG[ADC_IN] = ADC_Value;
    ADC1->CSR &= 0xf0;
    ADC1->CSR |= 0x03;
  }
  */
}

INTERRUPT_HANDLER(TIM2_handler,ITC_IRQ_TIM2_OVF)
{
  TIM2->SR1 = (uint8_t)~TIM2_SR1_UIF;
  delay--;
}

INTERRUPT_HANDLER(TIM4_handler,ITC_IRQ_TIM4_OVF)                // handler defined in stm8s_itc.h
{
  // TIM4 clear interrupt pending bits
  TIM4->SR1 &= (uint8_t)~TIM4_SR1_UIF;
  // write reverse RX_Led pin
  GPIO_WriteReverse(GPIOD,GPIO_PIN_3);
  if(UART_DATA.rxtimer++>=UART_DATA.delay)
  {
    // off RX_Led
    GPIO_WriteLow(GPIOD,GPIO_PIN_3);                            // rx-led
    UART_DATA.rxlen=UART_DATA.rxcnt;
    if(UART_DATA.rxcnt>4) UART_DATA.rxgap=1;
    UART_DATA.rxcnt=0;
    // enable UART1 RXNE interrupt
    UART1_ITConfig(UART1_IT_RXNE_OR,ENABLE);
    // disable TIM4
    TIM4->CR1 &= (uint8_t)(~TIM4_CR1_CEN);
  }
  else
    UART_DATA.rxgap=0;
}

INTERRUPT_HANDLER(UART1_RX,ITC_IRQ_UART1_RX)                    // handler defined in stm8s_itc.h
{
  if(UART1->SR & UART1_IT_RXNE)                                 // check UART1_RXNE int flag
  {
    UART1->SR &= (unsigned int)(~UART1_SR_RXNE);                // clear UART1_RXNE int flag
    UART_DATA.rxtimer = 0;
    UART_DATA.ADU[UART_DATA.rxcnt++] = UART1->DR;               // get UART1 RX data
    TIM4->CR1 |= TIM4_CR1_CEN;                                  // enable TIM4
  }
}

INTERRUPT_HANDLER(UART1_TX,ITC_IRQ_UART1_TX)                    // handler defined in stm8s_itc.h
{
  if(UART1->SR & UART1_IT_TC)                                   // check UART1_TC int flag
  {
    if(UART_DATA.txcnt < UART_DATA.txlen)
    {
      UART1->DR = UART_DATA.ADU[UART_DATA.txcnt++];
    }
    else
    {
      UART_DATA.txlen = 0;
      GPIOD->ODR &= (unsigned int)(~GPIO_PIN_4);                // ADM485 TXE to low
      UART1->CR2 &= (uint8_t)~0x40;                             // TX Int disable,by clear TCIEN bit on CR2
      UART1->CR2 |= 0x20;                                       // RX Int enable,by set RIEN bit on CR2
      //TIM4->IER |= (uint8_t)TIM4_IT_UPDATE;                   // TIM4 int enable
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file, u32 line)
{ 
  while (1)
  {
  }
}
#endif
