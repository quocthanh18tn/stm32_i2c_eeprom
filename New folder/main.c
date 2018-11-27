#include "stm32f4xx.h"
#include "system_timetick.h"

#define I2C_TIMEOUT_MAX         ((uint32_t)(10 * 5000))

void init_I2C1(void);
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
TestStatus FlagCompare=FAILED;

uint8_t Write_24Cxx(uint8_t AddrSlave, uint8_t *txbuffData,  uint16_t write_address );
uint8_t Read_24Cxx(uint8_t AddrSlave, uint16_t read_address,uint8_t *rxbuffData );
uint8_t aTxBuffer2[9] = "ab1";
uint8_t aRxBuffer2[9];
void delay_01ms(uint16_t period){

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM6->PSC = 8399;   // clk = SystemCoreClock /2 /(PSC+1) = 10KHz
    TIM6->ARR = period-1;
    TIM6->CNT = 0;
    TIM6->EGR = 1;    // update registers;

    TIM6->SR  = 0;    // clear overflow flag
    TIM6->CR1 = 1;    // enable Timer6

    while (!TIM6->SR);

    TIM6->CR1 = 0;    // stop Timer6
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}



int main(void)
{
  /* Enable SysTick at 10ms interrupt */
  SysTick_Config(SystemCoreClock/100);
  init_I2C1();
	//GPIO_ResetBits(GPIOD,GPIO_Pin_12);
  GPIO_SetBits(GPIOD,GPIO_Pin_12);
  GPIO_SetBits(GPIOD,GPIO_Pin_13);
  GPIO_SetBits(GPIOD,GPIO_Pin_14);
  GPIO_SetBits(GPIOD,GPIO_Pin_15);
  delay_01ms(10000);
  if ( Write_24Cxx(0xA0,aTxBuffer2,0x00)==0xFF)
  {
      while(1)
      {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
        delay_01ms(10000);
      }
  }
        delay_01ms(100);
  if (Read_24Cxx(0xA0,0x00,aRxBuffer2)==0xFF)
  {

     while(1)
      {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
        delay_01ms(10000);
      }
  }
  delay_01ms(1000);
  FlagCompare=Buffercmp(aRxBuffer2,aTxBuffer2,9);
   if (FlagCompare==PASSED)
  {
      while(1)
      {
        GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
        delay_01ms(10000);
      }
  }
  else if (FlagCompare!=PASSED)
  {
    while(1)
    {
      GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
        delay_01ms(10000);
    }

  }
  while(1){

  }
}


void init_I2C1(void){

  GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as I2C1SDA and I2C1SCL
  GPIO_InitTypeDef GPIO_Output;     // For some debugging LEDs
  I2C_InitTypeDef I2C_InitStruct; // this is for the I2C1 initilization

  /* enable APB1 peripheral clock for I2C1*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  /* enable the peripheral clock for the pins used by
   PB6 for I2C SCL and PB9 for I2C1_SDL*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* This sequence sets up the I2C1SDA and I2C1SCL pins
   * so they work correctly with the I2C1 peripheral
   */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; // Pins 6(I2C1_SCL) and 9(I2C1_SDA)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;// this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;// this defines the output type as open drain
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup resistors on the IO pins
  GPIO_Init(GPIOB,&GPIO_InitStruct);// now all the values are passed to the GPIO_Init()

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

   /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_Output.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_Output.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Output.GPIO_OType = GPIO_OType_PP;
  GPIO_Output.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Output.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD,&GPIO_Output);

  /* The I2C1_SCL and I2C1_SDA pins are now connected to their AF
   * so that the I2C1 can take over control of the
   * pins
   */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); //
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

   /* Configure I2C1 */
  I2C_DeInit(I2C1);

        /* Enable the I2C peripheral */

        /* Set the I2C structure parameters */
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0xEE;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_ClockSpeed = 100000;
        /* Initialize the I2C peripheral w/ selected parameters */
  I2C_Init(I2C1,&I2C_InitStruct);
  I2C_Cmd(I2C1, ENABLE);
}

uint8_t Write_24Cxx(uint8_t AddrSlave, uint8_t *txbuffData, uint16_t write_address )
{
  //uint8_t NumOfPage = 0, NumOfSingle = 0, count = 0;
  //uint16_t Addr = 0;
  uint8_t upper_addr,lower_addr;
  uint32_t timeout = I2C_TIMEOUT_MAX;

  lower_addr = (uint8_t)((0x00FF)&write_address);
  upper_addr = (uint8_t)((0xFF00)&write_address>>8);

  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {

      if((timeout--) == 0) return 0xFF;
    }

	I2C_GenerateSTART(I2C1, ENABLE);

       /* Test on I2C1 EV5, Start trnsmitted successfully and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
       {

            /* If the timeout delay is exeeded, exit with error code */
          if ((timeout--) == 0)
            return 0xFF;
       }
       //send address slave and mode transmitter to write data
  I2C_Send7bitAddress(I2C1, AddrSlave, I2C_Direction_Transmitter);
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
       {
           /* If the timeout delay is exeeded, exit with error code */
          if ((timeout--) == 0)
             return 0xFF;
       }

  /* Send I2C1 location address LSB */
  I2C_SendData(I2C1, upper_addr);

    /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0)
        return 0xFF;
    }

  I2C_SendData(I2C1, lower_addr);

  /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
            /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0)
          return 0xFF;
    }

  /* Send Data */
  I2C_SendData(I2C1,  *txbuffData);

  /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
       /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0)
       return 0xFF;
  }
  txbuffData++;
  I2C_SendData(I2C1,  *txbuffData);
 	timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
       /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0)
       return 0xFF;
  }
  txbuffData++;
  I2C_SendData(I2C1,  *txbuffData);
  /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
       /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0)
       return 0xFF;
  }
  /* Send I2C1 STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);

  /* If operation is OK, return 0 */
  return 0;

}
uint8_t Read_24Cxx(uint8_t AddrSlave,  uint16_t read_address,uint8_t *rxbuffData )
{
  uint32_t timeout = I2C_TIMEOUT_MAX;
  uint8_t upper_addr,lower_addr;

  lower_addr = (uint8_t)((0x00FF)&read_address);
  upper_addr = (uint8_t)((0xFF00)&read_address>>8);

  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
      if((timeout--) == 0)
					return 0xFF;
    }


  I2C_GenerateSTART(I2C1, ENABLE);

       /* Test on I2C1 EV5, Start trnsmitted successfully and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
       {
            /* If the timeout delay is exeeded, exit with error code */
          if ((timeout--) == 0)
            return 0xFF;
       }
       //send address slave and mode transmitter to write data
  I2C_Send7bitAddress(I2C1, AddrSlave, I2C_Direction_Transmitter);

  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
       {
           /* If the timeout delay is exeeded, exit with error code */
          if ((timeout--) == 0)
					return 0xFF;
       }
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  /* Send I2C1 location address LSB */
  I2C_SendData(I2C1, upper_addr);

    /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0)
        return 0xFF;
    }

  I2C_SendData(I2C1, lower_addr);

  /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
            /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0)
          return 0xFF;
    }

//dont know
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on I2C1 EV6 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
       /* If the timeout delay is exeeded, exit with error code */
    if ((timeout--) == 0) return 0xFF;
  }

  I2C_Send7bitAddress(I2C1, AddrSlave, I2C_Direction_Receiver);

  /* Test on I2C1 EV6 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
        /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0) return 0xFF;
  }
		timeout = I2C_TIMEOUT_MAX;
	   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
        /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0) return 0xFF;
  }
  *rxbuffData= I2C_ReceiveData(I2C1);

  rxbuffData++;
  timeout = I2C_TIMEOUT_MAX;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
        /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0) return 0xFF;
  }
  *rxbuffData= I2C_ReceiveData(I2C1);

  rxbuffData++;
  /* Prepare an NACK for the next data received */
  I2C_AcknowledgeConfig(I2C1, DISABLE);

  /* Test on I2C1 EV7 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
  {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
  }
  I2C_GenerateSTOP(I2C1, ENABLE);

  /* Receive the Data */
    *rxbuffData= I2C_ReceiveData(I2C1);
    /*!< Re-Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  return 0;
}
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}
