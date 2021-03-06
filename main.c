#include "stm32f4xx.h"
#include "system_timetick.h"

#define I2C_TIMEOUT_MAX         ((uint32_t)(10 * 5000))
#define DMALENGHT             4
#define BUFFER_EEPROM_WRITE             80

void init_I2C1(void);
void usart_init(void);
//i2c have 256pages, each page 32bytes
//write bytes to eeprom in one pages
uint8_t Write_24Cxx(uint8_t AddrSlave, uint8_t *txbuffData,  uint16_t write_address, uint32_t  NumberByteWrite );
// read bytes from eeprom from read_address to read_address+Number...
uint8_t Read_24Cxx(uint8_t AddrSlave, uint8_t *rxbuffData, uint16_t read_address, uint32_t  NumberByteRead );
void delay_01ms(uint16_t period);
// tuong duong strlen trong c
uint32_t strlenbuff(uint8_t *str);
//
uint8_t Write_Page(uint8_t AddrSlave, uint8_t *txbuffData, uint16_t write_address, uint32_t NumberByteWrite);
uint16_t ParseCharacterFromBuffer(void);
void strcopydata(uint8_t *aTxBuffer2,uint8_t *BufferEEPROMWrite, uint32_t lenbuff);
void ResetBuffer(uint8_t *BufferEEPROMWrite,uint32_t lenght);
//rxbuff cho DMA size 4
uint8_t RxDMA[DMALENGHT];
uint8_t BufferEEPROMWrite[BUFFER_EEPROM_WRITE];
uint16_t  index = 0;
uint8_t FlagCheckFullRxDma=0;
uint8_t FlagCheckResendTxDMA=0;
DMA_InitTypeDef   DMA_InitStructure;
//
int main(void)
{
  /* Enable SysTick at 10ms interrupt */
  SysTick_Config(SystemCoreClock/100);
  init_I2C1();
  usart_init();
  uint16_t address=0;
  uint32_t lastcharacter=0;
  uint32_t lenbuff=0;
  uint8_t  modeWriteOrRead=0;
   while(1)
     {
        if(FlagCheckFullRxDma==1)
        {
          //format bufferrx xxxxRDataaaaaaaaa0000****A
          // voi xxxx:adress 0-8192
          //R=0: mode read
          // R=1: mode write
          //Dataaaaaaa :data need to write
          // 0000: waste byte
          // **** : End character to recoginze transmit complete
          // A: number of waste byte (example 0000 -> A=4)
          lenbuff=strlenbuff(BufferEEPROMWrite);
        // call function split character from BufferEEPROMWrite
          address=ParseCharacterFromBuffer(); // calculate address
          lastcharacter=BufferEEPROMWrite[lenbuff-1]-'0';//return number of byte rac
          modeWriteOrRead=BufferEEPROMWrite[4]-'0';// considerate mode
          lenbuff=lenbuff - 4 - 1 - lastcharacter - 4 - 1;//lenght of data
          if (modeWriteOrRead==1)
          {
          	uint8_t aTxBuffer2[BUFFER_EEPROM_WRITE];
          	strcopydata(aTxBuffer2,BufferEEPROMWrite,lenbuff);
            Write_Page(0xA0,aTxBuffer2,address,lenbuff);
            GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
            delay_01ms(1000);
          }
          else
          {
          	uint32_t lenbuffbyte = (BufferEEPROMWrite[5]-'0')*1000 + (BufferEEPROMWrite[6]-'0')*100 +(BufferEEPROMWrite[7]-'0')*10+(BufferEEPROMWrite[8]-'0');
           	uint8_t aRxBuffer2[BUFFER_EEPROM_WRITE];
              if (Read_24Cxx(0xA0,aRxBuffer2,address,lenbuffbyte)==0xFF)
              {
                while(1)
                {
                  GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
                  delay_01ms(10000);
                }
              }
              delay_01ms(1000);
              GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
              if (FlagCheckResendTxDMA==0)
              {
                DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
                DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aRxBuffer2;
                DMA_InitStructure.DMA_BufferSize = lenbuffbyte;
                DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
                DMA_Init(DMA1_Stream4, &DMA_InitStructure);
                DMA_Cmd(DMA1_Stream4, ENABLE);
              }
              else
              {
                DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
                DMA1_Stream4->NDTR = lenbuffbyte;
                DMA_Cmd(DMA1_Stream4, ENABLE);
              }
              FlagCheckResendTxDMA=1;
              delay_01ms(5000);
          }
              index=0;
              ResetBuffer(BufferEEPROMWrite,lenbuff+10+lastcharacter);
              FlagCheckFullRxDma=0;
        }
        else
        {
          GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
          delay_01ms(5000);
        }
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

uint8_t Write_24Cxx(uint8_t AddrSlave, uint8_t *txbuffData, uint16_t write_address,uint32_t  NumberByteWrite )
{
  uint32_t timeout = I2C_TIMEOUT_MAX;

  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
      if((timeout--) == 0) return 0xFF;
    }

  I2C_GenerateSTART(I2C1, ENABLE);
   /* Test on I2C1 EV5, Start trnsmitted successfully and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
   {
      if ((timeout--) == 0) return 0xFF;
   }
   //send address slave and mode transmitter to write data
  I2C_Send7bitAddress(I2C1, AddrSlave, I2C_Direction_Transmitter);
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
   {
      if ((timeout--) == 0) return 0xFF;
   }

  /* Send I2C1 location address LSB */
  I2C_SendData(I2C1, (uint8_t)((write_address & 0xFF00) >> 8));
    /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
    {
      if ((timeout--) == 0) return 0xFF;
    }

  I2C_SendData(I2C1, (uint8_t)(write_address & 0x00FF));
  /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
    {
      if ((timeout--) == 0) return 0xFF;
    }
    // function send multi data create by thanh
  while(NumberByteWrite)
    {
      if(NumberByteWrite<2)
        {
          I2C_SendData(I2C1,  *txbuffData);
    /*   Test on I2C1 EV8 and clear it */
          timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
          while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
          {
            if ((timeout--) == 0) return 0xFF;
        }
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        /* Send I2C1 STOP Condition */
        I2C_GenerateSTOP(I2C1, ENABLE);
        /* If operation is OK, return 0 */
        return 0;
       }
      else
       {
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
      }
      NumberByteWrite--;
      }
      I2C_GenerateSTOP(I2C1, ENABLE);

    return 0;
}

uint8_t Write_Page(uint8_t AddrSlave, uint8_t *txbuffData, uint16_t write_address, uint32_t NumberByteWrite)
{
  uint32_t Page = NumberByteWrite/32;
  uint32_t SinglePage = NumberByteWrite%32;
  uint32_t AddressAligned = write_address%32;
  uint32_t RemainAddressAligned = 32 - AddressAligned;

  if (AddressAligned==0)
  {

      while(Page>0)
      {
        if ( Write_24Cxx(AddrSlave,txbuffData,write_address,32)==0xFF)
          {
            while(1)
            {
              GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
              delay_01ms(10000);
            }
          }
          delay_01ms(100);
    			txbuffData+=32;
          write_address+=32;

          Page--;
      }
      // else
      if (Page==0)
        {
          if (Write_24Cxx(AddrSlave,txbuffData,write_address,SinglePage)==0xFF)
          {
           while(1)
           {
             GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
             delay_01ms(10000);
           }
          }
        }
				return 0;
  }
  //not aligned
  else
  {
    if (SinglePage <RemainAddressAligned)
    {
      if ( Write_24Cxx(AddrSlave,txbuffData,write_address,SinglePage)==0xFF)
       {
         while(1)
         {
           GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
           delay_01ms(10000);
         }
       }
    }
    else{
      if ( Write_24Cxx(AddrSlave,txbuffData,write_address,RemainAddressAligned)==0xFF)
       {
         while(1)
         {
           GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
           delay_01ms(10000);
         }
       }
       delay_01ms(100);
       txbuffData+=RemainAddressAligned;
       write_address+=RemainAddressAligned;
       Page=(NumberByteWrite - RemainAddressAligned)/32;
       SinglePage=(NumberByteWrite - RemainAddressAligned)%32;

      while(Page>0)
      {
        if ( Write_24Cxx(AddrSlave,txbuffData,write_address,32)==0xFF)
          {
            while(1)
            {
              GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
              delay_01ms(10000);
            }
          }
          delay_01ms(100);
          txbuffData+=32;
          write_address+=32;
          Page--;
      }
      // else
      if (Page==0)
        {
          if (Write_24Cxx(AddrSlave,txbuffData,write_address,SinglePage)==0xFF)
          {
           while(1)
           {
             GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
             delay_01ms(10000);
           }
          }
        }

    }
  }
	return 0;
}


uint8_t Read_24Cxx(uint8_t AddrSlave, uint8_t *rxbuffData,  uint16_t read_address, uint32_t  NumberByteRead )
{
  uint32_t timeout = I2C_TIMEOUT_MAX;

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
  I2C_SendData(I2C1, (uint8_t)((read_address & 0xFF00) >> 8));

    /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
            /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0)
          return 0xFF;
    }
  I2C_SendData(I2C1, (uint8_t)((read_address & 0x00FF)));
  /* Test on I2C1 EV8 and clear it */
  timeout = I2C_TIMEOUT_MAX; /* Initialize timeout value */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
            /* If the timeout delay is exeeded, exit with error code */
      if ((timeout--) == 0)
          return 0xFF;
    }
 //feature thanh create
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
//receive multi byte
  while(NumberByteRead)
  {
    //receive one byte
    if(NumberByteRead<2)
    {
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
    else
    {
      timeout = I2C_TIMEOUT_MAX;
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
      {
            /* If the timeout delay is exeeded, exit with error code */
          if ((timeout--) == 0) return 0xFF;
      }
      *rxbuffData= I2C_ReceiveData(I2C1);
      rxbuffData++;
    }
    NumberByteRead--;

  }
  return 0;
}

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

uint32_t strlenbuff(uint8_t *str)
{
  uint32_t counter=0;
  while(*str!='\0')
    {
      counter++;
      str++;
    }
  return counter;
}

void usart_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  /* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* Connect UART4 pins to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* USARTx configured as follow:
    - BaudRate = 115200 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
  /* Enable USART */
  USART_Cmd(UART4, ENABLE);
  /* Enable UART4 DMA */
  /* DMA1 Stream2 Channel4 for USART4 Tx configuration */
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
  /* DMA1 Stream2 Channel4 for USART4 Tx configuration */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
   // DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aRxBuffer2;
  //DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aTxBuffer2;
  // DMA_InitStructure.DMA_BufferSize = BUFFER_EEPROM_WRITE;
  // DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  // DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
// viet them RX UART
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxDMA;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = DMALENGHT;
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream2, ENABLE);
  /* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
}

void DMA1_Stream2_IRQHandler(void)
{
  uint8_t i;
  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
  for(i=0; i<DMALENGHT; i++)
    {
      BufferEEPROMWrite[index + i] = RxDMA[i];
      if ((RxDMA[i]=='*') && (BufferEEPROMWrite[index + i-1]=='*') && (BufferEEPROMWrite[index + i-2]=='*' )&& (BufferEEPROMWrite[index + i-3]=='*' ))
      {
        FlagCheckFullRxDma=1;
      }
    }
  index = index + DMALENGHT;
  DMA_Cmd(DMA1_Stream2, ENABLE);
}

uint16_t ParseCharacterFromBuffer(void)
{
  //variable global compute lenght buffer
  uint16_t temp;
  temp=( (BufferEEPROMWrite[0]-'0')*1000 + (BufferEEPROMWrite[1]-'0')*100+ (BufferEEPROMWrite[2]-'0')*10+(BufferEEPROMWrite[3]-'0'));
  return temp;
}

void strcopydata(uint8_t *aTxBuffer2,uint8_t *BufferEEPROMWrite, uint32_t lenbuff)
{
  int i;
  for(i=0;i<lenbuff;i++)
    {
      aTxBuffer2[i]=BufferEEPROMWrite[i+5];
    }
}
void ResetBuffer(uint8_t *BufferEEPROMWrite,uint32_t lenght)
{
  uint8_t i;
  for (i=0;i<lenght;i++)
    BufferEEPROMWrite[i]='\0';
}
