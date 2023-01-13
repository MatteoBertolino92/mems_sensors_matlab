/**
  ******************************************************************************
  * @file    Demo/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32F10x.h"
#include "STM32vldiscovery.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct 
{
  char startsync[4];
  int timestamp;
  s16 m1x, m1y, m1z;
  s16 m2x, m2y, m2z;
  s16 a1x, a1y, a1z;
  s16 a2x, a2y, a2z;
  int wr_i_dbg;
  int empty;
} sensor_data_type;

/* Private define ------------------------------------------------------------*/
#define  LSE_FAIL_FLAG  0x80
#define  LSE_PASS_FLAG  0x100

#define LSM303D_ADD_1 0x3C
#define LSM303D_ADD_2 0x3A

#define _MAGNET
#define _ACCEL

/* Private macro -------------------------------------------------------------*/
/* Private consts ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
u32 LSE_Delay = 0;
u32 count = 0;
u32 BlinkSpeed = 0;
u32 KeyState = 0;

sensor_data_type my_data[100];
int wr_i;
static __IO uint32_t TimingDelay;
/* Private function prototypes -----------------------------------------------*/
void Delay(uint32_t nTime);
void TimingDelay_Decrement(void);
void transfer_data(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

void init_I2C1(void);
void I2C_Read(u8* pBuffer, u8 ReadAddr, u8 DevAddr, u8 NumByteToRead);
void I2C_Write(u8* pBuffer, u8 WriteAddr, u8 DevAddr, u8 NumByteToWrite);
void init_USART1(void);
void TIM1_Interrupt_Configuration();
void TIM1_Configuration();

int main(void)
{
  int i;
  u8 mychar = 'A';
  char mystring[100];
  
  /* Enable GPIOx Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  /* Initialise LEDs LD3&LD4, both off */
  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDInit(LED4);
  
  STM32vldiscovery_LEDOn(LED3);
  STM32vldiscovery_LEDOn(LED4);
  
  /* Initialise USER Button */
  STM32vldiscovery_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 
  
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }

  wr_i = 0;
  for (i=0; i<10; i++)
  {
    my_data[i].empty = 1;
  }
  
  init_I2C1();
  init_USART1();
  TIM1_Interrupt_Configuration();
  TIM1_Configuration();
  
#ifdef _MAGNET  
  mychar = 0x70; // 01110000 enable magnetometer (high resolution, 50 Hz)
  I2C_Write(&mychar, 0x24, LSM303D_ADD_1, 1);  // CTRL0
  I2C_Write(&mychar, 0x24, LSM303D_ADD_2, 1);  // CTRL0
  mychar = 0x00; // 00000000 +/-2 2 gauss full scale
  I2C_Write(&mychar, 0x25, LSM303D_ADD_1, 1);  // CTRL6
  I2C_Write(&mychar, 0x25, LSM303D_ADD_2, 1);  // CTRL6
  mychar = 0x00; // 00000000 enable magnetomter (continuous conversion)
  I2C_Write(&mychar, 0x26, LSM303D_ADD_1, 1);  // CTRL7
  I2C_Write(&mychar, 0x26, LSM303D_ADD_2, 1);  // CTRL7
#endif
  
#ifdef _ACCEL
  mychar =  0x57; // 01010111 enable accelerometer (50 Hz)
  I2C_Write(&mychar, 0x20, LSM303D_ADD_1, 1);  // CTRL0
  I2C_Write(&mychar, 0x20, LSM303D_ADD_2, 1);  // CTRL0   
#endif
  
  
  
  while (1)
  {
    if (!my_data[0].empty)
    {
//#ifdef _MAGNET
//    sprintf(mystring, "%d M1: %8d  %8d  %8d  ", my_data[0].timestamp, my_data[0].m1x, my_data[0].m1y, my_data[0].m1z);  
//    for (i=0; mystring[i]!='\0'; i++)
//    {
//      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
//      USART_SendData(USART1, mystring[i]);
//    }
//    
//    sprintf(mystring, "M2: %8d  %8d  %8d\n", my_data[0].m2x, my_data[0].m2y, my_data[0].m2z);    
//    for (i=0; mystring[i]!='\0'; i++)
//    {
//      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
//      USART_SendData(USART1, mystring[i]);
//    }
//#endif
//#ifdef _ACCEL
//    sprintf(mystring, "%d A1: %8.3f  %8.3f  %8.3f  ", my_data[0].timestamp, my_data[0].a1x, my_data[0].a1y, my_data[0].a1z);
//    for (i=0; mystring[i]!='\0'; i++)
//    {
//      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
//      USART_SendData(USART1, mystring[i]);
//    }   
//    
//    sprintf(mystring, "A2: %8.3f  %8.3f  %8.3f\n", my_data[0].a2x, my_data[0].a2y, my_data[0].a2z);
//    for (i=0; mystring[i]!='\0'; i++)
//    {
//      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
//      USART_SendData(USART1, mystring[i]);
//    }   
//    
//#endif

    strcpy(my_data[0].startsync, "STP");
    my_data[0].wr_i_dbg = wr_i;
    for (i=0; i<sizeof(sensor_data_type)-4; i++)
      {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
        USART_SendData(USART1, (char) *((my_data[0].startsync)+i));
      }     
    
    
    __disable_irq();
    for (i=1; i<wr_i; i++)
      my_data[i-1] = my_data[i];
    my_data[i-1].empty=1;
    wr_i--;
    __enable_irq();
    }
    
  }
  
  /* main while */

}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void init_I2C1(void)
{
I2C_InitTypeDef my_i2c;
GPIO_InitTypeDef my_gpio;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

/* Configure I2C pins: SCL and SDA */
my_gpio.GPIO_Pin =  GPIO_Pin_6| GPIO_Pin_7;
my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
my_gpio.GPIO_Mode = GPIO_Mode_AF_OD;
GPIO_Init(GPIOB, &my_gpio);


/* I2C1 Reset */
RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 , ENABLE);
RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1 , DISABLE);

I2C_StructInit(&my_i2c);
/* I2C configuration */
my_i2c.I2C_Mode = I2C_Mode_I2C;
my_i2c.I2C_DutyCycle = I2C_DutyCycle_2;
//my_i2c.I2C_OwnAddress1 = 0;
my_i2c.I2C_Ack = I2C_Ack_Enable;
my_i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
my_i2c.I2C_ClockSpeed = 150000; 

/* Apply I2C configuration after enabling it */
I2C_Init(I2C1, &my_i2c);

/* I2C Peripheral Enable */
I2C_Cmd(I2C1, ENABLE);

my_gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; // SA01 & SA02
my_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &my_gpio);
GPIO_SetBits(GPIOB, GPIO_Pin_4);            // SA02: 1 => address = 0x3A
GPIO_ResetBits(GPIOB, GPIO_Pin_5);          // SA01: 1 => address = 0x3C

my_gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // VDD
my_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &my_gpio);
GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
}


void init_USART1(void)
{
GPIO_InitTypeDef my_gpio;
USART_InitTypeDef my_usart;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

my_gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // SPI1_SS1
my_gpio.GPIO_Mode = GPIO_Mode_AF_PP;
my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &my_gpio);

USART_DeInit(USART1);
USART_StructInit(&my_usart);
my_usart.USART_BaudRate = 115200;
USART_Init(USART1, &my_usart);

USART_Cmd(USART1, ENABLE);
 
}

void I2C_Read(u8* pBuffer, u8 ReadAddr, u8 DevAddr, u8 NumByteToRead)
{
  __disable_irq();
    /* While the bus is busy */
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send LPS001DL address for write */
  I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);

  if(NumByteToRead>1)
  {
    ReadAddr += 0x80; //the MSB bit of address is set to 1 to allow incremental address
  }

  /* Send the LPS001DL's internal address to write to */
  I2C_SendData(I2C1, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send START condition a second time */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send LPS001DL address for read */
  I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the LPS001DL */
      *pBuffer = I2C_ReceiveData(I2C1);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  __enable_irq();

}

void I2C_Write(u8* pBuffer, u8 WriteAddr, u8 DevAddr, u8 NumByteToWrite)
{
  __disable_irq();    
  /* Send STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
  
  /* Send STLM75 address for write */
  I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Send the STLM75's internal address to write to */
  I2C_SendData(I2C1, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  
  while(NumByteToWrite)
  {
    /* Send the byte to be written */
    I2C_SendData(I2C1, *pBuffer);
    
    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    pBuffer++;
    NumByteToWrite--;
  }
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
  __enable_irq();
  
}

void TIM1_Configuration()
{
    TIM_TimeBaseInitTypeDef timer_1;
    
    /* Enable timer clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 
    TIM_TimeBaseStructInit (&timer_1);
    timer_1.TIM_ClockDivision=TIM_CKD_DIV1;
    timer_1.TIM_CounterMode=TIM_CounterMode_Up;
    timer_1.TIM_Prescaler = 11999;
    timer_1.TIM_Period=39;  //39 for 50 Hz
 
    TIM_TimeBaseInit (TIM1, &timer_1);
 
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
 
    TIM_Cmd(TIM1, ENABLE);
    return;
}
 
 
void TIM1_Interrupt_Configuration()
{
    NVIC_InitTypeDef timer_1_irq;
 
    timer_1_irq.NVIC_IRQChannel=TIM1_UP_TIM16_IRQn;
    timer_1_irq.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&timer_1_irq);
    return;
}

void transfer_data(void)
{
  union {
    s16 x;
    u8 c[2];
  } my_int[3];

  static int contatore = 0; 
    
  STM32vldiscovery_LEDToggle(LED3);
    

#ifdef _MAGNET
//  u8 mychar = 0x01; // 00000000 enable magnetomter (single-conversion)
//  I2C_Write(&mychar, 0x26, LSM303D_ADD_1, 1);  // CTRL7
  
  I2C_Read(&my_int[0].c[0], 0x08, LSM303D_ADD_1, 1); // read magnetometer X value
  I2C_Read(&my_int[0].c[1], 0x09, LSM303D_ADD_1, 1); 
  I2C_Read(&my_int[1].c[0], 0x0A, LSM303D_ADD_1, 1); // read magnetometer Y value
  I2C_Read(&my_int[1].c[1], 0x0B, LSM303D_ADD_1, 1); 
  I2C_Read(&my_int[2].c[0], 0x0C, LSM303D_ADD_1, 1); // read magnetometer Z value
  I2C_Read(&my_int[2].c[1], 0x0D, LSM303D_ADD_1, 1);
  
  my_data[wr_i].m1x = my_int[0].x;
  my_data[wr_i].m1y = my_int[1].x;
  my_data[wr_i].m1z = my_int[2].x;
    
/*
  sprintf(mystring, "%d M1: %8d  %8d  %8d  ", contatore, my_int[0].x*8, my_int[1].x*8, my_int[2].x*8);
 
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  }
*/
//  I2C_Write(&mychar, 0x26, LSM303D_ADD_2, 1);  // CTRL7  
  I2C_Read(&my_int[0].c[0], 0x08, LSM303D_ADD_2, 1); // read magnetometer X value
  I2C_Read(&my_int[0].c[1], 0x09, LSM303D_ADD_2, 1); 
  I2C_Read(&my_int[1].c[0], 0x0A, LSM303D_ADD_2, 1); // read magnetometer Y value
  I2C_Read(&my_int[1].c[1], 0x0B, LSM303D_ADD_2, 1); 
  I2C_Read(&my_int[2].c[0], 0x0C, LSM303D_ADD_2, 1); // read magnetometer Z value
  I2C_Read(&my_int[2].c[1], 0x0D, LSM303D_ADD_2, 1); 

  my_data[wr_i].m2x = my_int[0].x*8;
  my_data[wr_i].m2y = my_int[1].x*8;
  my_data[wr_i].m2z = my_int[2].x*8;
  
/*
  sprintf(mystring, "M2: %8d  %8d  %8d\n", my_int[0].x*8, my_int[1].x*8, my_int[2].x*8);

  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  } 
*/
#endif

#ifdef _ACCEL
  I2C_Read(&my_int[0].c[0], 0x28, LSM303D_ADD_1, 1); // read accelerometer X value
  I2C_Read(&my_int[0].c[1], 0x29, LSM303D_ADD_1, 1); 
  I2C_Read(&my_int[1].c[0], 0x2A, LSM303D_ADD_1, 1); // read accelerometer Y value
  I2C_Read(&my_int[1].c[1], 0x2B, LSM303D_ADD_1, 1); 
  I2C_Read(&my_int[2].c[0], 0x2C, LSM303D_ADD_1, 1); // read accelerometer Z value
  I2C_Read(&my_int[2].c[1], 0x2D, LSM303D_ADD_1, 1); 

  my_data[wr_i].a1x = my_int[0].x;
  my_data[wr_i].a1y = my_int[1].x;
  my_data[wr_i].a1z = my_int[2].x;

/*  
  sprintf(mystring, "%d A1: %8.3f  %8.3f  %8.3f  ", contatore, my_int[0].x*(2000.0/32768), my_int[1].x*(2000.0/32768), my_int[2].x*(2000.0/32768));
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  }
*/
  I2C_Read(&my_int[0].c[0], 0x28, LSM303D_ADD_2, 1); // read accelerometer X value
  I2C_Read(&my_int[0].c[1], 0x29, LSM303D_ADD_2, 1); 
  I2C_Read(&my_int[1].c[0], 0x2A, LSM303D_ADD_2, 1); // read accelerometer Y value
  I2C_Read(&my_int[1].c[1], 0x2B, LSM303D_ADD_2, 1); 
  I2C_Read(&my_int[2].c[0], 0x2C, LSM303D_ADD_2, 1); // read accelerometer Z value
  I2C_Read(&my_int[2].c[1], 0x2D, LSM303D_ADD_2, 1); 

  my_data[wr_i].a2x = my_int[0].x;
  my_data[wr_i].a2y = my_int[1].x;
  my_data[wr_i].a2z = my_int[2].x;
  /*
  sprintf(mystring, "A2: %8.3f  %8.3f  %8.3f\n", my_int[0].x*(2000.0/32768), my_int[1].x*(2000.0/32768), my_int[2].x*(2000.0/32768));
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  } 
*/  
#endif
  my_data[wr_i].empty = 0;
  my_data[wr_i].timestamp = contatore;
  wr_i++;
  if (wr_i>99)
  { //error
    STM32vldiscovery_LEDOff(LED3);
    STM32vldiscovery_LEDOff(LED4);
    while(1);
  }
  
  contatore++;
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
