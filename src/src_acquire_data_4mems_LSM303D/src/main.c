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
#include <stdio.h>

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
void init_I2C2(void);
void I2C_Read(u8* pBuffer, u8 ReadAddr, u8 DevAddr, u8 NumByteToRead, I2C_TypeDef* I2Cx);
void I2C_Write(u8* pBuffer, u8 WriteAddr, u8 DevAddr, u8 NumByteToWrite, I2C_TypeDef* I2Cx);
void init_USART1(void);
void TIM1_Interrupt_Configuration();
void TIM1_Configuration();
void azzeraPinI2C();

int main(void)
{
  int i;
  u8 mychar = 'A';
  
  /* Enable GPIOx Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); 
  
  /* Initialise LEDs LD3&LD4, both on */
  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDInit(LED4);
  azzeraPinI2C ();
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
  } //inizializza i 10 campi empty dell'array di struct my data a 1
   
  init_USART1();
   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, 'Z'); 
    
  init_I2C1(); //Inizializza il BUS I2C1
  init_I2C2();
  TIM1_Interrupt_Configuration();
    
#ifdef _MAGNET  
  mychar = 0x70; // 01110000 enable magnetometer (high resolution, 50 Hz)
 /// I2C_Write(&mychar, 0x24, LSM303D_ADD_1, 1, I2C1);  // CTRL0
  I2C_Write(&mychar, 0x24, LSM303D_ADD_1, 1, I2C2);  // CTRL0
  I2C_Write(&mychar, 0x24, LSM303D_ADD_2, 1, I2C1);  // CTRL0
  I2C_Write(&mychar, 0x24, LSM303D_ADD_2, 1, I2C2);  // CTRL0
  mychar = 0x00; // 00000000 +/-2 2 gauss full scale
  I2C_Write(&mychar, 0x25, LSM303D_ADD_1, 1, I2C1);  // CTRL6
  I2C_Write(&mychar, 0x25, LSM303D_ADD_1, 1, I2C2);  // CTRL6
  I2C_Write(&mychar, 0x25, LSM303D_ADD_2, 1, I2C1);  // CTRL6
  I2C_Write(&mychar, 0x25, LSM303D_ADD_2, 1, I2C2);  // CTRL6
  mychar = 0x00; // 00000000 enable magnetomter (continuous conversion)
  I2C_Write(&mychar, 0x26, LSM303D_ADD_1, 1, I2C1);  // CTRL7
  I2C_Write(&mychar, 0x26, LSM303D_ADD_1, 1, I2C2);  // CTRL7
  I2C_Write(&mychar, 0x26, LSM303D_ADD_2, 1, I2C1);  // CTRL7
  I2C_Write(&mychar, 0x26, LSM303D_ADD_2, 1, I2C2);  // CTRL7
#endif
  
#ifdef _ACCEL
  mychar =  0x57; // 01010111 enable accelerometer (50 Hz)
  I2C_Write(&mychar, 0x20, LSM303D_ADD_1, 1, I2C1);  // CTRL1: attiva l'accellerometro sui 3 
  I2C_Write(&mychar, 0x20, LSM303D_ADD_1, 1, I2C2);  // CTRL1: attiva l'accellerometro sui 3 assi
  I2C_Write(&mychar, 0x20, LSM303D_ADD_2, 1, I2C1);  // CTRL1
  I2C_Write(&mychar, 0x20, LSM303D_ADD_2, 1, I2C2);  // CTRL1
  //Full scale: CTRL 2: di defaul e +- 2g, quindi non lo settiamo.
#endif
  
  TIM1_Configuration();  
  while(1);
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

void init_I2C1(void) //I2C2 usa i piedini b10 (clk) e b11(data)
{
I2C_InitTypeDef my_i2c;
GPIO_InitTypeDef my_gpio;

my_gpio.GPIO_Pin = GPIO_Pin_5; // VDD: prendi alimentazione da pin 8 e 9
my_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &my_gpio);
GPIO_SetBits(GPIOB, GPIO_Pin_5);

for (int k1=0; k1<100000; k1++);
  
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //l'ho già inizializzato per azzeraPIN, è il caso di rifarlo?
RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

/* Configure I2C pins: SCL and SDA */
my_gpio.GPIO_Pin =  GPIO_Pin_6| GPIO_Pin_7; //BERT: il 6 è il piedino per il clock, il 7 quello per i dati
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
my_i2c.I2C_ClockSpeed =   30000;  //150000

/* Apply I2C configuration after enabling it */
I2C_Init(I2C1, &my_i2c);

/* I2C Peripheral Enable */
I2C_Cmd(I2C1, ENABLE);
}

void init_I2C2(void)
{

  I2C_InitTypeDef my_i2c;
  GPIO_InitTypeDef my_gpio;

my_gpio.GPIO_Pin = GPIO_Pin_12; // vdd
my_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &my_gpio);
GPIO_SetBits(GPIOB, GPIO_Pin_12);            // SA02: 1 => address = 0x3A

for (int k=0; k<100000; k++); 

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //l'ho già inizializzato per azzeraPIN, è il caso di rifarlo?
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

/* Configure I2C pins: SCL and SDA */
my_gpio.GPIO_Pin =  GPIO_Pin_10| GPIO_Pin_11; //BERT: il 10 è il piedino per il clock, il 11 quello per i dati
my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
my_gpio.GPIO_Mode = GPIO_Mode_AF_OD;
GPIO_Init(GPIOB, &my_gpio);

/* I2C1 Reset */
RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2 , ENABLE);
RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2 , DISABLE);

I2C_StructInit(&my_i2c);
/* I2C configuration */
my_i2c.I2C_Mode = I2C_Mode_I2C;
my_i2c.I2C_DutyCycle = I2C_DutyCycle_2;
//my_i2c.I2C_OwnAddress1 = 0;
my_i2c.I2C_Ack = I2C_Ack_Enable;
my_i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
my_i2c.I2C_ClockSpeed =   30000; 

/* Apply I2C configuration after enabling it */
I2C_Init(I2C2, &my_i2c);
/* I2C Peripheral Enable */
I2C_Cmd(I2C2, ENABLE);
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

void I2C_Read(u8* pBuffer, u8 ReadAddr, u8 DevAddr, u8 NumByteToRead, I2C_TypeDef* I2Cx)
{
  __disable_irq();
    /* While the bus is busy */
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send LPS001DL address for write */
  I2C_Send7bitAddress(I2Cx, DevAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2Cx, ENABLE);

  if(NumByteToRead>1)
  {
    ReadAddr += 0x80; //the MSB bit of address is set to 1 to allow incremental address
  }

  /* Send the LPS001DL's internal address to write to */
  I2C_SendData(I2Cx, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send START condition a second time */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send LPS001DL address for read */
  I2C_Send7bitAddress(I2Cx, DevAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2Cx, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2Cx, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the LPS001DL */
      *pBuffer = I2C_ReceiveData(I2Cx);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  __enable_irq();

}

void I2C_Write(u8* pBuffer, u8 WriteAddr, u8 DevAddr, u8 NumByteToWrite, I2C_TypeDef* I2Cx)
{
  __disable_irq();    //BERT: disabilita gli interrupt
  
  /* Send STRAT condition */
  I2C_GenerateSTART(I2Cx, ENABLE); //BERT: il master deve sempre aspettarlo, xk
                                   //significa che il BUS è libero.

  /* Test on EV5 and clear it */
  //
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));  
  
  /* Send STLM75 address for write */
  I2C_Send7bitAddress(I2Cx, DevAddr, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Send the STLM75's internal address to write to */
  I2C_SendData(I2Cx, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  
  while(NumByteToWrite)
  {
    /* Send the byte to be written */
    I2C_SendData(I2Cx, *pBuffer);
    
    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    pBuffer++;
    NumByteToWrite--;
  }
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2Cx, ENABLE);
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
    timer_1.TIM_Period= 39;  //39 for 50 Hz
 
    TIM_TimeBaseInit (TIM1, &timer_1);
 
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
 
    TIM_Cmd(TIM1, ENABLE);
    return;
}
 
void TIM1_Interrupt_Configuration()
{
    NVIC_InitTypeDef timer_1_irq;
 
    timer_1_irq.NVIC_IRQChannel=TIM1_UP_TIM16_IRQn; //accetta ogni evento di timer 16 e gli Update di timer 1
    timer_1_irq.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&timer_1_irq);
    return;
}

void transfer_data(void)
{
  s16 my_int[3];
  char mystring[100];
  int i;
  static int contatore = 0; 
  mystring[0]='\0';  
  STM32vldiscovery_LEDToggle(LED3);

  /****************************************/
      /*OPERAZIONI DI LETTURA I2C1*/
  /****************************************/

#ifdef _MAGNET

  for (i=0; i<3; i++)
  {
      my_int[i]=0;
  }
  
  I2C_Read((u8 *)my_int, 0x08, LSM303D_ADD_1, 6, I2C1); // read magnetometer X value, Y value, Z value  
  sprintf(mystring, "%d M1: %8d  %8d  %8d  ", contatore, my_int[0]*8, my_int[1]*8, my_int[2]*8);
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  } 
  I2C_Read((u8 *)my_int, 0x08, LSM303D_ADD_2, 6, I2C1); // read magnetometer X value, Y value, Z value  

  sprintf(mystring, "%d M2: %8d  %8d  %8d\n", contatore, my_int[0]*8, my_int[1]*8, my_int[2]*8);

  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  }
#endif

#ifdef _ACCEL
  I2C_Read((u8 *)my_int, 0x28, LSM303D_ADD_1, 6, I2C1); // read accelerometer X value, Y value, Z value  
  
 sprintf(mystring, "%d A1: %8.3f  %8.3f  %8.3f  ", contatore, my_int[0]*(2000.0/32768), my_int[1]*(2000.0/32768), my_int[2]*(2000.0/32768));
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  }

  I2C_Read((u8 *)my_int, 0x28, LSM303D_ADD_2, 6, I2C1); // read accelerometer X value, Y value, Z value  

  sprintf(mystring, "%d A2: %8.3f  %8.3f  %8.3f\n", contatore, my_int[0]*(2000.0/32768), my_int[1]*(2000.0/32768), my_int[2]*(2000.0/32768));
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
} //FINE I2C1
#endif


  /****************************************/
      /*OPERAZIONI DI LETTURA I2C2*/
  /****************************************/

#ifdef _MAGNET

for (i=0; i<3; i++)
  {
      my_int[i]=0;
  }
  
  I2C_Read((u8 *)my_int, 0x08, LSM303D_ADD_1, 6, I2C2); // read magnetometer X value, Y value, Z value  
    
  sprintf(mystring, "%d M3: %8d  %8d  %8d  ", contatore, my_int[0]*8, my_int[1]*8, my_int[2]*8);
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  } 

  I2C_Read((u8 *)my_int, 0x08, LSM303D_ADD_2, 6, I2C2); // read magnetometer X value, Y value, Z value  

  sprintf(mystring, "%d M4: %8d  %8d  %8d\n", contatore, my_int[0]*8, my_int[1]*8, my_int[2]*8);

  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  }
#endif

#ifdef _ACCEL
  I2C_Read((u8 *)my_int, 0x28, LSM303D_ADD_1, 6, I2C2); // read accelerometer X value, Y value, Z value  
  
 sprintf(mystring, "%d A3: %8.3f  %8.3f  %8.3f  ", contatore, my_int[0]*(2000.0/32768), my_int[1]*(2000.0/32768), my_int[2]*(2000.0/32768));
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
  }

  I2C_Read((u8 *)my_int, 0x28, LSM303D_ADD_2, 6, I2C2); // read accelerometer X value, Y value, Z value  

  sprintf(mystring, "%d A4: %8.3f  %8.3f  %8.3f\n", contatore, my_int[0]*(2000.0/32768), my_int[1]*(2000.0/32768), my_int[2]*(2000.0/32768));
  
  for (i=0; mystring[i]!='\0'; i++)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    USART_SendData(USART1, mystring[i]);
} //FINE I2C2
#endif

  contatore++;
}


/*BERT: Azzera i pin PB9 PB8 PB7 PB6 PB5 PB4 (quelli a cui è attaccato il sensore). 
Il 4 non si resetta perché la funzione principale associata a quel pin non è function port bensì NJTRST.*/ 

void azzeraPinI2C ()
{
  GPIO_InitTypeDef my_gpio;  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //abilito il clock per APB2
  /*Configuro la struct my_gpio: output, push pull, 50 MHz di default, poi resetto*/
  my_gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4; 
  my_gpio.GPIO_Mode = GPIO_Mode_Out_PP; //Output, push pull 
  my_gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &my_gpio);
  GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4);
  for (int k=0; k<100000; k++); 
  return;
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
