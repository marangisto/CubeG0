/**
  ******************************************************************************
  * @file    Examples_LL/I2C/I2C_OneBoard_Communication_IT/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to send/receive bytes over I2C IP using
  *          the STM32G0xx I2C LL API.
  *          Peripheral initialization done using LL unitary services functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void aux_main();
void toggle_probe();
void write_probe(uint8_t x);
void trace(const char *s, uint32_t x);
void init_i2c1();
void init_i2c2();
void write_i2c2(uint8_t addr, const uint8_t *buf, uint8_t nbytes);

/** @addtogroup STM32G0xx_LL_Examples
  * @{
  */

/** @addtogroup I2C_OneBoard_Communication_IT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/**
  * @brief I2C devices settings
  */
/* Timing register value is computed with the STM32CubeMX Tool,
  * Fast Mode @400kHz with I2CCLK = 56 MHz,
  * rise time = 100ns, fall time = 10ns
  * Timing Value = (uint32_t)0x00F02B86
  */
#define I2C_TIMING               0x00F02B86

/**
  * @brief Define related to SlaveTransmit process
  */
#define SLAVE_BYTE_TO_SEND       (uint8_t)0xA5

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress       = 0;

/**
  * @brief Variables related to MasterReceive process
  */
uint8_t      aReceiveBuffer[0xF] = {0};
__IO uint8_t ubReceiveIndex      = 0;

/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
void     LED_Blinking(uint32_t Period);
void     WaitForUserButtonPress(void);
void     Handle_I2C_Master(void);
void     UserButton_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the system clock to 56 MHz */
  SystemClock_Config();

  aux_main();

  /* Initialize LED4 */
  LED_Init();

  /* Set LED4 Off */
  LED_Off();

  /* Initialize User push-button in EXTI mode */
  UserButton_Init();

    init_i2c1();
    init_i2c2();

  NVIC_EnableIRQ(I2C2_IRQn);
  LL_I2C_EnableIT_RX(I2C2);
  LL_I2C_EnableIT_NACK(I2C2);
  LL_I2C_EnableIT_ERR(I2C2);
  LL_I2C_EnableIT_STOP(I2C2);

  /* Wait for User push-button press to start transfer */
  WaitForUserButtonPress();

  /* Handle I2C2 events (Master) */
  Handle_I2C_Master();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Initialize LED4.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED4 Clock */
  LED4_GPIO_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LED4 */
  LL_GPIO_SetPinMode(LED4_GPIO_PORT, LED4_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  //LL_GPIO_SetPinOutputType(LED4_GPIO_PORT, LED4_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  //LL_GPIO_SetPinSpeed(LED4_GPIO_PORT, LED4_PIN, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED4_GPIO_PORT, LED4_PIN, LL_GPIO_PULL_NO);
}

/**
  * @brief  Turn-on LED4.
  * @param  None
  * @retval None
  */
void LED_On(void)
{
  /* Turn LED4 on */
  LL_GPIO_SetOutputPin(LED4_GPIO_PORT, LED4_PIN);
}

/**
  * @brief  Turn-off LED4.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED4 off */
  LL_GPIO_ResetOutputPin(LED4_GPIO_PORT, LED4_PIN);
}

/**
  * @brief  Set LED4 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :   
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
void LED_Blinking(uint32_t Period)
{
  /* Turn LED4 on */
  LL_GPIO_SetOutputPin(LED4_GPIO_PORT, LED4_PIN);

  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED4_GPIO_PORT, LED4_PIN);  
    LL_mDelay(Period);
  }
}

/**
  * @brief  Configures User push-button in GPIO or EXTI Line Mode.
  * @param  None 
  * @retval None
  */
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();

  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* Connect External Line to the GPIO*/
  USER_BUTTON_SYSCFG_SET_EXTI();

  /* Enable a falling trigger External line 13 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_FALLING_TRIG_ENABLE();

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn); 
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn,0x03);  
}

/**
  * @brief  Wait for User push-button press to start transfer.
  * @param  None 
  * @retval None
  */
  /*  */
void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
    LL_GPIO_TogglePin(LED4_GPIO_PORT, LED4_PIN);
    LL_mDelay(LED_BLINK_FAST);
  }
  /* Turn LED4 off */
  LL_GPIO_ResetOutputPin(LED4_GPIO_PORT, LED4_PIN);
}

/**
  * @brief  This Function handle Master events to perform a reception process
  * @note  This function is composed in one step :
  *        -1- Initiate a Start condition to the Slave device.
  * @param  None
  * @retval None
  */
void Handle_I2C_Master(void)
{
  /* (1) Initiate a Start condition to the Slave device ***********************/

  /* Master Generate Start condition for a read request:
   *  - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS
   *  - with a auto stop condition generation when receive 1 byte
   */
  LL_I2C_HandleTransfer(I2C2, SLAVE_OWN_ADDRESS, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 56000000
  *            HCLK(Hz)                       = 56000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSI clock division factor      = 1  
  *            HSI Frequency(Hz)              = 16000000
  *            Flash Latency(WS)              = 2
  *            PLLM                           = 4
  *            PLLN                           = 70
  *            PLLR                           = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  /* HSI configuration and activation */
  LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_1);
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 70, LL_RCC_PLLR_DIV_5);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  /* Set systick to 1ms in using frequency set to 56MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_HSI_FREQ(),
                                  LL_RCC_PLLM_DIV_4, 70, LL_RCC_PLLR_DIV_5)*/
  LL_Init1msTick(56000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(56000000);
}
/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/
/**
  * @brief  Function to manage User push-button
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Update User push-button variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
}

/**
  * @brief  Function called from I2C IRQ Handler when TXIS flag is set
  *         Function is in charge of transmit a byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Ready_To_Transmit_Callback(void)
{
    toggle_probe();
  /* Send the Byte requested by the Master */
  LL_I2C_TransmitData8(I2C1, SLAVE_BYTE_TO_SEND);
}

/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of reading byte received on I2C lines.
  * @param  None
  * @retval None
  */
void Master_Reception_Callback(void)
{
    toggle_probe();
  /* Read character in Receive Data register.
  RXNE flag is cleared by reading data in RXDR register */

  aReceiveBuffer[ubReceiveIndex++] = LL_I2C_ReceiveData8(I2C2);
}

/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED4 is On if data are correct.
  * @param  None
  * @retval None
  */
void Master_Complete_Callback(void)
{
    toggle_probe();
  /* Read Received character.
  RXNE flag is cleared by reading of RXDR register */
  if(aReceiveBuffer[ubReceiveIndex-1] == SLAVE_BYTE_TO_SEND)
  {
    /* Turn LED4 On:
     * - Expected byte has been received
     * - Master Rx sequence completed successfully
     */
    LED_On();
  }
  else
  {
    /* Call Error function */
    Error_Callback();
  }
}

/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  /* Disable I2C1_IRQn and I2C2_IRQn */
  NVIC_DisableIRQ(I2C1_IRQn);
  NVIC_DisableIRQ(I2C2_IRQn);

  /* Unexpected event : Set LED4 to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
