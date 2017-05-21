/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "../../STM32_Driver/inc/stm32f4xx_it.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) SysTick_Handler(void)
{

}

/**
  * @brief  This function handles Window Watchdog Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) WWDG_IRQHandler(void)
{

}

/**
  * @brief  This function handles PVD Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) PVD_IRQHandler(void)
{

}

/**
  * @brief  This function handles Tamper Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TAMP_STAMP_IRQHandler(void)
{

}

/**
  * @brief  This function handles RTC Wakeup Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) RTC_WKUP_IRQHandler(void)
{

}

/**
  * @brief  This function handles Flash Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) FLASH_IRQHandler(void)
{

}

/**
  * @brief  This function handles RCC Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) RCC_IRQHandler(void)
{

}

/**
  * @brief  This function handles EXTI Line0 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) EXTI0_IRQHandler(void)
{

}

/**
  * @brief  This function handles EXTI Line1 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) EXTI1_IRQHandler(void)
{

}

/**
  * @brief  This function handles EXTI Line2 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) EXTI2_IRQHandler(void)
{

}

/**
  * @brief  This function handles EXTI Line3 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) EXTI3_IRQHandler(void)
{

}

/**
  * @brief  This function handles EXTI Line4 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) EXTI4_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream0 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream0_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream1 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream1_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream2 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream2_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream3 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream3_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream4 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream4_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream5 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream5_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream6 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream6_IRQHandler(void)
{

}

/**
  * @brief  This function handles ADC1, ADC2 and ADC3 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) ADC_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN1 TX Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN1_TX_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN1 RX0 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN1_RX0_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN1 RX1 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN1_RX1_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN1 SCE Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN1_SCE_IRQHandler(void)
{

}

/**
  * @brief  This function handles EXTI Line[9:5] Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) EXTI9_5_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM1 Break and TIM9 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM1_BRK_TIM9_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM1 Update and TIM10 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM1_UP_TIM10_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM1 Trigger and Commutation and TIM11 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM1_TRG_COM_TIM11_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM1 Capture Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM1_CC_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM2 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM2_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM3 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM3_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM4 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM4_IRQHandler(void)
{

}

/**
  * @brief  This function handles I2C1 Event Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) I2C1_EV_IRQHandler(void)
{

}

/**
  * @brief  This function handles I2C1 Error Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) I2C1_ER_IRQHandler(void)
{

}

/**
  * @brief  This function handles I2C2 Event Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) I2C2_EV_IRQHandler(void)
{

}

/**
  * @brief  This function handles I2C2 Error Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) I2C2_ER_IRQHandler(void)
{

}

/**
  * @brief  This function handles SPI1 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) SPI1_IRQHandler(void)
{

}

/**
  * @brief  This function handles SPI2 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) SPI2_IRQHandler(void)
{

}

/**
  * @brief  This function handles USART1 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) USART1_IRQHandler(void)
{

}

/**
  * @brief  This function handles USART2 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) USART2_IRQHandler(void)
{

}

/**
  * @brief  This function handles USART3 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) USART3_IRQHandler(void)
{

}

/**
  * @brief  This function handles EXTI Line[15:10] Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) EXTI15_10_IRQHandler(void)
{

}

/**
  * @brief  This function handles RTC Alarm Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) RTC_Alarm_IRQHandler(void)
{

}

/**
  * @brief  This function handles USB OTG FS Wakeup Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) OTG_FS_WKUP_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM8 Break and TIM12 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM8_BRK_TIM12_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM8 Update and TIM13 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM8_UP_TIM13_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM8 Trigger and Commutation and TIM14 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM8_TRG_COM_TIM14_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM8 Capture Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM8_CC_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream7 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA1_Stream7_IRQHandler(void)
{

}

/**
  * @brief  This function handles FSMC Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) FSMC_IRQHandler(void)
{

}

/**
  * @brief  This function handles SDIO Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) SDIO_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM5 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM5_IRQHandler(void)
{

}

/**
  * @brief  This function handles SPI3 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) SPI3_IRQHandler(void)
{

}

/**
  * @brief  This function handles UART4 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) UART4_IRQHandler(void)
{

}

/**
  * @brief  This function handles UART5 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) UART5_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM6 and DAC1&2 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM6_DAC_IRQHandler(void)
{

}

/**
  * @brief  This function handles TIM7 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) TIM7_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream0 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream0_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream1 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream1_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream2 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream2_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream3 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream3_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream4 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream4_IRQHandler(void)
{

}

/**
  * @brief  This function handles Ethernet Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) ETH_IRQHandler(void)
{

}

/**
  * @brief  This function handles Ethernet Wakeup Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) ETH_WKUP_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN2 TX Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN2_TX_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN2 RX0 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN2_RX0_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN2 RX1 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN2_RX1_IRQHandler(void)
{

}

/**
  * @brief  This function handles CAN2 SCE Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CAN2_SCE_IRQHandler(void)
{

}

/**
  * @brief  This function handles USB OTG FS Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) OTG_FS_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream5 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream5_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream6 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream6_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA2 Stream7 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DMA2_Stream7_IRQHandler(void)
{

}

/**
  * @brief  This function handles USART6 Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) USART6_IRQHandler(void)
{

}

/**
  * @brief  This function handles I2C3 Event Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) I2C3_EV_IRQHandler(void)
{

}

/**
  * @brief  This function handles I2C3 Error Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) I2C3_ER_IRQHandler(void)
{

}

/**
  * @brief  This function handles USB OTG HS End Point 1 Out Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) OTG_HS_EP1_OUT_IRQHandler(void)
{

}

/**
  * @brief  This function handles USB OTG HS End Point 1 In Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) OTG_HS_EP1_IN_IRQHandler(void)
{

}

/**
  * @brief  This function handles USB OTG HS Wakeup Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) OTG_HS_WKUP_IRQHandler(void)
{

}

/**
  * @brief  This function handles USB OTG HS Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) OTG_HS_IRQHandler(void)
{

}

/**
  * @brief  This function handles DCMI Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) DCMI_IRQHandler(void)
{

}

/**
  * @brief  This function handles Crypto Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) CRYP_IRQHandler(void)
{

}

/**
  * @brief  This function handles Hash and RNG Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) HASH_RNG_IRQHandler(void)
{

}

/**
  * @brief  This function handles FPU Interrupt Handler.
  * @param  None
  * @retval None
  */
void __attribute__((weak)) FPU_IRQHandler(void)
{

}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
