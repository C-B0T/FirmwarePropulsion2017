/**
 * @file	GPIO.cpp
 * @author	Kevin WYSOCKI
 * @date	8 nov. 2016
 * @brief	GPIO Abstraction Class
 */

#include <stdio.h>
#include "GPIO.hpp"
#include "common.h"

using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// DIG_IN2
#define GPIO0_PORT				(GPIOF)
#define GPIO0_PIN				(GPIO_Pin_7)
#define GPIO0_MODE				(GPIO_Mode_IN)
#define GPIO0_INT_PORTSOURCE	(EXTI_PortSourceGPIOF)
#define GPIO0_INT_PINSOURCE		(EXTI_PinSource7)
#define GPIO0_INT_LINE			(EXTI_Line7)
#define GPIO0_INT_TRIGGER		(EXTI_Trigger_Falling)
#define GPIO0_INT_CHANNEL		(EXTI9_5_IRQn)
#define GPIO0_INT_PRIORITY		(0u)

// PWM_OUT1
#define GPIO1_PORT				(GPIOA)
#define GPIO1_PIN				(GPIO_Pin_1)
#define GPIO1_MODE				(GPIO_Mode_OUT)

// DIG_OUT7
#define GPIO2_PORT				(GPIOH)
#define GPIO2_PIN				(GPIO_Pin_2)
#define GPIO2_MODE				(GPIO_Mode_OUT)

// DIG_IN3
#define GPIO3_PORT				(GPIOF)
#define GPIO3_PIN				(GPIO_Pin_9)
#define GPIO3_MODE				(GPIO_Mode_IN)
#define GPIO3_INT_PORTSOURCE	(EXTI_PortSourceGPIOF)
#define GPIO3_INT_PINSOURCE		(EXTI_PinSource9)
#define GPIO3_INT_LINE			(EXTI_Line9)
#define GPIO3_INT_TRIGGER		(EXTI_Trigger_Falling)
#define GPIO3_INT_CHANNEL		(EXTI9_5_IRQn)
#define GPIO3_INT_PRIORITY		(0u)

// PWM_OUT3
#define GPIO4_PORT				(GPIOA)
#define GPIO4_PIN				(GPIO_Pin_3)
#define GPIO4_MODE				(GPIO_Mode_OUT)

// DIG_OUT8
#define GPIO5_PORT				(GPIOH)
#define GPIO5_PIN				(GPIO_Pin_3)
#define GPIO5_MODE				(GPIO_Mode_OUT)

// LED1
#define GPIO6_PORT				(GPIOI)
#define GPIO6_PIN				(GPIO_Pin_8)
#define GPIO6_MODE				(GPIO_Mode_OUT)

// LED2
#define GPIO7_PORT				(GPIOI)
#define GPIO7_PIN				(GPIO_Pin_9)
#define GPIO7_MODE				(GPIO_Mode_OUT)

// LED3
#define GPIO8_PORT				(GPIOI)
#define GPIO8_PIN				(GPIO_Pin_10)
#define GPIO8_MODE				(GPIO_Mode_OUT)

// LED4
#define GPIO9_PORT				(GPIOI)
#define GPIO9_PIN				(GPIO_Pin_11)
#define GPIO9_MODE				(GPIO_Mode_OUT)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief GPIO instances
 */
GPIO* _gpio[GPIO::GPIO_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve GPIO definitions from GPIO ID
 * @param id : GPIO ID
 * @return GPIO_DEF structure
 */
static GPIO_DEF _getGPIOStruct (enum GPIO::ID id)
{
	GPIO_DEF gpio;

	assert(id < HAL::GPIO::GPIO_MAX);

	switch(id)
	{
	case HAL::GPIO::GPIO0:
		gpio.IO.PORT		=	GPIO0_PORT;
		gpio.IO.PIN			=	GPIO0_PIN;
		gpio.IO.MODE		=	GPIO0_MODE;
		gpio.INT.PORTSOURCE	=	GPIO0_INT_PORTSOURCE;
		gpio.INT.PINSOURCE	=	GPIO0_INT_PINSOURCE;
		gpio.INT.LINE		=	GPIO0_INT_LINE;
		gpio.INT.TRIGGER	=	GPIO0_INT_TRIGGER;
		gpio.INT.CHANNEL	=	GPIO0_INT_CHANNEL;
		gpio.INT.PRIORITY	=	GPIO0_INT_PRIORITY;
		break;
	case HAL::GPIO::GPIO1:
		gpio.IO.PORT	=	GPIO1_PORT;
		gpio.IO.PIN		=	GPIO1_PIN;
		gpio.IO.MODE	=	GPIO1_MODE;
		break;
	case HAL::GPIO::GPIO2:
		gpio.IO.PORT	=	GPIO2_PORT;
		gpio.IO.PIN		=	GPIO2_PIN;
		gpio.IO.MODE	=	GPIO2_MODE;
		break;
	case HAL::GPIO::GPIO3:
		gpio.IO.PORT		=	GPIO3_PORT;
		gpio.IO.PIN			=	GPIO3_PIN;
		gpio.INT.PORTSOURCE	=	GPIO3_INT_PORTSOURCE;
		gpio.INT.PINSOURCE	=	GPIO3_INT_PINSOURCE;
		gpio.INT.LINE		=	GPIO3_INT_LINE;
		gpio.INT.TRIGGER	=	GPIO3_INT_TRIGGER;
		gpio.INT.CHANNEL	=	GPIO3_INT_CHANNEL;
		gpio.INT.PRIORITY	=	GPIO3_INT_PRIORITY;
		gpio.IO.MODE		=	GPIO3_MODE;
		break;
	case HAL::GPIO::GPIO4:
		gpio.IO.PORT	=	GPIO4_PORT;
		gpio.IO.PIN		=	GPIO4_PIN;
		gpio.IO.MODE	=	GPIO4_MODE;
		break;
	case HAL::GPIO::GPIO5:
		gpio.IO.PORT	=	GPIO5_PORT;
		gpio.IO.PIN		=	GPIO5_PIN;
		gpio.IO.MODE	=	GPIO5_MODE;
		break;
	case HAL::GPIO::GPIO6:
		gpio.IO.PORT	=	GPIO6_PORT;
		gpio.IO.PIN		=	GPIO6_PIN;
		gpio.IO.MODE	=	GPIO6_MODE;
		break;
	case HAL::GPIO::GPIO7:
		gpio.IO.PORT	=	GPIO7_PORT;
		gpio.IO.PIN		=	GPIO7_PIN;
		gpio.IO.MODE	=	GPIO7_MODE;
		break;
	case HAL::GPIO::GPIO8:
		gpio.IO.PORT	=	GPIO8_PORT;
		gpio.IO.PIN		=	GPIO8_PIN;
		gpio.IO.MODE	=	GPIO8_MODE;
		break;
	case HAL::GPIO::GPIO9:
		gpio.IO.PORT	=	GPIO9_PORT;
		gpio.IO.PIN		=	GPIO9_PIN;
		gpio.IO.MODE	=	GPIO9_MODE;
		break;
	default:
		break;
	}

	return gpio;
}

/**
 * @brief Initialize peripheral for a specified GPIO
 * @param id : GPIO ID
 */
static void _hardwareInit (enum GPIO::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	EXTI_InitTypeDef EXTIStruct;
	NVIC_InitTypeDef NVICStruct;

	GPIO_DEF gpio;

	assert(id < HAL::GPIO::GPIO_MAX);

	gpio = _getGPIOStruct(id);

	// GPIO Init
	GPIOStruct.GPIO_OType 	= 	GPIO_OType_PP;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_High_Speed;
	GPIOStruct.GPIO_Pin		=	gpio.IO.PIN;
	GPIOStruct.GPIO_Mode	=	gpio.IO.MODE;

	GPIO_Init(gpio.IO.PORT, &GPIOStruct);

	// INT Init
	if(gpio.IO.MODE == GPIO_Mode_IN)
	{
		// Connect INT Line to GPIO pin
		SYSCFG_EXTILineConfig(gpio.INT.PORTSOURCE, gpio.INT.PINSOURCE);

		// Init INT
		EXTIStruct.EXTI_Line		= 	gpio.INT.LINE;
		EXTIStruct.EXTI_Trigger		=	(EXTITrigger_TypeDef)gpio.INT.TRIGGER;
		EXTIStruct.EXTI_Mode		=	EXTI_Mode_Interrupt;
		EXTIStruct.EXTI_LineCmd		=	ENABLE;

		EXTI_ClearITPendingBit(gpio.INT.LINE);

		EXTI_Init(&EXTIStruct);

		// Init NVIC
		NVICStruct.NVIC_IRQChannel						=	gpio.INT.CHANNEL;
		NVICStruct.NVIC_IRQChannelPreemptionPriority 	= 	gpio.INT.PRIORITY;
		NVICStruct.NVIC_IRQChannelSubPriority 			= 	0;
		NVICStruct.NVIC_IRQChannelCmd					=	ENABLE;

		NVIC_Init(&NVICStruct);
	}
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/
namespace HAL
{
	GPIO* GPIO::GetInstance (enum GPIO::ID id)
	{
		assert(id < GPIO::GPIO_MAX);

		// if GPIO instance already exists
		if(_gpio[id] != NULL)
		{
			return _gpio[id];
		}
		else
		{
			// Create GPIO instance
			_gpio[id] = new GPIO(id);

			return _gpio[id];
		}
	}

	GPIO::GPIO (enum GPIO::ID id)
	{
		this->id = id;
		this->intState = false;
		this->def = _getGPIOStruct(id);

		_hardwareInit(id);
	}

	enum GPIO::State GPIO::Get ()
	{
		enum GPIO::State state = GPIO::Low;
		BitAction bit = Bit_RESET;

		bit = (BitAction)GPIO_ReadInputDataBit(this->def.IO.PORT, this->def.IO.PIN);

		if(bit == Bit_SET)
		{
			state = GPIO::High;
		}

		return state;
	}

	void GPIO::Set (enum GPIO::State state)
	{
		BitAction bit = Bit_RESET;

		if(this->def.IO.MODE == GPIO_Mode_OUT)
		{
			if(state == GPIO::High)
			{
				bit = Bit_SET;
			}

			GPIO_WriteBit(this->def.IO.PORT, this->def.IO.PIN, bit);
		}
	}

	void GPIO::Toggle ()
	{
		if(this->def.IO.MODE == GPIO_Mode_OUT)
		{
			GPIO_ToggleBits(this->def.IO.PORT, this->def.IO.PIN);
		}
	}

	void GPIO::INTERNAL_InterruptCallback()
	{
		if(this->def.IO.MODE == GPIO_Mode_IN)
		{
			this->StateChanged();
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Interrupt Handler                                                          */
/*----------------------------------------------------------------------------*/

extern "C"
{
	/**
	 * @brief INT Line 9 to 5 Interrupt Handler
	 */
	void INT9_5_IRQHandler(void)
	{
		if(EXTI_GetFlagStatus(GPIO0_INT_LINE) == SET)
		{
			EXTI_ClearFlag(GPIO0_INT_LINE);

			GPIO* gpio = GPIO::GetInstance(GPIO::GPIO0);

			gpio->INTERNAL_InterruptCallback();
		}
		else if(EXTI_GetFlagStatus(GPIO3_INT_LINE) == SET)
		{
			EXTI_ClearFlag(GPIO3_INT_LINE);

			GPIO* gpio = GPIO::GetInstance(GPIO::GPIO3);

			gpio->INTERNAL_InterruptCallback();
		}
	}
}
