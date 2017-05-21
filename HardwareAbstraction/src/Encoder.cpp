/**
 * @file	Encoder.cpp
 * @author	Kevin WYSOCKI
 * @date	18 nov. 2016
 * @brief	Encoder abstraction class
 */

#include <stddef.h>
#include "Encoder.hpp"
#include "common.h"

using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// TIM5_CH1/CH2
#define ENC0_CH_A_PORT			(GPIOH)
#define ENC0_CH_A_PIN			(GPIO_Pin_10)
#define ENC0_CH_A_PINSOURCE		(GPIO_PinSource10)
#define ENC0_CH_B_PORT			(GPIOH)
#define ENC0_CH_B_PIN			(GPIO_Pin_11)
#define ENC0_CH_B_PINSOURCE		(GPIO_PinSource11)
#define ENC0_IO_AF				(GPIO_AF_TIM5)
#define ENC0_RELOAD_VALUE		(4095)			// Number of steps per encoder turn minus one
#define ENC0_TIMER				(TIM5)
#define ENC0_INT_CHANNEL		(TIM5_IRQn)
#define ENC0_INT_PRIORITY		(0u)

// TIM8_CH1/CH2
#define ENC1_CH_A_PORT			(GPIOI)
#define ENC1_CH_A_PIN			(GPIO_Pin_5)
#define ENC1_CH_A_PINSOURCE		(GPIO_PinSource5)
#define ENC1_CH_B_PORT			(GPIOI)
#define ENC1_CH_B_PIN			(GPIO_Pin_6)
#define ENC1_CH_B_PINSOURCE		(GPIO_PinSource6)
#define ENC1_IO_AF				(GPIO_AF_TIM8)
#define ENC1_RELOAD_VALUE		(4095)			// Number of steps per encoder turn minus one
#define ENC1_TIMER				(TIM8)
#define ENC1_INT_CHANNEL		(TIM8_UP_TIM13_IRQn)
#define ENC1_INT_PRIORITY		(0u)

#define TIM_FLAG_OVERFLOW		(1u << 0u)
#define TIM_FLAG_UNDERFLOW		(1u << 1u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

Encoder* _enc[Encoder::ENCODER_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve Encoder definitions from Encoder ID
 * @param id : Encoder ID
 * @return ENC_DEF structure
 */
static ENC_DEF _getENCStruct (enum Encoder::ID id)
{
	ENC_DEF enc;

	assert(id < HAL::Encoder::ENCODER_MAX);

	switch(id)
	{
	case Encoder::ENCODER0:
		enc.CH_A.PORT			=	ENC0_CH_A_PORT;
		enc.CH_A.PIN			=	ENC0_CH_A_PIN;
		enc.CH_A.PINSOURCE		=	ENC0_CH_A_PINSOURCE;
		enc.CH_A.AF				=	ENC0_IO_AF;
		enc.CH_B.PORT			=	ENC0_CH_B_PORT;
		enc.CH_B.PIN			=	ENC0_CH_B_PIN;
		enc.CH_B.PINSOURCE		=	ENC0_CH_B_PINSOURCE;
		enc.CH_B.AF				=	ENC0_IO_AF;

		enc.TIMER.TIMER			=	ENC0_TIMER;
		enc.TIMER.RELOAD_VAL	=	ENC0_RELOAD_VALUE;

		enc.INT.PRIORITY		=	ENC0_INT_PRIORITY;
		enc.INT.CHANNEL			=	ENC0_INT_CHANNEL;
		break;

	case Encoder::ENCODER1:
		enc.CH_A.PORT			=	ENC1_CH_A_PORT;
		enc.CH_A.PIN			=	ENC1_CH_A_PIN;
		enc.CH_A.PINSOURCE		=	ENC1_CH_A_PINSOURCE;
		enc.CH_A.AF				=	ENC1_IO_AF;
		enc.CH_B.PORT			=	ENC1_CH_B_PORT;
		enc.CH_B.PIN			=	ENC1_CH_B_PIN;
		enc.CH_B.PINSOURCE		=	ENC1_CH_B_PINSOURCE;
		enc.CH_B.AF				=	ENC1_IO_AF;

		enc.TIMER.TIMER			=	ENC1_TIMER;
		enc.TIMER.RELOAD_VAL	=	ENC1_RELOAD_VALUE;

		enc.INT.PRIORITY		=	ENC1_INT_PRIORITY;
		enc.INT.CHANNEL			=	ENC1_INT_CHANNEL;
		break;
	default:
		break;
	}

	return enc;
}

/**
 * @brief Initialize peripheral for a specific Encoder
 * @param id : Encoder ID
 */
static void _hardwareInit (enum Encoder::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	TIM_TimeBaseInitTypeDef TIMBaseStruct;
	NVIC_InitTypeDef NVICStruct;

	ENC_DEF enc;

	assert(id < Encoder::ENCODER_MAX);

	enc = _getENCStruct(id);

	// Init IO and TX pins
	GPIOStruct.GPIO_Mode	=	GPIO_Mode_AF;
	GPIOStruct.GPIO_OType	=	GPIO_OType_PP;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_High_Speed;
	GPIOStruct.GPIO_Pin		=	enc.CH_A.PIN;

	GPIO_PinAFConfig(enc.CH_A.PORT, enc.CH_A.PINSOURCE, enc.CH_A.AF);
	GPIO_Init(enc.CH_A.PORT, &GPIOStruct);

	GPIOStruct.GPIO_Pin		=	enc.CH_B.PIN;

	GPIO_PinAFConfig(enc.CH_B.PORT, enc.CH_B.PINSOURCE, enc.CH_B.AF);
	GPIO_Init(enc.CH_B.PORT, &GPIOStruct);

	// TIMER Init
	TIMBaseStruct.TIM_ClockDivision		=	TIM_CKD_DIV1;
	TIMBaseStruct.TIM_CounterMode		=	TIM_CounterMode_Up;
	TIMBaseStruct.TIM_Prescaler			=	0u;
	TIMBaseStruct.TIM_RepetitionCounter	=	0u;
	TIMBaseStruct.TIM_Period			=	enc.TIMER.RELOAD_VAL;

	TIM_Cmd(enc.TIMER.TIMER, DISABLE);
	TIM_TimeBaseInit(enc.TIMER.TIMER, &TIMBaseStruct);
	TIM_SetCounter(enc.TIMER.TIMER, 0u);

	TIM_EncoderInterfaceConfig(enc.TIMER.TIMER,
							   TIM_EncoderMode_TI12,
							   TIM_ICPolarity_Rising,
							   TIM_ICPolarity_Rising);

	TIM_ITConfig(enc.TIMER.TIMER, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(enc.TIMER.TIMER, TIM_FLAG_Update);

	// NVIC Init
	NVICStruct.NVIC_IRQChannel						=	enc.INT.CHANNEL;
	NVICStruct.NVIC_IRQChannelPreemptionPriority 	= 	enc.INT.PRIORITY;
	NVICStruct.NVIC_IRQChannelSubPriority 			= 	0;
	NVICStruct.NVIC_IRQChannelCmd					=	ENABLE;

	NVIC_Init(&NVICStruct);

	TIM_Cmd(enc.TIMER.TIMER, ENABLE);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	Encoder* Encoder::GetInstance (Encoder::ID id)
	{
		assert(id < Encoder::ENCODER_MAX);

		// if encoder instance already exists
		if(_enc[id] != NULL)
		{
			return _enc[id];
		}
		else
		{
			// Create encoder instance
			_enc[id] = new Encoder(id);

			return _enc[id];
		}
	}

	Encoder::Encoder (Encoder::ID id)
	{
		this->turnCounter	=	0;
		this->prevAbsValue	=	0;
		this->absolutePos	=	0;
		this->relativePos	=	0;

		_hardwareInit(id);

		this->def = _getENCStruct(id);
	}

	int64_t Encoder::GetAbsoluteValue()
	{
		this->prevAbsValue	= 	this->absolutePos;
		this->absolutePos 	= 	(this->def.TIMER.RELOAD_VAL * this->turnCounter) + TIM_GetCounter(this->def.TIMER.TIMER);
		this->relativePos 	= 	this->absolutePos - this->prevAbsValue;

		return this->absolutePos;
	}

	int32_t Encoder::GetRelativeValue()
	{
		this->prevAbsValue	= 	this->absolutePos;
		this->absolutePos 	= 	(this->def.TIMER.RELOAD_VAL * this->turnCounter) + TIM_GetCounter(this->def.TIMER.TIMER);
		this->relativePos 	= 	this->absolutePos - this->prevAbsValue;

		return this->relativePos;
	}

	void Encoder::INTERNAL_InterruptCallback(uint8_t flag)
	{
		if(flag == TIM_Direction_CounterDowncounting)
		{
			this->turnCounter--;
		}
		else
		{
			this->turnCounter++;
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Interrupt Handler                                                          */
/*----------------------------------------------------------------------------*/
extern "C"
{
	/**
	 * @brief Encoder 0 interrupt handler
	 */
	void TIM5_IRQHandler (void)
	{
		uint16_t flag = 0u;

		if(TIM_GetFlagStatus(TIM5, TIM_FLAG_Update) == SET)
		{
			flag = TIM_GetCounterDirection(TIM5);

			TIM_ClearFlag(TIM5, TIM_FLAG_Update);

			if(_enc[Encoder::ENCODER0] != NULL)
			{
				_enc[Encoder::ENCODER0]->INTERNAL_InterruptCallback(flag);
			}
		}
	}

	/**
	 * @brief Encoder 1 interrupt handler
	 */
	void TIM8_UP_TIM13_IRQHandler (void)
	{
		uint16_t flag = 0u;

		if(TIM_GetFlagStatus(TIM8, TIM_FLAG_Update) == SET)
		{
			flag = TIM_GetCounterDirection(TIM8);

			TIM_ClearFlag(TIM8, TIM_FLAG_Update);

			if(_enc[Encoder::ENCODER1] != NULL)
			{
				_enc[Encoder::ENCODER1]->INTERNAL_InterruptCallback(flag);
			}
		}
	}
}
