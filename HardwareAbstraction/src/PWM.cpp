/**
 * @file	PWM.cpp
 * @author	Kevin WYSOCKI
 * @date	14 nov. 2016
 * @brief	PWM Abstraction class
 */

#include <stddef.h>
#include "PWM.hpp"
#include "common.h"

using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// TIM4_CH1
#define PWM0_IO_PORT		(GPIOB)
#define PWM0_IO_PIN			(GPIO_Pin_6)
#define PWM0_IO_PINSOURCE	(GPIO_PinSource6)
#define PWM0_IO_AF			(GPIO_AF_TIM4)
#define PWM0_FREQ			(10000)
#define PWM0_DUTYCYCLE		(0.5f)
#define PWM0_TIMER			(TIM4)
#define PWM0_TIMER_CHANNEL	(TIM_Channel_1)
#define PWM0_TIMER_FREQ		(SystemCoreClock / 4)	// TIM4 clock is derivated from APB1 clock

// TIM4_CH4
#define PWM1_IO_PORT		(GPIOB)
#define PWM1_IO_PIN			(GPIO_Pin_9)
#define PWM1_IO_PINSOURCE	(GPIO_PinSource9)
#define PWM1_IO_AF			(GPIO_AF_TIM4)
#define PWM1_FREQ			(10000)
#define PWM1_DUTYCYCLE		(0.5f)
#define PWM1_TIMER			(TIM4)
#define PWM1_TIMER_CHANNEL	(TIM_Channel_4)
#define PWM1_TIMER_FREQ		(SystemCoreClock / 4)	// TIM4 clock is derivated from APB1 clock

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief PWM instances
 */
static PWM* _pwm[PWM::PWM_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve PWM definitions from PWM ID
 * @param id : PWM ID
 * @return PWM_DEF structure
 */
static PWM_DEF _getPWMStruct (enum PWM::ID id)
{
	PWM_DEF pwm;

	assert(id < HAL::PWM::PWM_MAX);

	switch(id)
	{
	case PWM::PWM0:
		pwm.IO.PORT					=	PWM0_IO_PORT;
		pwm.IO.PIN					=	PWM0_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM0_IO_PINSOURCE;
		pwm.IO.AF					=	PWM0_IO_AF;
		pwm.TIMER.TIMER				=	PWM0_TIMER;
		pwm.TIMER.CHANNEL			=	PWM0_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM0_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM0_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM0_DUTYCYCLE;
		break;
	case PWM::PWM1:
		pwm.IO.PORT					=	PWM1_IO_PORT;
		pwm.IO.PIN					=	PWM1_IO_PIN;
		pwm.IO.PINSOURCE			=	PWM1_IO_PINSOURCE;
		pwm.IO.AF					=	PWM1_IO_AF;
		pwm.TIMER.TIMER				=	PWM1_TIMER;
		pwm.TIMER.CHANNEL			=	PWM1_TIMER_CHANNEL;
		pwm.TIMER.CLOCKFREQ			=	PWM1_TIMER_FREQ;
		pwm.PWM.DEFAULT_FREQ		=	PWM1_FREQ;
		pwm.PWM.DEFAULT_DUTYCYCLE	=	PWM1_DUTYCYCLE;
		break;
	default:
		break;
	}

	return pwm;
}

/**
 * @brief Initialize peripheral for a specific PWM
 * @param id : PWM ID
 */
static void _hardwareInit (enum PWM::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	TIM_TimeBaseInitTypeDef TIMBaseStruct;
	TIM_OCInitTypeDef TIMOCStruct;

	PWM_DEF pwm;

	assert(id < HAL::PWM::PWM_MAX);

	pwm = _getPWMStruct(id);

	// Init IO and TX pins
	GPIOStruct.GPIO_Mode	=	GPIO_Mode_AF;
	GPIOStruct.GPIO_OType	=	GPIO_OType_PP;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_High_Speed;
	GPIOStruct.GPIO_Pin		=	pwm.IO.PIN;

	GPIO_PinAFConfig(pwm.IO.PORT, pwm.IO.PINSOURCE, pwm.IO.AF);
	GPIO_Init(pwm.IO.PORT, &GPIOStruct);

	// TIMER Init
	TIMBaseStruct.TIM_ClockDivision		=	TIM_CKD_DIV1;
	TIMBaseStruct.TIM_CounterMode		=	TIM_CounterMode_Up;
	TIMBaseStruct.TIM_Prescaler			=	0u;
	TIMBaseStruct.TIM_RepetitionCounter	=	0u;
	TIMBaseStruct.TIM_Period			=	(pwm.TIMER.CLOCKFREQ / pwm.PWM.DEFAULT_FREQ) - 1u;

	TIM_Cmd(pwm.TIMER.TIMER, DISABLE);
	TIM_TimeBaseInit(pwm.TIMER.TIMER, &TIMBaseStruct);
	TIM_SetCounter(pwm.TIMER.TIMER, 0u);
	TIM_ARRPreloadConfig(pwm.TIMER.TIMER, ENABLE);

	// Output compare init
	TIMOCStruct.TIM_OCMode			=	TIM_OCMode_PWM1;
	TIMOCStruct.TIM_OCIdleState		=	TIM_OCIdleState_Reset;
	TIMOCStruct.TIM_OCNIdleState	=	TIM_OCNIdleState_Reset;
	TIMOCStruct.TIM_OCNPolarity		=	TIM_OCNPolarity_High;
	TIMOCStruct.TIM_OCPolarity		=	TIM_OCPolarity_High;
	TIMOCStruct.TIM_OutputNState	=	TIM_OutputNState_Enable;
	TIMOCStruct.TIM_OutputState		=	TIM_OutputState_Enable;
	TIMOCStruct.TIM_Pulse			=	(uint32_t)((float32_t)TIMBaseStruct.TIM_Period * pwm.PWM.DEFAULT_DUTYCYCLE);

	TIM_OCxInit(pwm.TIMER.TIMER, &TIMOCStruct, pwm.TIMER.CHANNEL);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	PWM* PWM::GetInstance (enum PWM::ID id)
	{
		assert(id < PWM::PWM_MAX);

		// if PWM instance already exists
		if(_pwm[id] != NULL)
		{
			return _pwm[id];
		}
		else
		{
			// Create PWM instance
			_pwm[id] = new PWM(id);

			return _pwm[id];
		}
	}

	PWM::PWM (enum PWM::ID id)
	{
		this->id = id;
		this->state = PWM::DISABLED;
		this->def = _getPWMStruct(id);
		this->dutyCycle = this->def.PWM.DEFAULT_DUTYCYCLE;
		this->frequency = this->def.PWM.DEFAULT_FREQ;

		_hardwareInit(id);
	}

	void PWM::SetDutyCycle (float32_t percent)
	{
		uint32_t ARR = 0u, CCR = 0u;

		if(percent > 1.0f)
		{
			percent = 1.0f;
		}
		else if(percent < 0.0f)
		{
			percent = 0.0f;
		}

		// 1. Get ARRx and CCRx value
		ARR = (PWM0_TIMER_FREQ / this->frequency) - 1u;
		CCR = (uint32_t)((float32_t)ARR * percent);

		// 2. Update ARRx register
		TIM_SetAutoreload(this->def.TIMER.TIMER, ARR);

		// 3. Update CCRx
		TIM_SetCompareX(this->def.TIMER.TIMER, CCR, this->def.TIMER.CHANNEL);

		// 4. Update instance duty cycle
		this->dutyCycle = percent;
	}

	void PWM::SetFrequency (uint32_t freq)
	{
		uint32_t ARR = 0u, CCR = 0u;

		assert(freq > 0u);

		// 1. Get ARRx and CCRx value
		ARR = (PWM0_TIMER_FREQ / freq) - 1u;
		CCR = (uint32_t)((float32_t)ARR * this->dutyCycle);

		// 2. Update ARRx register
		TIM_SetAutoreload(this->def.TIMER.TIMER, ARR);

		// 3. Update CCRx
		TIM_SetCompareX(this->def.TIMER.TIMER, CCR, this->def.TIMER.CHANNEL);

		// 4. Update instance frequency
		this->frequency = freq;
	}

	void PWM::SetState (PWM::State state)
	{
		// 1. Reset timer
		TIM_SetCounter(this->def.TIMER.TIMER, 0u);

		// 2. Set timer state
		if(state == PWM::ENABLED)
		{
			TIM_Cmd(this->def.TIMER.TIMER, ENABLE);
		}
		else
		{
			TIM_Cmd(this->def.TIMER.TIMER, DISABLE);
		}

		// 3. Update instance state
		this->state = state;
	}
}

