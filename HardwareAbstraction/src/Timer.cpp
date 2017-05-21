/**
 * @file	Timer.cpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Timer Abstraction Class
 */

#include <stddef.h>
#include "Timer.hpp"
#include "common.h"

using namespace HAL;
using namespace Utils;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define TIMER_COUNTER_MAX_VALUE (65535)	/**< 16-bit timer max value */

// TIM4
#define TIMER0_TIMER			(TIM4)
#define TIMER0_PERIOD_US		(10000u)
#define TIMER0_FREQUENCY		(1000000/TIMER0_PERIOD_US)
#define TIMER0_TIMER_FREQ		(SystemCoreClock / 2)	// TIM4 clock is derivated from APB1 clock
#define TIMER0_INT_CHANNEL		(TIM4_IRQn)
#define TIMER0_INT_PRIORITY		(0u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief Timer instances
 */
Timer* _timer[Timer::TIMER_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve Timer definitions from Timer ID
 * @param id : Timer ID
 * @return TIM_DEF structure
 */
static TIM_DEF _getTimerStruct (enum Timer::ID id)
{
	TIM_DEF tim;

	assert(id < HAL::Timer::TIMER_MAX);

	switch(id)
	{
	case HAL::Timer::TIMER0:
		tim.TIMER.TIMER		=	TIMER0_TIMER;
		tim.TIMER.PERIOD	=	TIMER0_PERIOD_US;
		tim.TIMER.FREQ		=	TIMER0_FREQUENCY;
		tim.TIMER.CLOCKFREQ	=	TIMER0_TIMER_FREQ;
		tim.INT.CHANNEL		=	TIMER0_INT_CHANNEL;
		tim.INT.PRIORITY	=	TIMER0_INT_PRIORITY;
		break;
	default:
		break;
	}

	return tim;
}

/**
 * @brief Initialize peripheral for a specified Timer
 * @param id : Timer ID
 */
static void _hardwareInit (enum Timer::ID id)
{
	TIM_TimeBaseInitTypeDef TIMBaseStruct;
	NVIC_InitTypeDef NVICStruct;

	TIM_DEF tim;
	uint32_t minFrequency = 0, prescaler = 0u;

	assert(id < HAL::Timer::TIMER_MAX);

	tim = _getTimerStruct(id);

	// TIMER Init
	TIMBaseStruct.TIM_ClockDivision		=	TIM_CKD_DIV1;
	TIMBaseStruct.TIM_CounterMode		=	TIM_CounterMode_Up;
	TIMBaseStruct.TIM_RepetitionCounter	=	0u;

	minFrequency = tim.TIMER.CLOCKFREQ / (TIMER_COUNTER_MAX_VALUE + 1u);

	if(tim.TIMER.FREQ > minFrequency)
	{
		TIMBaseStruct.TIM_Prescaler	= 	0u;
		TIMBaseStruct.TIM_Period	=	(tim.TIMER.CLOCKFREQ / tim.TIMER.FREQ) - 1u;
	}
	else
	{
		prescaler = (minFrequency / tim.TIMER.FREQ) + 1u;
		TIMBaseStruct.TIM_Prescaler	= 	prescaler - 1u;
		TIMBaseStruct.TIM_Period	=	((tim.TIMER.CLOCKFREQ / prescaler) / tim.TIMER.FREQ) - 1u;
	}

	TIM_Cmd(tim.TIMER.TIMER, DISABLE);
	TIM_TimeBaseInit(tim.TIMER.TIMER, &TIMBaseStruct);
	TIM_SetCounter(tim.TIMER.TIMER, 0u);
	TIM_ARRPreloadConfig(tim.TIMER.TIMER, ENABLE);

	TIM_ClearFlag(tim.TIMER.TIMER, TIM_FLAG_Update);
	TIM_ITConfig(tim.TIMER.TIMER, TIM_IT_Update, ENABLE);

	// Init NVIC
	NVICStruct.NVIC_IRQChannel						=	tim.INT.CHANNEL;
	NVICStruct.NVIC_IRQChannelPreemptionPriority 	= 	tim.INT.PRIORITY;
	NVICStruct.NVIC_IRQChannelSubPriority 			= 	0;
	NVICStruct.NVIC_IRQChannelCmd					=	ENABLE;

	NVIC_Init(&NVICStruct);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/
namespace HAL
{
	Timer* Timer::GetInstance (enum Timer::ID id)
	{
		assert(id < Timer::TIMER_MAX);

		// if Timer instance already exists
		if(_timer[id] != NULL)
		{
			return _timer[id];
		}
		else
		{
			// Create Timer instance
			_timer[id] = new Timer(id);

			return _timer[id];
		}
	}

	Timer::Timer (enum Timer::ID id)
	{
		this->id = id;
		this->def = _getTimerStruct(id);
		this->period_us = this->def.TIMER.PERIOD;

		_hardwareInit(id);
	}

	void Timer::SetPeriod(uint32_t period_us)
	{
		TIM_TimeBaseInitTypeDef TIMBaseStruct;
		uint32_t prescaler = 0u;
		float32_t frequency = 0.0f, minFrequency = 0.0f;

		frequency = 1000000.0 / (float32_t)period_us;

		TIMBaseStruct.TIM_ClockDivision		=	TIM_CKD_DIV1;
		TIMBaseStruct.TIM_CounterMode		=	TIM_CounterMode_Up;
		TIMBaseStruct.TIM_RepetitionCounter	=	0u;

		minFrequency = this->def.TIMER.CLOCKFREQ / (TIMER_COUNTER_MAX_VALUE + 1u);

		if(frequency > minFrequency)
		{
			TIMBaseStruct.TIM_Prescaler	= 	0u;
			TIMBaseStruct.TIM_Period	=	(this->def.TIMER.CLOCKFREQ / frequency) - 1u;
		}
		else
		{
			prescaler = (uint32_t)((minFrequency / frequency) + 1.0f);
			TIMBaseStruct.TIM_Prescaler	= 	prescaler - 1u;
			TIMBaseStruct.TIM_Period	=	((this->def.TIMER.CLOCKFREQ / prescaler) / frequency) - 1u;
		}

		TIM_TimeBaseInit(this->def.TIMER.TIMER, &TIMBaseStruct);
		TIM_SetCounter(this->def.TIMER.TIMER, 0u);
	}

	void Timer::Restart()
	{
		TIM_SetCounter(this->def.TIMER.TIMER, 0u);
		TIM_Cmd(this->def.TIMER.TIMER, ENABLE);
	}

	void Timer::Start()
	{
		TIM_Cmd(this->def.TIMER.TIMER, ENABLE);
	}

	void Timer::Stop()
	{
		TIM_Cmd(this->def.TIMER.TIMER, DISABLE);
	}

	void Timer::INTERNAL_InterruptCallback(uint16_t flag)
	{
		if(flag == TIM_FLAG_Update)
		{
			this->TimerElapsed();
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Interrupt Handler                                                          */
/*----------------------------------------------------------------------------*/

extern "C"
{
	/**
	 * @brief TIM4 Interrupt Handler
	 */
	void TIM4_IRQHandler(void)
	{
		if(TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == SET)
		{
			TIM_ClearFlag(TIM4, TIM_FLAG_Update);

			Timer* tim = Timer::GetInstance(Timer::TIMER0);

			tim->INTERNAL_InterruptCallback(TIM_FLAG_Update);
		}
	}
}
