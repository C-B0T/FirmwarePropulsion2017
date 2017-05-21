/**
 * @file	BrushlessMotor.cpp
 * @author	Kevin WYSOCKI
 * @date	2 déc. 2016
 * @brief	BrushlessMotor class
 *
 *
 */

#include "BrushlessMotor.hpp"
#include "common.h"



using namespace HAL;
using namespace Utils;
using namespace MotionControl;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define L_MOT_ENCODER_ID	(Encoder::ENCODER0)
#define L_MOT_PID_KP		(1.0f)
#define L_MOT_PID_KI		(0.0f)
#define L_MOT_PID_KD		(0.0f)

#define R_MOT_ENCODER_ID	(Encoder::ENCODER1)
#define R_MOT_PID_KP		(1.0f)
#define R_MOT_PID_KI		(0.0f)
#define R_MOT_PID_KD		(0.0f)


#define MOT_TASK_STACK_SIZE		(64u)
#define MOT_TASK_PRIORITY		(configMAX_PRIORITIES-1)	// Highest priority task

#define MOT_TASK_EVENT_TIMER	(1u << 0u)

#define SPEED_LOOP_PERIOD_MS	(1u) /**< 1ms speed control loop */

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static BrushlessMotor* _motors[BrushlessMotor::MOTOR_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static BLMOT_DEF _getDefStructure (enum BrushlessMotor::ID id)
{
	BLMOT_DEF def;

	assert(id < BrushlessMotor::MOTOR_MAX);

	switch(id)
	{
	case BrushlessMotor::MOTOR_LEFT:
		def.Encoder.ID	=	L_MOT_ENCODER_ID;
		def.PID.kp		=	L_MOT_PID_KP;
		def.PID.ki		=	L_MOT_PID_KI;
		def.PID.kd		=	L_MOT_PID_KD;
		break;

	case BrushlessMotor::MOTOR_RIGHT:
		def.Encoder.ID	=	R_MOT_ENCODER_ID;
		def.PID.kp		=	R_MOT_PID_KP;
		def.PID.ki		=	R_MOT_PID_KI;
		def.PID.kd		=	R_MOT_PID_KD;
		break;

	default:
		break;
	}

	return def;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{
	BrushlessMotor* BrushlessMotor::GetInstance(enum ID id)
	{
		assert(id < MOTOR_MAX);

		if(_motors[id] != NULL)
		{
			return _motors[id];
		}
		else
		{
			_motors[id] = new BrushlessMotor(id);

			return _motors[id];
		}
	}

	BrushlessMotor::BrushlessMotor(enum ID id) : BrushlessMotorDriver((BrushlessMotorDriver::ID)id)
	{
		if(id == BrushlessMotor::MOTOR_LEFT)
		{
			this->name = "MOT_L";
		}
		else
		{
			this->name = "MOT_R";
		}

		this->def = _getDefStructure(id);

		// Init PID
		this->pid = PID(this->def.PID.kp,
						this->def.PID.ki,
						this->def.PID.kd,
						SPEED_LOOP_PERIOD_MS);

		// Init encoder
		this->encoder = Encoder::GetInstance(this->def.Encoder.ID);

		// Create task
		xTaskCreate((TaskFunction_t)(&BrushlessMotor::taskHandler),
					this->name.c_str(),
					MOT_TASK_STACK_SIZE,
					(void*)this,
					MOT_TASK_PRIORITY,
					&this->taskHandle);

		// Create Timer
		this->loopTimer = xTimerCreate(this->name.c_str(),
									   pdMS_TO_TICKS(SPEED_LOOP_PERIOD_MS),
									   pdTRUE,
									   (void*)this,
									   (TimerCallbackFunction_t)(&BrushlessMotor::timerCallback));
	}

	void BrushlessMotor::taskHandler(void* obj)
	{
		BrushlessMotor* instance = (BrushlessMotor*)obj;
		uint32_t event = 0;
		TickType_t prevTick = 0u,  tick = 0u;
		int32_t encoderPos = 0;
		float32_t period = 0.0f, speedFeedback = 0.0f, speed = 0.0f;

		// 1. Start loop timer
		xTimerStart(instance->loopTimer, 0u);

		// 2. Get tick count
		prevTick = xTaskGetTickCount();

		while(1)
		{
			// 2. Wait until timer elapsed
			if(xTaskNotifyWait(0,
							   0xFFFFFFFF,
							   &event,
							   portMAX_DELAY) == pdTRUE)
			{
				if(event == MOT_TASK_EVENT_TIMER)
				{
					// 3. Get tick
					tick = xTaskGetTickCount();

					period = (float32_t)tick - (float32_t)prevTick;

					//4. Get encoder value
					encoderPos = instance->encoder->GetRelativeValue();

					// 5. Get speed in encoder_tick/os_tick
					speedFeedback = (float32_t)((float32_t)encoderPos / period);

					// 6. Compute PID from speed feedback
					speed = instance->pid.Get(speedFeedback, period);

					// 7. Set speed
					instance->SetSpeed(speed);

					// 8. Set previous tick
					prevTick = tick;
				}
			}
		}
	}

	void BrushlessMotor::timerCallback(TimerHandle_t handle)
	{
		BrushlessMotor* instance = (BrushlessMotor*)pvTimerGetTimerID(handle);

		xTaskNotify(instance->taskHandle,
					MOT_TASK_EVENT_TIMER,
					eSetValueWithOverwrite);
	}
}
