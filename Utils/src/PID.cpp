/**
 * @file	PID.cpp
 * @author	Kevin WYSOCKI
 * @date	18 nov. 2016
 * @brief	PID Controller class
 */

#include "PID.hpp"

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Utils
{
	PID::PID()
	{
		this->kp		=	0.0f;
		this->ki		=	0.0f;
		this->kd		=	0.0f;
		this->dt		=	1.0f;
		this->err		=	0.0f;
		this->intErr	=	0.0f;
		this->diffErr	=	0.0f;
		this->setpoint	=	0.0f;
		this->output	=	0.0f;
	}

	PID::PID(float32_t kp, float32_t ki, float32_t kd, float32_t dt) : PID()
	{
		this->kp	=	kp;
		this->ki	=	ki;
		this->kd	=	kd;
		this->dt	=	dt;

	}

	void PID::Reset ()
	{
		this->err		=	0.0f;
		this->intErr	=	0.0f;
		this->diffErr	=	0.0f;
	}

	float32_t PID::Get (float32_t feedback)
	{
		float32_t err = 0.0;

		err = (this->setpoint - feedback) / 1.0;	// Filter output removed


		this->intErr	=	err + this->intErr;
		this->diffErr	=	err - this->err;
		this->err 		= 	err;

		// Output    =  kp * current error    + Ki * error sum * integration time    + kd * differential error / derivation time
		this->output = (this->kp * this->err) + (this->ki * this->intErr * this->dt) + (this->kd * this->diffErr / this->dt);

		return this->output;
	}

	float32_t PID::Get(float32_t feedback, float32_t period)
	{
		this->dt = period;

		return this->Get(feedback);
	}
}
