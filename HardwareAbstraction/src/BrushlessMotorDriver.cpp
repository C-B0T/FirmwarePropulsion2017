/**
 * @file	BrushlessMotorDriver.cpp
 * @author	Kevin
 * @date	1 d�c. 2016
 * @brief	L6235 Brushless Motor Driver class
 */

#include "BrushlessMotorDriver.hpp"

using namespace Utils;
using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define DRV0_BRAKE_GPIO_ID		(GPIO::GPIO2)
#define DRV0_DIR_GPIO_ID		(GPIO::GPIO1)
#define DRV0_DIAG_GPIO_ID		(GPIO::GPIO0)
#define DRV0_ENABLE_PWM_ID		(PWM::PWM0)

#define DRV1_BRAKE_GPIO_ID		(GPIO::GPIO5)
#define DRV1_DIR_GPIO_ID		(GPIO::GPIO4)
#define DRV1_DIAG_GPIO_ID		(GPIO::GPIO3)
#define DRV1_ENABLE_PWM_ID		(PWM::PWM1)

#define ENABLE_PWM_FREQ			(10000u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static BrushlessMotorDriver* _drivers[BrushlessMotorDriver::DRIVER_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Get hardware definition structure
 * @param id : BrushlessMotorDriver identifier
 */
static BLMOTDRV_DEF _getDefStructure (BrushlessMotorDriver::ID id)
{
	BLMOTDRV_DEF def;

	assert(id < BrushlessMotorDriver::DRIVER_MAX);

	switch(id)
	{
	case BrushlessMotorDriver::DRIVER0:
		def.BrakePinID	=	DRV0_BRAKE_GPIO_ID;
		def.DirPinID	=	DRV0_DIR_GPIO_ID;
		def.DiagPinID	=	DRV0_DIAG_GPIO_ID;
		def.EnablePinID	=	DRV0_ENABLE_PWM_ID;
		break;

	case BrushlessMotorDriver::DRIVER1:
		def.BrakePinID	=	DRV1_BRAKE_GPIO_ID;
		def.DirPinID	=	DRV1_DIR_GPIO_ID;
		def.DiagPinID	=	DRV1_DIAG_GPIO_ID;
		def.EnablePinID	=	DRV1_ENABLE_PWM_ID;
		break;

	default:
		break;
	}

	return def;
}

static void _diagStateChanged (void * obj)
{
	BrushlessMotorDriver* instance = static_cast<BrushlessMotorDriver*>(obj);

	instance->OverCurrentDetected();
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	BrushlessMotorDriver* BrushlessMotorDriver::GetInstance(BrushlessMotorDriver::ID id)
	{
		assert(id < BrushlessMotorDriver::DRIVER_MAX);

		// if instance already exists
		if(_drivers[id] != NULL)
		{
			return _drivers[id];
		}
		else
		{
			// Create instance
			_drivers[id] = new BrushlessMotorDriver(id);

			return _drivers[id];
		}
	}

	BrushlessMotorDriver::BrushlessMotorDriver(BrushlessMotorDriver::ID id)
	{
		this->id = id;
		this->speed = 0.0f;
		this->direction = BrushlessMotorDriver::FORWARD;
		this->def = _getDefStructure(id);

		// Get brake pin instance
		this->brakePin = GPIO::GetInstance(this->def.BrakePinID);
		this->brakePin->Set(GPIO::Low);

		// Get dir pin instance
		this->dirPin = GPIO::GetInstance(this->def.DirPinID);
		this->dirPin->Set(GPIO::Low);

		// Get enable pin instance
		this->enablePin = PWM::GetInstance(this->def.EnablePinID);
		this->enablePin->SetFrequency(ENABLE_PWM_FREQ);
		this->enablePin->SetDutyCycle(0.0f);

		// Get diag pin instance
		this->diagPin = GPIO::GetInstance(this->def.DiagPinID);
		this->diagPin->StateChanged.Subscribe(this, _diagStateChanged);
	}

	void BrushlessMotorDriver::SetMotorSpeed(float32_t percent)
	{
		// Limits
		if(percent > 1.0f)
		{
			percent = 1.0f;
		}
		//else if(percent < -1.0f)
		else if(percent < -1.0f)
		{
			percent = -1.0f;
		}

		// Simple speed
		if(percent > 0.0f)
		{
			this->SetDirection(BrushlessMotorDriver::FORWARD);
			this->SetSpeed(percent);
			this->Move();
		}
		else if(percent < 0.0f)
		{
			this->SetDirection(BrushlessMotorDriver::REVERSE);
			this->SetSpeed(-percent);
			this->Move();
		}
		else
		{
			this->SetSpeed(0.0f);
			this->Freewheel();
		}
	}

	void BrushlessMotorDriver::SetSpeed(float32_t percent)
	{
		if(percent > 1.0f)
		{
			percent = 1.0f;
		}
		else if (percent < 0.0f)
		{
			percent = 0.0f;
		}

		this->speed = percent;

		this->enablePin->SetDutyCycle(percent);
	}

	void BrushlessMotorDriver::SetDirection(enum BrushlessMotorDriver::Direction direction)
	{
		if(direction == BrushlessMotorDriver::FORWARD)
		{
			this->dirPin->Set(GPIO::High);
		}
		else
		{
			this->dirPin->Set(GPIO::Low);
		}
	}

	void BrushlessMotorDriver::Brake()
	{
		// Make sure the ENABLE pin is high, unless H-Bridge is disabled
		this->enablePin->SetDutyCycle(1.0f);
		this->enablePin->SetState(PWM::ENABLED);

		// Turn on all high-side MOSFET
		this->brakePin->Set(GPIO::Low);
	}

	void BrushlessMotorDriver::Move()
	{
		// Set PWM duty cycle with last speed value
		this->enablePin->SetDutyCycle(this->speed);
		this->enablePin->SetState(PWM::ENABLED);

		// Turn off braking function
		this->brakePin->Set(GPIO::High);

	}

	void BrushlessMotorDriver::Freewheel()
	{
		// Disable h-bridge
		this->enablePin->SetDutyCycle(0.0f);
		this->enablePin->SetState(PWM::DISABLED);

		// Turn off braking function
		this->brakePin->Set(GPIO::High);

	}
}
