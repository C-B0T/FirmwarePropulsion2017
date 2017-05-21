/**
 * @file	BrushlessMotor.hpp
 * @author	Kevin WYSOCKI
 * @date	2 déc. 2016
 * @brief	BrushlessMotor class
 */

#ifndef INC_BRUSHLESSMOTOR_HPP_
#define INC_BRUSHLESSMOTOR_HPP_

#include "HAL.hpp"
#include "Utils.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef struct
{
	// Encoder
	struct
	{
		HAL::Encoder::ID	ID;
	}Encoder;

	// PID
	struct
	{
		float32_t	kp;
		float32_t 	ki;
		float32_t	kd;
	}PID;
}BLMOT_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
	/**
	 * @class BrushlessMotor
	 * @brief Brushless motor control class
	 *
	 * Derivated from BrushlessMotorDriver class, includes speed regulation from encoder speed feedback and PID controller
	 */
	class BrushlessMotor : public HAL::BrushlessMotorDriver
	{
	public:

		enum ID
		{
			MOTOR_LEFT = 0,
			MOTOR_RIGHT,
			MOTOR_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : Brushless motor ID
		 * @return BrushlessMotor instance
		 */
		static BrushlessMotor* GetInstance (enum ID id);

		/**
		 * @brief Return instance name
		 */
		std::string Name()
		{
			return this->name;
		}

		/** @todo Add motion profile */
		//void SetMotionProfile (MotionProfile& profile);

	private:

		/**
		 * @private
		 * @brief Private constructor
		 * @param id : BrushlessMotor identifier
		 */
		BrushlessMotor (enum ID id);

		/**
		 * @private
		 * @brief Instance name
		 */
		std::string name;

		/**
		 * @private
		 * @brief Speed PID controller
		 */
		Utils::PID	pid;

		/**
		 * @private
		 * @brief Wheel encoder
		 */
		HAL::Encoder* encoder;

		/** @todo Add motion profile */
		// MotionProfile profile;

		/**
		 * @private
		 * @brief Hardware definitions
		 */
		BLMOT_DEF def;

		/**
		 * @private
		 * @brief OS Task handle
		 *
		 * Used by speed control loop
		 */
		TaskHandle_t taskHandle;

		/**
		 * @private
		 * @brief OS Timer Handle
		 *
		 * Used by speed control loop
		 */
		TimerHandle_t loopTimer;

		/**
		 * @private
		 * @brief Speed control loop task handler
		 * @param obj : Always NULL
		 */
		void taskHandler (void* obj);

		/**
		 * @private
		 * @brief Instance timer callback
		 * @param handle : OS Timer handle
		 */
		void timerCallback (TimerHandle_t handle);
	};
}

#endif /* INC_BRUSHLESSMOTOR_HPP_ */
