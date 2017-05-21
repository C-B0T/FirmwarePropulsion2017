/**
 * @file	BrushlessMotorDriver.hpp
 * @author	Kevin WYSOCKI
 * @date	1 d�c. 2016
 * @brief	L6235 Brushless Motor Driver class
 */

#ifndef INC_BRUSHLESSMOTORDRIVER_HPP_
#define INC_BRUSHLESSMOTORDRIVER_HPP_

#include "PWM.hpp"
#include "GPIO.hpp"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef struct
{
	HAL::GPIO::ID	BrakePinID;
	HAL::GPIO::ID	DirPinID;
	HAL::GPIO::ID	DiagPinID;
	HAL::PWM::ID	EnablePinID;
}BLMOTDRV_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace HAL
 */
namespace HAL
{
	/**
	 * @class BrushlessMotorDriver
	 * @brief L6235 brushless motor driver abstraction class
	 */
	class BrushlessMotorDriver
	{
	public:

		/**
		 * @brief Motor Driver Identifier
		 */
		enum ID
		{
			DRIVER0 = 0,	//!< LEFT_MOTOR
			DRIVER1,		//!< RIGHT_MOTOR
			DRIVER_MAX   	//!< MOTOR_MAX
		};

		/**
		 * @brief Motor Direction
		 */
		enum Direction
		{
			FORWARD,//!< FORWARD
			REVERSE//!< REVERSE
		};

		/**
		 * @brief Get instance method
		 * @param id : Brushless motor driver identifier
		 * @return BrushlessMotorDriver instance
		 */
		static BrushlessMotorDriver* GetInstance (enum ID id);

		/**
		 * @brief Set motor speed in percent (-1.0 to 1.0)
		 * @param percent : Motor speed in percent
		 */
		void SetMotorSpeed (float32_t percent);

		/**
		 * @brief Set motor speed in percent (0.0 to 1.0)
		 * @param percent : Motor speed in percent
		 */
		void SetSpeed (float32_t percent);

		/**
		 * @brief Return motor current speed
		 */
		float32_t GetSpeed ()
		{
			return this->speed;
		}

		/**
		 * @brief Set motor driver direction
		 */
		void SetDirection (enum Direction direction);

		/**
		 * @brief Return current direction
		 */
		enum Direction GetDirection ()
		{
			return this->direction;
		}

		/**
		 * @brief Brake motor
		 */
		void Brake ();

		/**
		 * @brief Move motor
		 */
		void Move ();

		/**
		 * @brief Stop motor - Free wheel
		 */
		void Freewheel ();

		Utils::Event OverCurrentDetected;

	protected :

		/**
		 * @protected
		 * @brief Private constructor
		 * @param id : BrushlessMotorDriver identifier
		 */
		BrushlessMotorDriver (enum ID id);

		/**
		 * @protected
		 * @brief Motor driver identifier
		 */
		enum ID id;

	private:

		/**
		 * @private
		 * @brief Current speed
		 */
		float32_t speed;

		/**
		 * @private
		 * @brief Current direction
		 */
		enum Direction direction;

		/**
		 * @private
		 * @brief nBRAKE input
		 */
		GPIO* brakePin;

		/**
		 * @private
		 * @brief FWD/REV input
		 */
		GPIO* dirPin;

		/**
		 * @brief EN input
		 */
		PWM* enablePin;

		/**
		 * @brief DIAG output
		 */
		GPIO* diagPin;

		/**
		 * @brief Hardware related elements
		 */
		BLMOTDRV_DEF	def;
	};
}

#endif /* INC_BRUSHLESSMOTORDRIVER_HPP_ */
