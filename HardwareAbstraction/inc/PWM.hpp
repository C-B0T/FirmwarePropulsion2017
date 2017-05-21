/**
 * @file	PWM.hpp
 * @author	Kevin WYSOCKI
 * @date	14 nov. 2016
 * @brief	PWM Abstraction class
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include "stm32f4xx.h"
#include "common.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief PWM Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	// IO definitions
	struct Pin
	{
		GPIO_TypeDef *	PORT;
		uint16_t		PIN;
		uint8_t			PINSOURCE;
		uint8_t			AF;
	}IO;

	// Timer definitions
	struct Timer
	{
		TIM_TypeDef *	TIMER;
		uint16_t		CHANNEL;
		uint32_t		CLOCKFREQ;
	}TIMER;

	//PWM definitions
	struct pwm
	{
		uint32_t 	DEFAULT_FREQ;
		float32_t 	DEFAULT_DUTYCYCLE;
	}PWM;
}PWM_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/
/**
 * @namespace HAL
 */
namespace HAL
{
	/**
	 * @class PWM
	 * @brief PWM Abstraction Class
	 *
	 * HOWTO :
	 * - Get PWM instance with PWM::GetInstance()
	 * - Use SetDutyCycle() methods to update PWM duty cycle
	 * - Use SetFrequency() method to update PWM frequency
	 * - Use SetState() method to enable or disable PWM
	 */
	class PWM
	{
	public:

		/**
		 * @brief PWM Identifier list
		 */
		enum ID
		{
			PWM0,  //!< PWM_OUT0
			PWM1,  //!< PWM_OUT2
			PWM_MAX//!< PWM_MAX
		};

		/**
		 * @brief PWM State
		 */
		enum State
		{
			ENABLED,//!< ENABLED
			DISABLED//!< DISABLED
		};

		/**
		 * @brief Get instance method
		 * @param id : PWM ID
		 * @return PWM instance
		 */
		static PWM* GetInstance (enum ID id);

		/**
		 * @brief Return instance ID
		 */
		enum ID GetID ()
		{
			return this->id;
		}

		/**
		 * @brief Set PWM duty cycle
		 * @param percent : Duty cycle in percent
		 */
		void SetDutyCycle (float32_t percent);

		/**
		 * @brief Return current PWM duty cycle
		 */
		float32_t GetDutyCycle ()
		{
			return this->dutyCycle;
		}

		/**
		 * @brief Set PWM frequency
		 * @param freq : PWM frequency in Hz
		 */
		void SetFrequency (uint32_t freq);

		/**
		 * @brief Return current PWM frequency
		 */
		uint32_t GetFrequency ()
		{
			return this->frequency;
		}

		/**
		 * @brief Set PWM state
		 * @param state : PWM State
		 */
		void SetState (PWM::State state);

		/**
		 * @brief Return current PWM state
		 */
		PWM::State GetState ()
		{
			return this->state;
		}

	private:

		/**
		 * @private
		 * @brief PWM private constructor
		 * @param id : PWM identifier
		 */
		PWM (PWM::ID id);

		/**
		 * @private
		 * @brief Instance ID
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Peripheral definition
		 */
		PWM_DEF def;

		/**
		 * @private
		 * @brief Duty cycle
		 */
		float32_t dutyCycle;

		/**
		 * @private
		 * @brief Frequency
		 */
		uint32_t frequency;

		/**
		 * @private
		 * @brief State
		 */
		PWM::State state;
	};
}

#endif /* INC_PWM_HPP */
