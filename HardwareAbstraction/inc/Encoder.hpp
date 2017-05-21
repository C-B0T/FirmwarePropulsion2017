/**
 * @file	Encoder.hpp
 * @author	Kevin WYSOCKI
 * @date	16 nov. 2016
 * @brief	Encoder abstraction class
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stm32f4xx.h"


/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief Encoder Definition structure
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
	}CH_A;

	struct Pin CH_B;

	// Timer definitions
	struct Timer
	{
		TIM_TypeDef *	TIMER;
		uint16_t		RELOAD_VAL;
	}TIMER;

	// Interrupt definitions
	struct Interrupt
	{
		uint8_t		PRIORITY;		/**< Interrupt priority, 0 to 15, 0 is the highest priority */
		uint8_t		CHANNEL;		/**< Interrupt IRQ Channel */
	}INT;

}ENC_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace HAL
 */
namespace HAL
{
	/**
	 * @class Encoder
	 * @brief Encoder abstraction class
	 *
	 * HOWTO :
	 * - Get Encoder instance with Encoder::GetInstance()
	 * - Use GetAbsoluteValue() and GetRelativeValue() methods to know
	 *   absolute or relative (since last call) position
	 */
	class Encoder
	{
	public:

		/**
		 * @brief Encoder Identifier List
		 */
		enum ID
		{
			ENCODER0,  //!< ENCODER0
			ENCODER1,  //!< ENCODER1
			ENCODER_MAX//!< ENCODER_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : Encoder ID
		 * @return Encoder
		 *  instance
		 */
		static Encoder* GetInstance (Encoder::ID id);

		/**
		 * @brief Reset position and turn counter
		 */
		void Reset()
		{
			this->turnCounter 	= 	0;
			this->absolutePos 	= 	0;
			this->relativePos 	= 	0;
			this->prevAbsValue 	= 	0;
		}

		/**
		 * @brief Return encoder absolute values
		 */
		int64_t GetAbsoluteValue ();

		/**
		 * @brief Return encoder value since last retrieval
		 */
		int32_t GetRelativeValue ();

		/**
		 * @private
		 * @brief Internal interrupt callback. DO NOT CALL !!
		 */
		void INTERNAL_InterruptCallback (uint8_t flag);

	private:

		/**
		 * @private
		 * @brief Encoder private constructor
		 * @param id : Encoder identifier
		 */
		Encoder(Encoder::ID id);

		/**
		 * @private
		 * @brief Encoder turn counter
		 */
		int32_t turnCounter;

		/**
		 * @private
		 * @brief Timer previous absolute value
		 */
		int64_t prevAbsValue;

		/**
		 * @private
		 * @brief Encoder absolute position
		 */
		int64_t absolutePos;

		/**
		 * @private
		 * @brief Encoder relative position
		 */
		int32_t relativePos;

		/**
		 * @private
		 * @brief Peripheral definitions
		 */
		ENC_DEF def;
	};
}

#endif /* INC_ENCODER_HPP_ */
