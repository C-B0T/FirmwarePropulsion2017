/**
 * @file	Timer.hpp
 * @author	Kevin WYSOCKI
 * @date	24 nov. 2016
 * @brief	Timer Abstraction Class
 */

#ifndef INC_TIMER_HPP_
#define INC_TIMER_HPP_

#include "stm32f4xx.h"
#include "common.h"

#include "Observable.hpp"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief Timer Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	// Timer definitions
	struct Timer
	{
		TIM_TypeDef *	TIMER;
		uint32_t 		PERIOD;
		uint32_t		FREQ;
		uint32_t		CLOCKFREQ;
	}TIMER;

	// Interrupt definitions
	struct Interrupt
	{
		uint8_t		PRIORITY;		/**< Interrupt priority, 0 to 15, 0 is the highest priority */
		uint8_t		CHANNEL;		/**< Interrupt IRQ Channel */
	}INT;
}TIM_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace HAL
 */
namespace HAL
{
	/**
	 * @class Timer
	 * @brief Timer Abstraction Class
	 *
	 * HOWTO:
	 *  - Get a timer instance with GetInstance() method
	 *  - Set timer period with SetPeriod()
	 *  - Start, stop or restart timer with Start(), Stop(), Restart() methods
	 *  - Register to Timer Elapsed event by add your callback to TimerElapsed Event
	 */
	class Timer //: protected Utils::Observable
	{
	public:

		/**
		 * @brief Timer identifier list
		 */
		enum ID
		{
			TIMER0,  //!< TIMER0
			TIMER_MAX//!< TIMER_MAX
		};

		/**
		 * @brief Return a timer instance
		 * @param id : Timer identifier
		 * @return Timer instance
		 */
		static Timer* GetInstance (enum ID id);

		/**
		 * @brief Return Timer identifier
		 */
		enum ID GetID()
		{
			return this->id;
		}

		/**
		 * @brief Set Timer period
		 * @param period_us : Timer period in microseconds
		 */
		void SetPeriod (uint32_t period_us);

		/**
		 * @brief Reset timer and start it
		 */
		void Restart();

		/**
		 * @brief Start timer where it was previously stopped
		 */
		void Start ();

		/**
		 * @brief Stop timer
		 */
		void Stop ();

		/**
		 * @brief Timer elapsed event;
		 * Add your callback to this event to be notified when Timer elapses
		 */
		Utils::Event TimerElapsed;

		/**
		 * @private
		 * @brief Internal interrupt callback. DO NOT CALL !!
		 * @param flag : interrupt flag
		 */
		void INTERNAL_InterruptCallback (uint16_t flag);

	private:

		/**
		 * @private
		 * @brief Timer constructor
		 * @param id : Timer identifier
		 */
		Timer (enum ID id);

		/**
		 * @private
		 * @brief Timer identifier
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Timer definitions
		 */
		TIM_DEF def;

		/**
		 * @private
		 * @brief Timer period in microseconds
		 */
		uint32_t period_us;
	};
}


#endif /* INC_TIMER_HPP_ */
