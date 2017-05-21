/**
 * @file	Communication.hpp
 * @author	Kevin WYSOCKI
 * @date	20 avr. 2017
 * @brief	Communication Handler class
 */

#ifndef INC_COMMUNICATIONHANDLER_HPP_
#define INC_COMMUNICATIONHANDLER_HPP_

#include "common.h"
#include "I2CSlave.hpp"
#include "Message.hpp"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{
	/**
	 * @brief Communication Handler class
	 */
	class CommunicationHandler
	{
	public :

		/**
		 * @brief Return CommunicationHandler instance
		 */
		static CommunicationHandler * GetInstance ();

		/**
		 * @brief Return current error;
		 */
		int32_t GetError()
		{
			int32_t error = this->error;

			this->error = NO_ERROR;

			return error;
		}

		/**
		 * @brief Send data after a data request
		 * @param msg : Message to send
		 * @return Nb of data bytes sent if >= 0, error if < 0
		 */
		int32_t Write (Message * msg);

		/**
		 * @brief Retrieve last received message
		 * @param msg : Last received message
		 */
		void GetLastMessage (Message * msg);

		/**
		 * @brief Message received event
		 */
		Utils::Event MessageReceived;

		/**
		 * @brief Error occured event
		 */
		Utils::Event ErrorOccured;

		/**
		 * @private
		 * @brief Communication task handler
		 */
		void TaskHandler (void);

	private :

		/**
		 * @private
		 * @brief Private constructor
		 */
		CommunicationHandler();

		/**
		 * @private
		 * @brief Current message
		 */
		Message msg;

		int32_t error;

		/**
		 * @private
		 * @brief Current frame
		 */
		I2C_FRAME frame;

		/**
		 * @private
		 * @brief I2C bus
		 */
		HAL::I2CSlave * bus;
	};
}

#endif /* INC_COMMUNICATIONHANDLER_HPP_ */
