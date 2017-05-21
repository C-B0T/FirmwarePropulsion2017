/**
 * @file	I2CSlave.hpp
 * @author	Kevin WYSOCKI
 * @date	6 mars 2017
 * @brief	I2C Slave abstraction class
 */

#ifndef INC_I2CSLAVE_HPP_
#define INC_I2CSLAVE_HPP_

#include "I2CCommon.h"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	/**
	 * @brief I2CSlave abstraction class
	 */
	class I2CSlave
	{
	public:

		/**
		 * @brief I2C Identifier list
		 */
		enum ID
		{
			I2C_SLAVE0 = 0,//!< I2C0
			I2C_SLAVE_MAX  //!< I2C_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : I2C ID
		 * @return I2C instance
		 */
		static I2CSlave* GetInstance (enum ID id);

		/**
		 * @brief Return instance ID
		 */
		enum ID GetID ()
		{
			return this->id;
		}

		/**
		 * @brief Return current error (= 0 if no error, < 0 else)
		 */
		int32_t GetError ()
		{
			int32_t error = this->error;

			this->error = 0;

			return error;
		}

		/**
		 * @brief Send frame
		 * @param frame : I2C Frame
		 * @return Number of bytes sent or error code (if return value < 0)
		 */
		int32_t Write (I2C_FRAME * frame);

		/**
		 * @brief Read incoming frame
		 * @param frame : Buffered I2C Frame
		 * @return Error code (if < 0)
		 */
		int32_t Read (I2C_FRAME * frame);

		/**
		 * @private
		 * @brief Internal interrupt callback. DO NOT CALL !!
		 */
		void INTERNAL_InterruptCallback (uint32_t flag);

		/**
		 * @brief Event raised when slave address matched and master transmitted data (write operation)
		 */
		Utils::Event DataReceived;

		/**
		 * @brief Event raised when slave address matched and master is requesting data (read operation)
		 */
		Utils::Event DataRequest;

		/**
		 * @brief Event raised when an error occurred during a transaction
		 */
		Utils::Event ErrorOccurred;

	private:

		/**
		 * @private
		 * @brief I2C private constructor
		 * @param id : I2C identifier
		 */
		I2CSlave (enum ID id);

		/**
		 * @private
		 * @brief Instance ID
		 */
		enum ID id;

		int32_t error;

		/**
		 * @private
		 * @brief Peripheral definition
		 */
		I2C_DEF def;

		/**
		 * @private
		 * @brief Received frame buffer
		 */
		I2C_FRAMEBUFFERR buffer;

		/**
		 * @private
		 * @brief Previous CRC value
		 */
		uint8_t prevCRC;

		/**
		 * @private
		 * @brief Current CRC value
		 */
		uint8_t curCRC;
	};
}

#endif /* INC_I2CSLAVE_HPP_ */
