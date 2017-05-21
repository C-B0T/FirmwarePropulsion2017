/**
 * @file	Serial.hpp
 * @author	Kevin WYSOCKI
 * @date	9 nov. 2016
 * @brief	Serial Communication Abstraction Class
 */

#ifndef INC_SERIAL_HPP_
#define INC_SERIAL_HPP_

#include "stm32f4xx.h"
#include <stdint.h>
#include <string>
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief USART Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	// RX Pin definitions
	struct Pin
	{
		GPIO_TypeDef *	PORT;
		uint16_t		PIN;
		uint8_t			PINSOURCE;
		uint8_t			AF;
	}RX;

	// TX Pin definitions
	struct Pin TX;

	// UART port definitions
	struct Usart
	{
		USART_TypeDef * PORT;
		uint32_t		BAUDRATE;
	}USART;

	struct Int
	{
		uint8_t	PRIORITY;		/**< Interrupt priority, 0 to 15, 0 is the highest priority */
		uint8_t	CHANNEL;		/**< Interrupt IRQ Channel */
	}INT;


}SERIAL_DEF;

/*----------------------------------------------------------------------------*/
/* Class										                              */
/*----------------------------------------------------------------------------*/

/**
 * @namespace HAL
 */
namespace HAL
{
	/**
	 * @class Serial
	 * @brief Serial Communication Abstraction Class
	 *
	 * HOWTO :
	 * - Get Serial instance with Serial::GetInstance()
	 * - Use Send() methods to send data
	 * - Use Read() to peek one or more data
	 * - OnDataReceivedCallback or OnEndOfTransmissionCallback have to be used to
	 * be notified of data received or end of transmission event. No data can be send if
	 * there is already data being sent.
	 */
	class Serial
	{
	public:

		/**
		 * @brief Serial Identifier list
		 */
		enum ID
		{
			SERIAL0,  //!< UART1
			SERIAL1,  //!< UART3
			SERIAL_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : Serial ID
		 * @return Serial instance
		 */
		static Serial* GetInstance (enum ID id);

		/**
		 * @brief Return Serial ID
		 * @return ID
		 */
		enum ID GetID ()
		{
			return this->id;
		}

		/**
		 * @brief Return the number of buffered bytes read to be read
		 */
		uint32_t BytesToRead()
		{
			return (this->rxBuffer.wrIndex - this->rxBuffer.wrIndex);
		}

		/**
		 * @brief Return the number of buffered bytes left to be transmitted
		 */
		uint32_t BytesToSend()
		{
			return (this->txBuffer.wrIndex - this->rxBuffer.rdIndex);
		}

		/**
		 * @brief Send a single byte
		 * @param byte : Byte to send
		 * @return true if byte have been buffered, false else
		 */
		bool Send (uint8_t byte);

		/**
		 * @brief Send a string
		 * @param str : string to send
		 * @return true if string has been buffered to be sent, false else
		 */
		bool Send (std::string& str);

		/**
		 * @brief Send a C-type string (NULL terminated)
		 * @param c_str : C-type string
		 * @return true if string has been buffered to be sent, false else
		 */
		bool Send (const char * c_str);

		/**
		 * @brief Send bytes from buffer
		 * @param buffer : Bytes to send buffer
		 * @param length : Number of bytes to send
		 * @return	true if bytes have been buffered, false else
		 */
		bool Send (const uint8_t * buffer, uint32_t length);

		/**
		 * @brief Read one buffered bytes
		 * @return Next byte to read if more than one byte buffered, last received byte else
		 */
		uint8_t Read ();

		/**
		 * @brief Read a few buffered bytes
		 * @param buffer : Buffer where bytes are stored
		 * @param length : Number of bytes to read
		 *
		 * If not enough bytes have been buffered, length contains the number of bytes actually read.
		 */
		void Read (uint8_t * buffer, uint32_t& length);

		/**
		 * @brief Read a '\n' terminated string
		 * @return string instance if a character '\n' is found, NULL else
		 */
		std::string ReadLine ();

		/**
		 * @brief Data received event
		 */
		Utils::Event DataReceived;

		/**
		 * @brief End of transmission event
		 */
		Utils::Event EndOfTransmission;

		/**
		 * @private
		 * @brief Internal interrupt callback. DO NOT CALL !!
		 * @param flag : interrupt flag
		 */
		void INTERNAL_InterruptCallback (uint16_t flag);

	private:

		/**
		 * @private
		 * @brief Rx/Tx data buffer
		 */
		typedef struct
		{
			uint8_t * data;		/**< FIFO pointer) */
			uint32_t wrIndex;	/**< FIFO write index */
			uint32_t rdIndex;	/**< FIFO read index */
		}SERIAL_BUFFER;

		/**
		 * @private
		 * @brief Serial private constructor
		 * @param id : Serial ID
		 */
		Serial (enum ID id);

		/**
		 * @private
		 * @brief Instance identifier
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Peripheral definitions
		 */
		SERIAL_DEF def;

		/**
		 * @private
		 * @brief RX FIFO
		 */
		SERIAL_BUFFER rxBuffer;

		/**
		 * @private
		 * @brief TX FIFOs
		 */
		SERIAL_BUFFER txBuffer;
	};
}


#endif /* INC_SERIAL_HPP_ */
