/**
 * @file	SPIMaster.hpp
 * @author	Kevin WYSOCKI
 * @date	24 avr. 2017
 * @brief	SPI master abstraction class
 */

#ifndef INC_SPIMasterMASTER_HPP_
#define INC_SPIMasterMASTER_HPP_

#include "stm32f4xx.h"
#include "common.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define SPI_ERROR_TIMEOUT		(-1)

/**
 * @brief SPIMaster Definition structure
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
	}CS;

	struct Pin SCK;
	struct Pin MOSI;
	struct Pin MISO;

	// I2C definitions
	struct spi
	{
		SPI_TypeDef *	BUS;
		uint32_t		CLOCKPRESCALER;
		uint16_t		CLOCKPOLARITY;
		uint16_t		CLOCKPHASE;
	}SPI;
}SPIMaster_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	/**
	 * @brief SPIMaster abstraction class
	 */
	class SPIMaster
	{
	public:

		/**
		 * @brief SPI Identifier list
		 */
		enum ID
		{
			SPI_MASTER0 = 0,//!< SPI_MASTER0
			SPI_MASTER_MAX  //!< SPI_MASTER_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : SPI ID
		 * @return SPIMaster instance
		 */
		static SPIMaster* GetInstance (enum ID id);

		/**
		 * @brief Return instance ID
		 */
		enum ID GetID()
		{
			return this->id;
		}

		/**
		 * @brief Transfer and receive n data
		 * @param txBuffer : Data to transmit buffer
		 * @param rxBuffer : Received data buffer
		 * @param length : Transfer length
		 * @return = 0 if no error, < 0 else
		 */
		int32_t Transfer (uint8_t * txBuffer, uint8_t * rxBuffer, uint32_t length);

	private:

		/**
		 * @private
		 * @brief SPIMaster private constructor
		 * @param id : SPIMaster identifier
		 */
		SPIMaster (enum ID id);

		/**
		 * @private
		 * @brief Instance identifier
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Peripheral definition
		 */
		SPIMaster_DEF def;
	};
}

#endif /* INC_SPIMasterMASTER_HPP_ */
