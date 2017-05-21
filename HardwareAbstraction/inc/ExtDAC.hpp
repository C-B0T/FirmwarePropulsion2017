/**
 * @file	ExtDAC.hpp
 * @author	Kevin WYSOCKI
 * @date	25 avr. 2017
 * @brief	DAC088S085 Digital-To-Analog Converter class
 */

#ifndef INC_EXTDAC_HPP_
#define INC_EXTDAC_HPP_

#include "SPIMaster.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/


/**
 * @brief ExtDAC Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	HAL::SPIMaster::ID	BusID;
}EXTDAC_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	/**
	 * @brief DAC088S085 Digital-To-Analog Converter class
	 */
	class ExtDAC
	{
	public:

		/**
		 * @brief ExtDAC Identifier list
		 */
		enum ID
		{
			EXTDAC0 = 0,//!< EXTDAC0
			EXTDAC_MAX  //!< EXTDAC_MAX
		};

		/**
		 * @brief ExtDAC Channel list
		 */
		enum Channel
		{
			ExtDAC_Channel0 = 0,//!< ExtDAC_Channel0
			ExtDAC_Channel1,    //!< ExtDAC_Channel1
			ExtDAC_Channel2,    //!< ExtDAC_Channel2
			ExtDAC_Channel3,    //!< ExtDAC_Channel3
			ExtDAC_Channel4,    //!< ExtDAC_Channel4
			ExtDAC_Channel5,    //!< ExtDAC_Channel5
			ExtDAC_Channel6,    //!< ExtDAC_Channel6
			ExtDAC_Channel7,    //!< ExtDAC_Channel7
			ExtDAC_Channel_MAX  //!< ExtDAC_Channel_MAX
		};

		/**
		 * @brief Get instance method
		 * @param id : ExtDAC ID
		 * @return ExtDAC instance
		 */
		static ExtDAC* GetInstance (enum ID id);

		/**
		 * @brief Return instance ID
		 */
		enum ID GetID()
		{
			return this->id;
		}

		/**
		 * @brief Set output value of a specific channel
		 * @param ch : DAC channel
		 * @param output : Channel output value
		 * @return = 0 if no error, < 0 else
		 */
		int32_t SetOutputValue (enum Channel ch, uint8_t output);

	private:

		/**
		 * @private
		 * @brief ExtDAC private constructor
		 * @param id : ExtDAC ID
		 */
		ExtDAC(enum ID id);

		/**
		 * @private
		 * @brief Initialize DAC
		 */
		void Init ();

		/**
		 * @private
		 * @brief Instance ID
		 */
		enum ID id;

		/**
		 * @private
		 * @brief Peripheral definition
		 */
		EXTDAC_DEF def;

		/**
		 * @private
		 * @brief SPI bus used to communicate with DAC
		 */
		SPIMaster * bus;
	};
}

#endif /* INC_EXTDAC_HPP_ */
