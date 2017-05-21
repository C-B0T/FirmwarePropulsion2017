/**
 * @file	ExtDAC.cpp
 * @author	Kevin WYSOCKI
 * @date	25 avr. 2017
 * @brief	DAC088S085 Digital-To-Analog Converter class
 */

#include "ExtDAC.hpp"
#include <stddef.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define EXT_DAC_CMD_SET_MODE_WRM			(0x8000u)
#define EXT_DAC_CMD_SET_MODE_WTM			(0x9000u)
#define EXT_DAC_CMD_SET_OUTPUT_HIZ			(0xD000u)
#define EXT_DAC_CMD_SET_OUTPUT_100K			(0xE000u)
#define EXT_DAC_CMD_SET_OUTPUT_2K5			(0xF000u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static HAL::ExtDAC* _instance[HAL::ExtDAC::EXTDAC_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static EXTDAC_DEF _getEXTDACStruct (enum HAL::ExtDAC::ID id)
{
	EXTDAC_DEF def;

	assert(id < HAL::ExtDAC::EXTDAC_MAX);

	switch(id)
	{
	case HAL::ExtDAC::EXTDAC0:
		def.BusID = HAL::SPIMaster::SPI_MASTER0;
		break;
	default:
		break;
	}

	return def;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	ExtDAC* ExtDAC::GetInstance(enum ExtDAC::ID id)
	{
		assert(id < ExtDAC::EXTDAC_MAX);

		if(_instance[id] == NULL)
		{
			_instance[id] = new ExtDAC(id);
		}
		else
		{

		}

		return _instance[id];
	}

	ExtDAC::ExtDAC(enum ExtDAC::ID id)
	{
		this->id = id;
		this->def = _getEXTDACStruct(id);

		this->bus = SPIMaster::GetInstance(this->def.BusID);

		this->Init();
	}

	void ExtDAC::Init ()
	{
		int32_t rval = 0u;
		uint16_t txWord = 0u;
		uint8_t txBuffer[2];
		uint8_t rxBuffer[2];
		uint32_t length = 0u;

		// 1. Prepare txBuffer - 2.5kOhms output Mode
		if(rval == NO_ERROR)
		{
			length = 0u;
			txWord = (EXT_DAC_CMD_SET_OUTPUT_2K5 | 0x000Fu);
			txBuffer[length++] = (uint8_t)((txWord >> 8u) & 0x00FFu);
			txBuffer[length++] = (uint8_t)((txWord) & 0x00FFu);
		}

		// 2. Send txBuffer - 2.5kOhms output Mode
		if(rval == NO_ERROR)
		{
			rval = this->bus->Transfer(txBuffer, rxBuffer, length);
		}

		// 1. Prepare txBuffer - WTM Mode
		if(rval == NO_ERROR)
		{
			length = 0u;
			txWord = (EXT_DAC_CMD_SET_MODE_WTM);
			txBuffer[length++] = (uint8_t)((txWord >> 8u) & 0x00FFu);
			txBuffer[length++] = (uint8_t)((txWord) & 0x00FFu);
		}

		// 2. Send txBuffer - WTM Mode
		if(rval == NO_ERROR)
		{
			rval = this->bus->Transfer(txBuffer, rxBuffer, length);
		}
	}

	int32_t ExtDAC::SetOutputValue (enum ExtDAC::Channel ch, uint8_t output)
	{
		int32_t rval = 0u;
		uint16_t txWord = 0u;
		uint8_t txBuffer[2];
		uint8_t rxBuffer[2];
		uint32_t length = 0u;

		// 1. Prepare txBuffer
		if(rval == NO_ERROR)
		{
			length = 0u;
			txWord = ( ((uint16_t)ch << 12u) | ((uint16_t)output << 4u) );
			txBuffer[length++] = (uint8_t)((txWord >> 8u) & 0x00FFu);
			txBuffer[length++] = (uint8_t)((txWord) & 0x00FFu);
		}

		// 2. Send txBuffer - WTM Mode
		if(rval == NO_ERROR)
		{
			rval = this->bus->Transfer(txBuffer, rxBuffer, length);
		}

		return rval;
	}
}
