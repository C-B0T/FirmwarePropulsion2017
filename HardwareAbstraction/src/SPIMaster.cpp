/**
 * @file	SPIMaster.cpp
 * @author	Kevin WYSOCKI
 * @date	24 avr. 2017
 * @brief	SPI master abstraction class
 */


#include <stddef.h>
#include <SPIMaster.hpp>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define SPI0_CS_PORT			(GPIOC)
#define SPI0_CS_PIN				(GPIO_Pin_15)
#define SPI0_CS_PINSOURCE		(GPIO_PinSource15)
#define SPI0_SCK_PORT			(GPIOB)
#define SPI0_SCK_PIN			(GPIO_Pin_3)
#define SPI0_SCK_PINSOURCE		(GPIO_PinSource3)
#define SPI0_MISO_PORT			(GPIOB)
#define SPI0_MISO_PIN			(GPIO_Pin_4)
#define SPI0_MISO_PINSOURCE		(GPIO_PinSource4)
#define SPI0_MOSI_PORT			(GPIOB)
#define SPI0_MOSI_PIN			(GPIO_Pin_5)
#define SPI0_MOSI_PINSOURCE		(GPIO_PinSource5)
#define SPI0_IO_AF				(GPIO_AF_SPI1)
#define SPI0_BUS				(SPI1)
#define SPI0_CLOCK_PRESCALER	(SPI_BaudRatePrescaler_8)
#define SPI0_CLOCK_POLARITY		(SPI_CPOL_High)
#define SPI0_CLOCK_PHASE		(SPI_CPHA_2Edge)

#define SPI_TIMEOUT				(0x10000u)


/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief SPIMaster instances
 */
static HAL::SPIMaster* _instance[HAL::SPIMaster::SPI_MASTER_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static SPIMaster_DEF _getSPIStruct (enum HAL::SPIMaster::ID id)
{
	SPIMaster_DEF spi;

	assert(id < HAL::SPIMaster::SPI_MASTER_MAX);

	switch(id)
	{
	case HAL::SPIMaster::SPI_MASTER0:
		// CS Pin
		spi.CS.PORT				=	SPI0_CS_PORT;
		spi.CS.PIN				=	SPI0_CS_PIN;
		spi.CS.PINSOURCE		=	SPI0_CS_PINSOURCE;
		spi.CS.AF				=	SPI0_IO_AF;
		// SCK Pin
		spi.SCK.PORT			=	SPI0_SCK_PORT;
		spi.SCK.PIN				=	SPI0_SCK_PIN;
		spi.SCK.PINSOURCE		=	SPI0_SCK_PINSOURCE;
		spi.SCK.AF				=	SPI0_IO_AF;
		// MISO Pin
		spi.MISO.PORT			=	SPI0_MISO_PORT;
		spi.MISO.PIN			=	SPI0_MISO_PIN;
		spi.MISO.PINSOURCE		=	SPI0_MISO_PINSOURCE;
		spi.MISO.AF				=	SPI0_IO_AF;
		// MOSI Pin
		spi.MOSI.PORT			=	SPI0_MOSI_PORT;
		spi.MOSI.PIN			=	SPI0_MOSI_PIN;
		spi.MOSI.PINSOURCE		=	SPI0_MOSI_PINSOURCE;
		spi.MOSI.AF				=	SPI0_IO_AF;

		// SPIMaster Peripheral
		spi.SPI.BUS				=	SPI0_BUS;
		spi.SPI.CLOCKPRESCALER	=	SPI0_CLOCK_PRESCALER;
		spi.SPI.CLOCKPOLARITY	=	SPI0_CLOCK_POLARITY;
		spi.SPI.CLOCKPHASE		=	SPI0_CLOCK_PHASE;
		break;

	default:
		break;
	}

	return spi;
}

static void _hardwareInit (enum HAL::SPIMaster::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	SPI_InitTypeDef SPIStruct;

	SPIMaster_DEF spi;

	assert(id < HAL::SPIMaster::SPI_MASTER_MAX);

	spi = _getSPIStruct(id);

	// Init CS, SCK, MISO and MOSI pins
	GPIOStruct.GPIO_Mode	=	GPIO_Mode_OUT;
	GPIOStruct.GPIO_OType	=	GPIO_OType_PP;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_Speed_100MHz;
	GPIOStruct.GPIO_Pin		=	spi.CS.PIN;

	GPIO_Init(spi.CS.PORT, &GPIOStruct);

	GPIOStruct.GPIO_Mode	=	GPIO_Mode_AF;
	GPIOStruct.GPIO_Pin		=	spi.SCK.PIN;

	GPIO_PinAFConfig(spi.SCK.PORT, spi.SCK.PINSOURCE, spi.SCK.AF);
	GPIO_Init(spi.SCK.PORT, &GPIOStruct);

	GPIOStruct.GPIO_Pin		=	spi.MOSI.PIN;

	GPIO_PinAFConfig(spi.MOSI.PORT, spi.MOSI.PINSOURCE, spi.MOSI.AF);
	GPIO_Init(spi.MOSI.PORT, &GPIOStruct);

	GPIOStruct.GPIO_Pin		=	spi.MISO.PIN;

	GPIO_PinAFConfig(spi.MISO.PORT, spi.MISO.PINSOURCE, spi.MISO.AF);
	GPIO_Init(spi.MISO.PORT, &GPIOStruct);

	// SPI Init
	SPIStruct.SPI_Direction			=	SPI_Direction_2Lines_FullDuplex;
	SPIStruct.SPI_Mode				=	SPI_Mode_Master;
	SPIStruct.SPI_DataSize			=	SPI_DataSize_8b;
	SPIStruct.SPI_CPOL				=	spi.SPI.CLOCKPOLARITY;
	SPIStruct.SPI_CPHA				=	spi.SPI.CLOCKPHASE;
	SPIStruct.SPI_NSS				=	SPI_NSS_Soft;
	SPIStruct.SPI_BaudRatePrescaler	=	spi.SPI.CLOCKPRESCALER;
	SPIStruct.SPI_FirstBit			=	SPI_FirstBit_MSB;
	SPIStruct.SPI_CRCPolynomial		=	0x0000u;

	SPI_Init(spi.SPI.BUS, &SPIStruct);
	SPI_Cmd(spi.SPI.BUS, ENABLE);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	SPIMaster* SPIMaster::GetInstance(enum SPIMaster::ID id)
	{
		assert(id < SPIMaster::SPI_MASTER_MAX);

		if(_instance[id] == NULL)
		{
			_instance[id] = new SPIMaster(id);

			return _instance[id];
		}
		else
		{
			return _instance[id];
		}
	}

	SPIMaster::SPIMaster(enum SPIMaster::ID id)
	{
		this->id = id;
		this->def = _getSPIStruct(id);

		_hardwareInit(id);
	}

	int32_t SPIMaster::Transfer(uint8_t * txBuffer, uint8_t * rxBuffer, uint32_t length)
	{
		int32_t rval = NO_ERROR;
		uint32_t i = 0u;

		// 1. Drive nCS low
		if(rval == NO_ERROR)
		{
			GPIO_ResetBits(this->def.CS.PORT, this->def.CS.PIN);
		}

		// 2. Transfer data
		for(i=0u; i<length; i++)
		{
			// 2.1 Send data
			SPI_SendData(this->def.SPI.BUS, (uint16_t)txBuffer[i]);

			// 2.2 Wait while RX buffer is empty
			while(SPI_I2S_GetFlagStatus(this->def.SPI.BUS, SPI_I2S_FLAG_RXNE) == RESET);

			// 2.3 Read data
			rxBuffer[i] = (uint8_t)SPI_ReceiveData(this->def.SPI.BUS);
		}

		// 3. Drive nCS high
		if(rval == NO_ERROR)
		{
			GPIO_SetBits(this->def.CS.PORT, this->def.CS.PIN);
		}

		return rval;
	}
}
