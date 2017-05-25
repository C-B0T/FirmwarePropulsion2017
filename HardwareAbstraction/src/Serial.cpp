/**
 * @file	Serial.cpp
 * @author	Kevin WYSOCKI
 * @date	9 nov. 2016
 * @brief	Serial Communication Abstraction Class
 *
 * HOWTO :
 * - Get Serial instance with Serial::GetInstance()
 * - Use Send() methods to send data
 * - Use Read() to peek one or more data
 * - OnDataReceivedCallback or OnEndOfTransmissionCallback can be used to
 * be notified of data received or end of transmission event
 */

#include <string.h>
#include "Serial.hpp"
#include "common.h"
#include "FreeRTOS.h"

using namespace std;
using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/**
 * @brief Serial FIFO size
 */
#define SERIAL_BUFFER_SIZE		(128u)

// UART1
#define SERIAL0_RX_PORT			(GPIOA)
#define SERIAL0_RX_PIN			(GPIO_Pin_10)
#define SERIAL0_RX_PINSOURCE	(GPIO_PinSource10)
#define SERIAL0_TX_PORT			(GPIOA)
#define SERIAL0_TX_PIN			(GPIO_Pin_9)
#define SERIAL0_TX_PINSOURCE	(GPIO_PinSource9)
#define SERIAL0_IO_AF			(GPIO_AF_USART1)
#define SERIAL0_BAUDRATE		(19200u)
#define SERIAL0_PORT			(USART1)
#define SERIAL0_INT_CHANNEL		(USART1_IRQn)
#define SERIAL0_INT_PRIORTY		(7u)

// UART3
#define SERIAL1_RX_PORT			(GPIOB)
#define SERIAL1_RX_PIN			(GPIO_Pin_11)
#define SERIAL1_RX_PINSOURCE	(GPIO_PinSource11)
#define SERIAL1_TX_PORT			(GPIOB)
#define SERIAL1_TX_PIN			(GPIO_Pin_10)
#define SERIAL1_TX_PINSOURCE	(GPIO_PinSource10)
#define SERIAL1_IO_AF			(GPIO_AF_USART3)
#define SERIAL1_BAUDRATE		(115200u)
#define SERIAL1_PORT			(USART3)
#define SERIAL1_INT_CHANNEL		(USART3_IRQn)
#define SERIAL1_INT_PRIORTY		(1u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief Serial instances
 */
static Serial* _serial[Serial::SERIAL_MAX] = {NULL};

/**
 * @brief Serial receive buffer
 */
static uint8_t _rxBuffer[Serial::SERIAL_MAX][SERIAL_BUFFER_SIZE];

/**
 * @brief Serial transmit buffer
 */
static uint8_t _txBuffer[Serial::SERIAL_MAX][SERIAL_BUFFER_SIZE];


/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve Serial definitions from Serial ID
 * @param id : Serial ID
 * @return SERIAL_DEF structure
 */
static SERIAL_DEF _getSerialStruct (enum Serial::ID id)
{
	SERIAL_DEF serial;

	assert(id < HAL::Serial::SERIAL_MAX);

	switch(id)
	{
	case Serial::SERIAL0:
		serial.RX.PORT			=	SERIAL0_RX_PORT;
		serial.RX.PIN			=	SERIAL0_RX_PIN;
		serial.RX.PINSOURCE		=	SERIAL0_RX_PINSOURCE;
		serial.RX.AF			=	SERIAL0_IO_AF;
		serial.TX.PORT			=	SERIAL0_TX_PORT;
		serial.TX.PIN			=	SERIAL0_TX_PIN;
		serial.TX.PINSOURCE		=	SERIAL0_TX_PINSOURCE;
		serial.TX.AF			=	SERIAL0_IO_AF;
		serial.USART.BAUDRATE	=	SERIAL0_BAUDRATE;
		serial.USART.PORT		=	SERIAL0_PORT;
		serial.INT.CHANNEL		=	SERIAL0_INT_CHANNEL;
		serial.INT.PRIORITY		=	SERIAL0_INT_PRIORTY;
		break;
	case Serial::SERIAL1:
		serial.RX.PORT			=	SERIAL1_RX_PORT;
		serial.RX.PIN			=	SERIAL1_RX_PIN;
		serial.RX.PINSOURCE		=	SERIAL1_RX_PINSOURCE;
		serial.RX.AF			=	SERIAL1_IO_AF;
		serial.TX.PORT			=	SERIAL1_TX_PORT;
		serial.TX.PIN			=	SERIAL1_TX_PIN;
		serial.TX.PINSOURCE		=	SERIAL1_TX_PINSOURCE;
		serial.TX.AF			=	SERIAL1_IO_AF;
		serial.USART.BAUDRATE	=	SERIAL1_BAUDRATE;
		serial.USART.PORT		=	SERIAL1_PORT;
		serial.INT.CHANNEL		=	SERIAL1_INT_CHANNEL;
		serial.INT.PRIORITY		=	SERIAL1_INT_PRIORTY;
		break;
	default:
		break;
	}

	return serial;
}

/**
 * @brief Initialize peripheral for a specific Serial
 * @param id : Serial ID
 */
static void _hardwareInit (enum Serial::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	USART_InitTypeDef UARTStruct;
	NVIC_InitTypeDef NVICStruct;

	SERIAL_DEF serial;

	assert(id < HAL::Serial::SERIAL_MAX);

	serial = _getSerialStruct(id);

	// Init RX and TX pins
	GPIOStruct.GPIO_Mode	=	GPIO_Mode_AF;
	GPIOStruct.GPIO_OType	=	GPIO_OType_PP;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_High_Speed;
	GPIOStruct.GPIO_Pin		=	serial.RX.PIN;

	GPIO_PinAFConfig(serial.RX.PORT, serial.RX.PINSOURCE, serial.RX.AF);
	GPIO_Init(serial.RX.PORT, &GPIOStruct);

	GPIOStruct.GPIO_Pin		=	serial.TX.PIN;

	GPIO_PinAFConfig(serial.TX.PORT, serial.TX.PINSOURCE, serial.TX.AF);
	GPIO_Init(serial.TX.PORT, &GPIOStruct);

    USART_Cmd(serial.USART.PORT, ENABLE);

	// UART Init
	UARTStruct.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
	UARTStruct.USART_Mode					=	USART_Mode_Rx | USART_Mode_Tx;
	UARTStruct.USART_Parity					=	USART_Parity_No;
	UARTStruct.USART_StopBits				=	USART_StopBits_1;
	UARTStruct.USART_WordLength				=	USART_WordLength_8b;
	UARTStruct.USART_BaudRate				=	serial.USART.BAUDRATE;

	USART_Init(serial.USART.PORT, &UARTStruct);

	USART_ITConfig(serial.USART.PORT, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(serial.USART.PORT, USART_IT_TXE, ENABLE);

	//NVIC Init
	NVICStruct.NVIC_IRQChannelCmd					=	ENABLE;
	NVICStruct.NVIC_IRQChannelSubPriority			=	0;
	NVICStruct.NVIC_IRQChannelPreemptionPriority	=	serial.INT.PRIORITY;
	NVICStruct.NVIC_IRQChannel						=	serial.INT.CHANNEL;

	NVIC_Init(&NVICStruct);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	Serial* Serial::GetInstance (enum Serial::ID id)
	{
		assert(id < Serial::SERIAL_MAX);

		// if Serial instance already exists
		if(_serial[id] != NULL)
		{
			return _serial[id];
		}
		else
		{
			// Create Serial instance
			_serial[id] = new Serial(id);

			return _serial[id];
		}
	}

	Serial::Serial (enum Serial::ID id)
	{
		this->id = id;
		this->def = _getSerialStruct(id);
		this->rxBuffer.rdIndex = 0;
		this->rxBuffer.wrIndex = 0;
		this->rxBuffer.data = _rxBuffer[id];
		this->txBuffer.rdIndex = 0;
		this->txBuffer.wrIndex = 0;
		this->txBuffer.data = _txBuffer[id];

		_hardwareInit(id);
	}

	bool Serial::Send (uint8_t byte)
	{
		bool sent = false;

		if((this->txBuffer.wrIndex + 1) < SERIAL_BUFFER_SIZE)
		{
			this->txBuffer.data[this->txBuffer.wrIndex++] = byte;

			USART_SendData(this->def.USART.PORT, this->txBuffer.data[this->txBuffer.rdIndex++]);

			sent = true;
		}

		return sent;
	}

	bool Serial::Send (string& str)
	{
		bool sent = false;
		size_t length = str.length();

		if(((this->txBuffer.wrIndex + length) < SERIAL_BUFFER_SIZE) &&
			(length > 0))
		{
			memcpy(this->txBuffer.data, str.c_str(), length);

			this->txBuffer.wrIndex += length;

			USART_ITConfig(this->def.USART.PORT, USART_IT_TXE, ENABLE);
			//USART_SendData(this->def.USART.PORT, this->txBuffer.data[this->txBuffer.rdIndex++]);

			sent = true;
		}

		return sent;
	}

	bool Serial::Send (const char * c_str)
	{
		bool sent = false;

		size_t length = strlen(c_str);

		if(((this->txBuffer.wrIndex + length) < SERIAL_BUFFER_SIZE) &&
		   (length > 0))
		{
			memcpy(this->txBuffer.data, c_str, length);

			this->txBuffer.wrIndex += length;

			USART_ITConfig(this->def.USART.PORT, USART_IT_TXE, ENABLE);
			//USART_SendData(this->def.USART.PORT, this->txBuffer.data[this->txBuffer.rdIndex++]);
		}

		return sent;
	}

	bool Serial::Send (const uint8_t * buffer, uint32_t length)
	{
		bool sent = false;

		if(((this->txBuffer.wrIndex + length) < SERIAL_BUFFER_SIZE) &&
		   (length > 0))
		{
			memcpy(this->txBuffer.data, buffer, length);

			this->txBuffer.wrIndex += length;

			USART_ITConfig(this->def.USART.PORT, USART_IT_TXE, ENABLE);
			//USART_SendData(this->def.USART.PORT, this->txBuffer.data[this->txBuffer.rdIndex++]);
		}

		return sent;
	}

	uint8_t Serial::Read ()
	{
		uint8_t byte = 0;

		if(this->rxBuffer.rdIndex < this->rxBuffer.wrIndex)
		{
			byte = this->rxBuffer.data[this->rxBuffer.rdIndex++];
		}
		else
		{
			byte = this->rxBuffer.data[this->rxBuffer.rdIndex];

			this->rxBuffer.wrIndex = 0;
			this->rxBuffer.rdIndex = 0;
		}

		return byte;
	}

	void Serial::Read (uint8_t * buffer, uint32_t& length)
	{
		if((this->rxBuffer.rdIndex + length) < this->rxBuffer.wrIndex)
		{
			memcpy(buffer, &this->rxBuffer.data[this->rxBuffer.rdIndex], length);

			this->rxBuffer.rdIndex += length;
		}
		else
		{
			length = this->rxBuffer.wrIndex - this->rxBuffer.rdIndex;

			memcpy(buffer, &this->rxBuffer.data[this->rxBuffer.rdIndex], length);
		}

		this->rxBuffer.rdIndex += length;

		if(this->rxBuffer.rdIndex == this->rxBuffer.wrIndex)
		{
			this->rxBuffer.rdIndex = 0;
			this->rxBuffer.wrIndex = 0;
		}
	}

	string Serial::ReadLine ()
	{
		string str = NULL;

		if(this->rxBuffer.rdIndex < this->rxBuffer.wrIndex)
		{
			uint32_t length = 0;
			char * pCh = strchr((const char*)&this->rxBuffer.data[this->rxBuffer.rdIndex], '\n');

			if(pCh != NULL)
			{
				length = (uint32_t)((char*)pCh - (char*)&this->rxBuffer.data[this->rxBuffer.rdIndex]) + 1u;

				if((this->rxBuffer.rdIndex + length)  < SERIAL_BUFFER_SIZE)
				{
					str = string((const char*)&this->rxBuffer.data[this->rxBuffer.rdIndex], length);

					this->rxBuffer.rdIndex += length;

					if(this->rxBuffer.rdIndex == this->rxBuffer.wrIndex)
					{
						this->rxBuffer.rdIndex = 0;
						this->rxBuffer.wrIndex = 0;
					}
				}
			}
		}

		return str;
	}

	void Serial::INTERNAL_InterruptCallback(uint16_t flag)
	{
		// Manage transmission
		if(flag == USART_FLAG_TXE)
		{
			if(this->txBuffer.rdIndex < this->txBuffer.wrIndex)
			{
				USART_SendData(this->def.USART.PORT, this->txBuffer.data[this->txBuffer.rdIndex++]);
			}
			else
			{
				this->txBuffer.rdIndex = 0;
				this->txBuffer.wrIndex = 0;
			}
		}
		// Manage end of transmission
		else if(flag == USART_FLAG_TC)
		{
			USART_ITConfig(this->def.USART.PORT, USART_IT_TXE, DISABLE);

			this->EndOfTransmission();
		}
		else if(flag == USART_FLAG_RXNE)
		{
			if(this->rxBuffer.wrIndex < (SERIAL_BUFFER_SIZE - 1))
			{
				this->rxBuffer.data[this->rxBuffer.wrIndex++] = (uint8_t)USART_ReceiveData(this->def.USART.PORT);

				this->DataReceived();
			}
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Interrupt Handler                                                          */
/*----------------------------------------------------------------------------*/

extern "C"
{
	/**
	 * @brief USART1 IRQ Handler
	 */
	void USART1_IRQHandler (void)
	{
		Serial* serial = Serial::GetInstance(Serial::SERIAL0);

		/*if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
		{
			serial->INTERNAL_InterruptCallback(USART_FLAG_TXE);
		}
		else if(USART_GetFlagStatus(USART1, USART_FLAG_TC) == SET)
		{
			USART_ClearFlag(USART1, USART_FLAG_TC);

			serial->INTERNAL_InterruptCallback(USART_FLAG_TC);
		}
		else */if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		{
			USART_ClearFlag(USART1, USART_FLAG_RXNE);

			serial->INTERNAL_InterruptCallback(USART_FLAG_RXNE);
		}
	}

	/**
	 * @brief USART3 IRQ Handler
	 */
	void USART3_IRQHandler (void)
	{
		Serial* serial = Serial::GetInstance(Serial::SERIAL1);

		if(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == SET)
		{
			serial->INTERNAL_InterruptCallback(USART_FLAG_TXE);
		}
		else if(USART_GetFlagStatus(USART3, USART_FLAG_TC) == SET)
		{
			serial->INTERNAL_InterruptCallback(USART_FLAG_TC);

			USART_ClearFlag(USART3, USART_FLAG_TC);
		}
		else if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
		{
			serial->INTERNAL_InterruptCallback(USART_FLAG_RXNE);

			USART_ClearFlag(USART3, USART_FLAG_RXNE);
		}
	}
}

/*----------------------------------------------------------------------------*/
/* ReRoute stdout                                                             */
/*----------------------------------------------------------------------------*/
extern "C"
{
	int _read (int file, char *ptr, int len)
	{
#if 0
		int DataIdx;

		if (len == 0)
			return 0;

		/* Force reading one by one */
		len = 1;

		for (DataIdx = 0; DataIdx < len; DataIdx++)
		{
			/* Loop until received data register is empty */
			while ((USART1->SR & USART_SR_RXNE) == 0)
			{}
			*ptr++ = USART_ReceiveData(USART1);
		}
#endif

		return len;
	}

	int _write(int file, char *ptr, int len)
	{
#if 0
		int DataIdx;

		for (DataIdx = 0; DataIdx < len; DataIdx++)
		{
			/* Loop until transmit data register is empty */
			while (!(USART1->SR & USART_SR_TXE))
			{}
			USART_SendData(USART1, (uint8_t) (*ptr++));
		}
#endif
		return len;
	}
}

