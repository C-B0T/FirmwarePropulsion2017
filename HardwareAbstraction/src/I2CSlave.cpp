/**
 * @file	I2C.hpp
 * @author	Kevin WYSOCKI
 * @date	6 mars 2017
 * @brief	I2C abstraction class
 */

#include <I2CSlave.hpp>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"

using namespace std;
using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// I2C0
#define I2C0_SCL_PORT			(GPIOA)
#define I2C0_SCL_PIN			(GPIO_Pin_8)
#define I2C0_SCL_PINSOURCE		(GPIO_PinSource8)
#define I2C0_SDA_PORT			(GPIOC)
#define I2C0_SDA_PIN			(GPIO_Pin_9)
#define I2C0_SDA_PINSOURCE		(GPIO_PinSource9)
#define I2C0_IO_AF				(GPIO_AF_I2C3)
#define I2C0_CLOCKFREQ			(100000u)
#define I2C0_SLAVEADDR			(0x10)
#define I2C0_BUS				(I2C3)
#define I2C0_INT_EVENT_CHANNEL	(I2C3_EV_IRQn)
#define I2C0_INT_ERROR_CHANNEL	(I2C3_ER_IRQn)
#define I2C0_INT_PRIORITY		(8u)

#define I2C_TIMEOUT				(0xFFFFu)

#define I2C_EVENT_START_OK							(I2C_EVENT_MASTER_MODE_SELECT)
#define I2C_EVENT_SLAVE_ACK_ADDR_READY_TO_TRANSMIT	(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
#define I2C_EVENT_SLAVE_ACK_ADDR_READY_TO_RECEIVE	(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief I2C instances
 */
static I2CSlave* _i2cSlave[I2CSlave::I2C_SLAVE_MAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static I2C_DEF _getI2CStruct (enum I2CSlave::ID id)
{
	I2C_DEF i2c;

	assert(id < I2CSlave::I2C_SLAVE_MAX);

	switch(id)
	{
	case I2CSlave::I2C_SLAVE0:
		// SCL Pin
		i2c.SCL.PORT			=	I2C0_SCL_PORT;
		i2c.SCL.PIN				=	I2C0_SCL_PIN;
		i2c.SCL.PINSOURCE		=	I2C0_SCL_PINSOURCE;
		i2c.SCL.AF				=	I2C0_IO_AF;
		// SDA Pin
		i2c.SDA.PORT			=	I2C0_SDA_PORT;
		i2c.SDA.PIN				=	I2C0_SDA_PIN;
		i2c.SDA.PINSOURCE		=	I2C0_SDA_PINSOURCE;
		i2c.SDA.AF				=	I2C0_IO_AF;
		// I2C Peripheral
		i2c.I2C.BUS				=	I2C0_BUS;
		i2c.I2C.SLAVE_ADDR		=	I2C0_SLAVEADDR;
		i2c.I2C.CLOCKFREQ		=	I2C0_CLOCKFREQ;
		// NVIC Peripheral
		i2c.INT.PRIORITY		=	I2C0_INT_PRIORITY;
		i2c.INT.EV_CHANNEL		=	I2C0_INT_EVENT_CHANNEL;
		i2c.INT.ER_CHANNEL		=	I2C0_INT_ERROR_CHANNEL;
		break;

	default:
		break;
	}

	return i2c;
}

static void _hardwareInit (enum I2CSlave::ID id)
{
	GPIO_InitTypeDef GPIOStruct;
	I2C_InitTypeDef I2CStruct;
	NVIC_InitTypeDef NVICStruct;

	I2C_DEF i2c;

	assert(id < I2CSlave::I2C_SLAVE_MAX);

	i2c = _getI2CStruct(id);

	// Init SCL and SDA pins
	GPIOStruct.GPIO_Mode	=	GPIO_Mode_AF;
	GPIOStruct.GPIO_OType	=	GPIO_OType_OD;
	GPIOStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIOStruct.GPIO_Speed	=	GPIO_Speed_100MHz;
	GPIOStruct.GPIO_Pin		=	i2c.SCL.PIN;

	GPIO_PinAFConfig(i2c.SCL.PORT, i2c.SCL.PINSOURCE, i2c.SCL.AF);
	GPIO_Init(i2c.SCL.PORT, &GPIOStruct);

	GPIOStruct.GPIO_Pin		=	i2c.SDA.PIN;

	GPIO_PinAFConfig(i2c.SDA.PORT, i2c.SDA.PINSOURCE, i2c.SDA.AF);
	GPIO_Init(i2c.SDA.PORT, &GPIOStruct);

	// I2C Init
	I2CStruct.I2C_Mode					=	I2C_Mode_I2C;
	I2CStruct.I2C_DutyCycle				=	I2C_DutyCycle_2;
	I2CStruct.I2C_Ack					=	I2C_Ack_Enable;
	I2CStruct.I2C_AcknowledgedAddress	=	I2C_AcknowledgedAddress_7bit;
	I2CStruct.I2C_OwnAddress1			=	i2c.I2C.SLAVE_ADDR;
	I2CStruct.I2C_ClockSpeed			=	i2c.I2C.CLOCKFREQ;

	I2C_Init(i2c.I2C.BUS, &I2CStruct);

	I2C_ITConfig(i2c.I2C.BUS, I2C_IT_EVT | /*I2C_IT_BUF |*/ I2C_IT_ERR, ENABLE);
	I2C_GeneralCallCmd(i2c.I2C.BUS, ENABLE);
	I2C_CalculatePEC(i2c.I2C.BUS, ENABLE);
	I2C_Cmd(i2c.I2C.BUS, ENABLE);

	// NVIC Init - Event interrupt
	NVICStruct.NVIC_IRQChannel						=	i2c.INT.EV_CHANNEL;
	NVICStruct.NVIC_IRQChannelPreemptionPriority 	= 	i2c.INT.PRIORITY;
	NVICStruct.NVIC_IRQChannelSubPriority 			= 	0;
	NVICStruct.NVIC_IRQChannelCmd					=	ENABLE;

	NVIC_Init(&NVICStruct);

	// NVIC Init - Error interrupt
	NVICStruct.NVIC_IRQChannel						=	i2c.INT.ER_CHANNEL;

	//NVIC_Init(&NVICStruct);
}

static ErrorStatus _waitEvent (I2C_TypeDef* i2c, uint32_t event)
{
	int32_t timeout = I2C_TIMEOUT;
	ErrorStatus status = ERROR;

	while(timeout > 0)
	{
		status = I2C_CheckEvent(i2c, event);

		if(status == SUCCESS)
		{
			break;
		}

		timeout--;
	}

	return status;
}

static int32_t _getErrorFromFlag (uint32_t flag)
{
	int32_t rval = 0;

	switch(flag)
	{
	case I2C_FLAG_BERR :
		rval = I2C_ERROR_MISPLACED_START_STOP;
		break;
	case I2C_FLAG_AF :
		rval = I2C_ERROR_ACKNOWLEDGE_FAILURE;
		break;
	case I2C_FLAG_OVR :
		rval = I2C_ERROR_OVER_UNDERRUN;
		break;
	case I2C_FLAG_PECERR :
		rval = I2C_ERROR_PACKET_ERROR;
		break;
	case I2C_FLAG_TIMEOUT :
		rval = I2C_ERROR_TIMEOUT;
		break;
	}

	return rval;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace HAL
{
	I2CSlave* I2CSlave::GetInstance(enum I2CSlave::ID id)
	{
		assert(id < I2CSlave::I2C_SLAVE_MAX);

		// if I2C instance already exists
		if(_i2cSlave[id] != NULL)
		{
			return _i2cSlave[id];
		}
		else
		{
			_i2cSlave[id] = new I2CSlave(id);

			return _i2cSlave[id];
		}
	}

	I2CSlave::I2CSlave(enum I2CSlave::ID id)
	{
		this->id = id;
		this->error = 0;
		this->def = _getI2CStruct(id);

		this->buffer.rdIndex = 0u;
		this->buffer.wrIndex = 0u;
		memset(this->buffer.frame, 0, sizeof(this->buffer.frame));

		_hardwareInit(id);
	}

	int32_t I2CSlave::Write(I2C_FRAME * frame)
	{
		uint32_t i = 0u;
		int32_t rval = 0;

		assert(frame != NULL);

		portENTER_CRITICAL();

		// Disable I2C interrupt to avoid ACK failure interrupt when master will NAK at the end of the transaction
		I2C_ITConfig(this->def.I2C.BUS, I2C_IT_EVT | /*I2C_IT_BUF |*/ I2C_IT_ERR, DISABLE);

		// Wait read event
		if(_waitEvent(this->def.I2C.BUS, I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED)!= SUCCESS)
		{
			rval = I2C_ERROR_TIMEOUT;
		}

		if(rval == NO_ERROR)
		{
			for(i=0u; i < frame->Length; )
			{
				// Send a byte
				I2C_SendData(this->def.I2C.BUS, frame->Data[i]);

				// Wait byte transmitted event
				if(_waitEvent(this->def.I2C.BUS, I2C_EVENT_SLAVE_BYTE_TRANSMITTED) != SUCCESS)
				{
					rval = I2C_ERROR_SLAVE_SEND_DATA_FAILED;
					break;
				}
				else
				{
					i++;	// Byte has been transmitted

					// If last byte, transmit CRC
					if(i == frame->Length)
					{
						I2C_TransmitPEC(this->def.I2C.BUS, ENABLE);

						// Wait NAK
						_waitEvent(this->def.I2C.BUS, I2C_EVENT_SLAVE_ACK_FAILURE);
					}
				}

				rval = i;
			}
		}

		// Clear AF bit and re-enable error interrupt
		I2C_ClearFlag(this->def.I2C.BUS, I2C_FLAG_AF);
		I2C_ITConfig(this->def.I2C.BUS, I2C_IT_EVT | /*I2C_IT_BUF |*/ I2C_IT_ERR, ENABLE);

		portEXIT_CRITICAL();

		return rval;
	}

	int32_t	I2CSlave::Read(I2C_FRAME * frame)
	{
		int32_t error = 0;

		assert(frame != NULL);

		if(this->buffer.wrIndex >= this->buffer.rdIndex)
		{
			memcpy(frame, &this->buffer.frame[this->buffer.rdIndex], sizeof(this->buffer.frame[this->buffer.rdIndex]));

			this->buffer.rdIndex++;

			// If all buffered frame read, clear indexes
			if(this->buffer.rdIndex >= this->buffer.wrIndex)
			{
				this->buffer.rdIndex = 0u;
				this->buffer.wrIndex = 0u;
			}

			this->error = NO_ERROR;
		}
		else
		{
			error = I2C_ERROR_NO_FRAME_BUFFERED;
		}

		return error;
	}

	void I2CSlave::INTERNAL_InterruptCallback(uint32_t flag)
	{
		uint8_t data = 0u;

		switch(flag)
		{
		// Address matched, store a new frame
		case I2C_FLAG_ADDR:
			if(this->buffer.wrIndex < (I2C_MAX_BUFFER_SIZE - 1u))
			{
				this->buffer.frame[this->buffer.wrIndex].Type 	=	(I2C_GetFlagStatus(this->def.I2C.BUS, I2C_FLAG_TRA) == SET ? I2C_FRAME_TYPE_READ : I2C_FRAME_TYPE_WRITE);
				this->buffer.frame[this->buffer.wrIndex].Length = 	0u;

				this->prevCRC = 0u;
				this->curCRC = I2C_GetPEC(this->def.I2C.BUS);

				if(this->buffer.frame[this->buffer.wrIndex].Type == I2C_FRAME_TYPE_READ)
				{
					//I2C_ITConfig(this->def.I2C.BUS, I2C_IT_BUF, DISABLE);

					this->DataRequest();
				}
				else
				{
					//I2C_ITConfig(this->def.I2C.BUS, I2C_IT_BUF, ENABLE);
				}

			}
			else
			{
				// drop frame
			}
			break;

		// Data received, store it
		case I2C_FLAG_RXNE:
			data = I2C_ReceiveData(this->def.I2C.BUS);

			if(this->buffer.wrIndex < (I2C_MAX_BUFFER_SIZE - 1u))
			{
				this->buffer.frame[this->buffer.wrIndex].Data[this->buffer.frame[this->buffer.wrIndex].Length] = data;

				this->buffer.frame[this->buffer.wrIndex].Length++;

				this->prevCRC = this->curCRC;
				this->curCRC = I2C_GetPEC(this->def.I2C.BUS);
			}
			else
			{
				// drop frame
			}
			break;

		// Frame ended, raise event
		case I2C_FLAG_STOPF:
			if(this->buffer.wrIndex < (I2C_MAX_BUFFER_SIZE - 1u))
			{
				this->buffer.frame[this->buffer.wrIndex].CRCval = this->prevCRC;
				this->buffer.wrIndex++;

				this->error = NO_ERROR;

				if(this->buffer.frame[this->buffer.wrIndex - 1u].Type == I2C_FRAME_TYPE_WRITE)
				{
					this->DataReceived();
				}
			}
			else
			{
				// drop frame
			}
			break;

		// Error management
		case I2C_FLAG_BERR:
		case I2C_FLAG_AF:
		case I2C_FLAG_OVR :
		case I2C_FLAG_PECERR :
		case I2C_FLAG_TIMEOUT:
			this->error = _getErrorFromFlag(flag);
			this->ErrorOccurred();
			break;
		}
	}
}

/*----------------------------------------------------------------------------*/
/* Interrupt Handler                                                          */
/*----------------------------------------------------------------------------*/

extern "C"
{
	void I2C3_EV_IRQHandler (void)
	{
		I2CSlave* instance = _i2cSlave[I2CSlave::I2C_SLAVE0];

		// Slave or General call address matched
		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_ADDR) == SET)
		{
			// SR2 must be read to clear ADDR flag in SR1
			I2C_ReadRegister(I2C3, I2C_Register_SR2);

			instance->INTERNAL_InterruptCallback(I2C_FLAG_ADDR);
		}

		// Byte transfer finished
		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_RXNE) == SET)
		{
			instance->INTERNAL_InterruptCallback(I2C_FLAG_RXNE);
		}

		// Stop bit received
		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_STOPF) == SET)
		{
			// Write to CR1 to clear STOPF in SR1
			I2C_GenerateSTOP(I2C3, DISABLE);

			instance->INTERNAL_InterruptCallback(I2C_FLAG_STOPF);
		}
	}

	void I2C3_ER_IRQHandler (void)
	{
		I2CSlave* instance = _i2cSlave[I2CSlave::I2C_SLAVE0];

		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_BERR) == SET)
		{
			I2C_ClearFlag(I2C3, I2C_FLAG_BERR);

			instance->INTERNAL_InterruptCallback(I2C_FLAG_BERR);
		}

		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_AF) == SET)
		{
			I2C_ClearFlag(I2C3, I2C_FLAG_AF);

			instance->INTERNAL_InterruptCallback(I2C_FLAG_AF);
		}

		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_OVR) == SET)
		{
			I2C_ClearFlag(I2C3, I2C_FLAG_OVR);

			instance->INTERNAL_InterruptCallback(I2C_FLAG_OVR);
		}

		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_PECERR) == SET)
		{
			I2C_ClearFlag(I2C3, I2C_FLAG_PECERR);

			instance->INTERNAL_InterruptCallback(I2C_FLAG_PECERR);
		}

		if(I2C_GetFlagStatus(I2C3, I2C_FLAG_TIMEOUT) == SET)
		{
			I2C_ClearFlag(I2C3, I2C_FLAG_TIMEOUT);

			instance->INTERNAL_InterruptCallback(I2C_FLAG_TIMEOUT);
		}
	}
}
