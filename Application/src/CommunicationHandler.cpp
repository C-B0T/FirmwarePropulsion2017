/**
 * @file	Communication.cpp
 * @author	Kevin WYSOCKI
 * @date	20 avr. 2017
 * @brief	Communication Handler class
 */

#include <CommunicationHandler.hpp>
#include <string.h>

// FreeRTOS API
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define COM_SOF		'<'
#define COM_EOF		'>'

#define COM_EVENT_DATA_RECEIVED		(0x01u)
#define COM_EVENT_DATA_REQUESTED	(0x02u)
#define COM_EVENT_ERROR_OCCURED		(0x04u)
#define COM_EVENT_MASK				(COM_EVENT_DATA_RECEIVED 	| \
									 COM_EVENT_DATA_REQUESTED	| \
									 COM_EVENT_ERROR_OCCURED)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Communication::CommunicationHandler * _instance = NULL;

static EventGroupHandle_t _eventHandle;
static TaskHandle_t _taskHandle = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static void _onDataReceived (void * obj)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xEventGroupSetBitsFromISR(_eventHandle, COM_EVENT_DATA_RECEIVED, &xHigherPriorityTaskWoken);
}

static void _onDataRequested (void * obj)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xEventGroupSetBitsFromISR(_eventHandle, COM_EVENT_DATA_REQUESTED, &xHigherPriorityTaskWoken);
}

static void _onError (void * obj)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xEventGroupSetBitsFromISR(_eventHandle, COM_EVENT_ERROR_OCCURED, &xHigherPriorityTaskWoken);
}

static void _taskHandler (void * obj)
{
	Communication::CommunicationHandler* comHandler = static_cast<Communication::CommunicationHandler*>(obj);

	comHandler->TaskHandler();
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{
	CommunicationHandler * CommunicationHandler::GetInstance()
	{
		if(_instance == NULL)
		{
			_instance = new CommunicationHandler();

			return _instance;
		}
		else
		{
			return _instance;
		}
	}

	CommunicationHandler::CommunicationHandler()
	{
		this->error = NO_ERROR;

		// Create I2C bus
//		this->bus = HAL::I2CSlave::GetInstance(HAL::I2CSlave::I2C_SLAVE0);
		this->bus = HAL::Serial::GetInstance(HAL::Serial::SERIAL0);

		this->bus->DataReceived += _onDataReceived;
//		this->bus->DataRequest += _onDataRequested;
//		this->bus->ErrorOccurred += _onError;

		// Create event
		_eventHandle = xEventGroupCreate();

		// Create Communication task
		xTaskCreate((TaskFunction_t)_taskHandler,
					"Communication Task",
					512,
					(void*)this,
					configMAX_PRIORITIES - 1u,		// Communication must be higher priority task to avoid bus overrun
					&_taskHandle);
	}

	int32_t CommunicationHandler::Write (Message * msg)
	{
		int32_t rval = NO_ERROR;

		assert(msg != NULL);

		if(rval == NO_ERROR)
		{
//			rval = msg->Encode(&this->frame);
		}

		if(rval == NO_ERROR)
		{
//			rval = this->bus->Write(&this->frame);
		}

		return rval;
	}

	void CommunicationHandler::GetLastMessage (Message * msg)
	{
		memcpy(msg, &this->msg, sizeof(Message));
	}

	void CommunicationHandler::TaskHandler(void)
	{
		EventBits_t events = 0u;
		uint8_t data = 0u;

		while(1)
		{
			// 1. Wait event
			events = xEventGroupWaitBits(_eventHandle,
										 COM_EVENT_MASK,
										 pdTRUE,
										 pdFALSE,
										 portMAX_DELAY);

			if((events & COM_EVENT_DATA_RECEIVED) == COM_EVENT_DATA_RECEIVED)
			{
				if(this->error == NO_ERROR)
				{
					data = this->bus->Read();

					if(data == COM_SOF)
					{
						this->buffer.wrIndex = 0u;
					}
					else if(data == COM_EOF)
					{
						this->msg.Decode(this->buffer.data, this->buffer.wrIndex);
						this->MessageReceived();
					}
					else
					{
						this->buffer.data[this->buffer.wrIndex++] = data;
					}
				}
			}
		}
	}
}
