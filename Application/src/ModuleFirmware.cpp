/**
 * @file	ModuleFirmware.cpp
 * @author	Kevin WYSOCKI
 * @date	16 mai 2017
 * @brief	Module that handle Firmware related actions
 */

#include "ModuleFirmware.hpp"
#include <stddef.h>

// FreeRTOS API
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

using namespace Communication;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MOD_FW_EVENT_HANDLE_MSG     (0x01u)
#define MOD_FW_EVENT_MASK           (MOD_FW_EVENT_HANDLE_MSG)

#define FW_PING_KEY                 (0x5AA555AAu)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static ModuleFirmware* _instance = NULL;

static EventGroupHandle_t _eventHandle = NULL;
static TaskHandle_t _taskHandle = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static void _taskHandler (void* obj)
{
    ModuleFirmware * instance = static_cast<ModuleFirmware*>(obj);

    instance->TaskHandler();
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

ModuleFirmware* ModuleFirmware::GetInstance()
{
    if(_instance == NULL)
    {
        _instance = new ModuleFirmware();
    }
    else
    {

    }

    return _instance;
}

ModuleFirmware::ModuleFirmware()
{
    this->comHandler = CommunicationHandler::GetInstance();

    // Create event
    _eventHandle = xEventGroupCreate();

    // Create task
    xTaskCreate((TaskFunction_t)_taskHandler,
                "Firmware Module Task",
                128,
                (void*)this,
                0,
                &_taskHandle);
}

void ModuleFirmware::TaskHandler (void)
{
    EventBits_t events = 0u;
    int32_t error = NO_ERROR;

    while(1)
    {
        // 1. Wait event
        events = xEventGroupWaitBits(_eventHandle,
                                     MOD_FW_EVENT_MASK,
                                     pdTRUE,
                                     pdFALSE,
                                     portMAX_DELAY);

        if((events & MOD_FW_EVENT_HANDLE_MSG) == MOD_FW_EVENT_HANDLE_MSG)
        {
            // Handle message
            if(error == NO_ERROR)
            {
                switch(this->msg.Type)
                {
                case MSG_TYPE_FW_RESET:
                    error = this->Reset();
                    break;
                case MSG_TYPE_FW_BOOT_MODE:
                    error = this->BootMode();
                    break;
                case MSG_TYPE_FW_PING:
                    error = this->Ping();
                    break;
                case MSG_TYPE_FW_CHANGE_ADDR:
                    error = this->ChangeAddr(this->msg.Param.ChangeAddress.addr);
                    break;
                default:
                    break;
                }
            }

            // action is done
            if(error == NO_ERROR)
            {
                this->action.status = ModuleActionStatus_Done;
            }

            // Notify that action ended
            if(error == NO_ERROR)
            {
                this->ActionEnded();
            }
        }

        error = NO_ERROR;
    }
}

int32_t ModuleFirmware::HandleMessage(Message msg)
{
    int32_t error = NO_ERROR;

    // Check module is idle
    if(this->action.status != ModuleActionStatus_Idle)
    {
        error = MOD_ERROR_NOT_IDLE;
    }
    else
    {
        this->action.type = msg.Type;
        this->action.status = ModuleActionStatus_Running;
        this->msg = msg;

        xEventGroupSetBits(_eventHandle, MOD_FW_EVENT_HANDLE_MSG);
    }

    return error;
}

int32_t ModuleFirmware::Reset ()
{
    int32_t error = NO_ERROR;

    NVIC_SystemReset();

    return error;
}

int32_t ModuleFirmware::BootMode ()
{
    int32_t error = NO_ERROR;

    /** @todo Jump to bootloader address */

    return error;
}

int32_t ModuleFirmware::Ping ()
{
    int32_t error = NO_ERROR;

    // 1. Set report
    this->report = Message(MSG_TYPE_FW_PING);
    this->report.Param.Ping.key = FW_PING_KEY;

    // 2. Send report
    error = this->comHandler->Write(&this->report);

    return error;
}

int32_t ModuleFirmware::ChangeAddr (uint8_t addr)
{
    int32_t error = NO_ERROR;

    return error;
}
