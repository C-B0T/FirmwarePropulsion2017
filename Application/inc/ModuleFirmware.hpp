/**
 * @file	ModuleFirmware.hpp
 * @author	Kevin WYSOCKI
 * @date	16 mai 2017
 * @brief	Module that handle Firmware related actions
 */

#ifndef INC_MODULEFIRMWARE_HPP_
#define INC_MODULEFIRMWARE_HPP_

#include "ModuleManager.hpp"
#include "CommunicationHandler.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

class ModuleFirmware : public ModuleManager
{
public:

    static ModuleFirmware* GetInstance();

    /**
     * @private
     * @brief Firmware module task handler
     */
    void TaskHandler (void);

    Communication::Message GetReport()
    {
        return this->report;
    }

    int32_t HandleMessage (Communication::Message msg);

private:

    ModuleFirmware();

    int32_t Reset ();

    int32_t BootMode ();

    int32_t Ping ();

    int32_t ChangeAddr (uint8_t addr);

    Communication::CommunicationHandler* comHandler;

};

#endif /* INC_MODULEFIRMWARE_HPP_ */
