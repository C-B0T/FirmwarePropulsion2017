/**
 * @file    ModuleManager.hpp
 * @author  Kevin Wysocki
 * @date    20 avr. 2017
 * @brief   Message Class
 *
 *  HOWTO :
 *  - Create a new ModuleManager with new_ModuleManager
 *  - You need to override HandleRequest() and Observer.onEvent()
 */

#ifndef MODULEMANAGER_HPP_
#define MODULEMANAGER_HPP_

#include "Message.hpp"
#include "Event.hpp"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MOD_ERROR_NOT_IDLE      (-1)

/**
 * @brief Module Action Status
 */
typedef enum
{
	ModuleActionStatus_Idle = 0u,		/**< No action */
	ModuleActionStatus_Done,			/**< Action is done */
	ModuleActionStatus_Running,			/**< Action is running */
}ModuleActionStatus;

/**
 * @brief Module Action
 */
typedef struct
{
	MESSAGE_TYPE	        type;			/**< Action type */
	ModuleActionStatus  	status;			/**< Action status */
}ModuleAction;

/* Exported constants --------------------------------------------------------*/

/* Exported class ------------------------------------------------------------*/

class ModuleManager
{
public:
    ModuleManager();

    uint32_t GetID ()
    {
        return this->id;
    }

    ModuleAction GetAction ()
    {
        return this->action;
    }

    virtual Communication::Message GetReport ();

    virtual int32_t HandleMessage (Communication::Message msg);

    /**
     * @pure
     * @brief Virtual pure destructor
     */
    virtual ~ModuleManager() = 0;


    Utils::Event ActionEnded;

protected:
    uint32_t id;

    ModuleAction action;
    Communication::Message msg;
    Communication::Message report;
};

#endif /* MODULEMANAGER_HPP_ */
