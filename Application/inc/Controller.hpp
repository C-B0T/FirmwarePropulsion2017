/**
 * @file	Controller.hpp
 * @author	Kevin WYSOCKI
 * @date	30 avr. 2017
 * @brief	Controller class
 */

#ifndef INC_CONTROLLER_HPP_
#define INC_CONTROLLER_HPP_

#include "common.h"
#include "CommunicationHandler.hpp"
#include "Message.hpp"
#include "ModuleManager.hpp"
#include "ModulesDef.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define CONTROLLER_ERROR_WRONG_PARAM	(-1)

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @brief Controller class
 */
class Controller
{
public:
	/**
	 * @brief Return Controller instance
	 */
	static Controller * GetInstance();

	/**
	 * @private
	 * @brief Controller task handler
	 */
	void TaskHandler (void);

private:

	/**
	 * @private
	 * @brief Private constructor
	 */
	Controller ();

	/**
	 * @private
	 * @brief Create modules handled by the firmware
	 */
	void CreateModuleList ();

	ModuleManager* FindModule (enum ModuleID moduleID);

	/**
	 * @private
	 * @brief Communication handler
	 */
	Communication::CommunicationHandler* comHandler;

	/**
	 * @private
	 * @brief Current message
	 */
	Communication::Message msg;

	std::vector<ModuleManager*> moduleList;
};

#endif /* INC_CONTROLLER_HPP_ */
