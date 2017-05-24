/**
 * @file    ModulesDef.h
 * @author  Kevin WYSOCKI
 * @date    30 avr. 2017
 * @brief   Controller class
 */

#ifndef INC_MODULES_H_
#define INC_MODULES_H_

#include "common.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

enum ModuleID
{
	ModuleID_Firmware		=	0x00u,
	ModuleID_Prop			=	0x10u,
	ModuleID_Servo			=	0x20u,
	ModuleID_MPP			=	0x30u,
	ModuleID_GPIO           =   0x40u,
	ModuleID_BAR            =   0x50u,
	ModuleID_Mask	 		= 	0xF0u,
};

/**
 * @}//addtogroup
 */

#endif /* MODULES_H_ */
