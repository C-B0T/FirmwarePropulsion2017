/**
 * @file	common.h
 * @author	Kevin WYSOCKI
 * @date	8 nov. 2016
 * @brief	Common definitions, types, etc.
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

/*----------------------------------------------------------------------------*/
/* Doc		                                                                  */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 * @brief Motion control objects
 *
 * This namespace contains objects used for motion control (speed regulation, odometry, trajectories planning, etc..)
 */

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#ifdef assert
	#undef assert
#endif

#define assert(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
#define ASSERT_FAILED_MESSSAGE	("ERROR | Assertion failed in %s @ line %d\n")

#define vTaskDelayMs(t)	vTaskDelay((t) / portTICK_PERIOD_MS)

#define NO_ERROR	(0)

/*----------------------------------------------------------------------------*/
/* Types										                              */
/*----------------------------------------------------------------------------*/

typedef float float32_t;
typedef double float64_t;

/*----------------------------------------------------------------------------*/
/* Enums										                              */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class										                              */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Functions export                                                           */
/*----------------------------------------------------------------------------*/

extern "C" void assert_failed(uint8_t* file, uint32_t line);


#endif /* COMMON_H_ */
