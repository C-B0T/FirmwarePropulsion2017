/**
 * @file	I2CCommon.h
 * @author	Kevin Wysocki
 * @date	20 avr. 2017
 * @brief	Some I2C Master/Slave common definitions, types, etc.
 */

#ifndef INC_I2CCOMMON_H_
#define INC_I2CCOMMON_H_

#include "stm32f4xx.h"
#include "common.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define I2C_MAX_BUFFER_SIZE		(8u)
#define I2C_MAX_FRAME_SIZE		(32u)

#define I2C_ERROR_NO_FRAME_BUFFERED			(-1)	/**< No incoming frame buffered */
#define I2C_ERROR_MISPLACED_START_STOP		(-2)
#define I2C_ERROR_ACKNOWLEDGE_FAILURE		(-3)
#define I2C_ERROR_OVER_UNDERRUN				(-4)
#define I2C_ERROR_PACKET_ERROR				(-5)
#define I2C_ERROR_TIMEOUT					(-6)
#define I2C_ERROR_SLAVE_SEND_DATA_FAILED	(-7)

/**
 * @brief I2C Definition structure
 * Used to define peripheral definition in order to initialize them
 */
typedef struct
{
	// IO definitions
	struct Pin
	{
		GPIO_TypeDef *	PORT;
		uint16_t		PIN;
		uint8_t			PINSOURCE;
		uint8_t			AF;
	}SDA;

	struct Pin SCL;

	// I2C definitions
	struct I2c
	{
		I2C_TypeDef *	BUS;
		uint32_t		CLOCKFREQ;
		uint8_t			SLAVE_ADDR;
	}I2C;

	// Interrupt vector controller definitions
	struct Int
	{
		uint8_t	PRIORITY;		/**< Interrupt priority, 0 to 15, 0 is the highest priority */
		uint8_t	EV_CHANNEL;		/**< Interrupt IRQ Channel */
		uint8_t	ER_CHANNEL;		/**< Interrupt IRQ Channel */
	}INT;
}I2C_DEF;

/**
 * brief I2C Frame Type
 */
typedef enum
{
	I2C_FRAME_TYPE_WRITE = 0,	/**< I2C Write Frame */
	I2C_FRAME_TYPE_READ = 1, 	/**< I2C Read Frame */
	I2C_FRAME_TYPE_MASK	= 1  	/**< I2C Frame Type Mask */
}I2C_FRAME_TYPE;

/**
 * brief I2C Frame structure
 */
typedef struct
{
	I2C_FRAME_TYPE	Type;						/**< Frame Type */
	uint32_t		Length;						/**< Frame Length (excluding CRC) */
	uint8_t			Data[I2C_MAX_FRAME_SIZE];	/**< Frame Data (excluding CRC) */
	uint8_t			CRCval;						/**< CRC-8 value */
}I2C_FRAME;

/**
 * brief Frame Buffer
 */
typedef struct
{
	uint32_t 	rdIndex;						/**< Frame write index */
	uint32_t 	wrIndex;						/**< Frame read index */
	I2C_FRAME 	frame[I2C_MAX_BUFFER_SIZE];		/**< Frame buffer */
}I2C_FRAMEBUFFERR;

#endif /* INC_I2CCOMMON_H_ */
