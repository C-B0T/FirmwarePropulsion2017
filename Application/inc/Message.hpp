/**
 * @file	Message.hpp
 * @author	Kevin Wysocki
 * @date	20 avr. 2017
 * @brief	Message Class
 *
 *	Used to encode and decode messages
 *	written or read on the I2C bus
 */

#ifndef INC_MESSAGE_HPP_
#define INC_MESSAGE_HPP_

#include "common.h"
#include "I2CCommon.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MSG_ERROR_UNKNOWN_TYPE		(-1)
#define MSG_ERROR_WRONG_NB_DATA		(-2)
#define MSG_ERROR_NO_ANSWER_NEEDED	(-3)
#define MSG_ERROR_WRONG_CRC			(-4)

#define MSG_PARAM_GPIO_MAX_ANALOG_INPUTS	(10u)

/**
 * @brief Message type list
 */
typedef enum
{
	MSG_TYPE_UNKNOWN				=	-1,

	// Firmware messages
	MSG_TYPE_FW_RESET 				= 	0x00,
	MSG_TYPE_FW_BOOT_MODE			=	0x01,
	MSG_TYPE_FW_PING				=	0x02,
	MSG_TYPE_FW_CHANGE_ADDR			=	0x03,
	//MSG_TYPE_CHECKUP				=	0x10,

	// Propulsion messages
	MSG_TYPE_PROP_INIT				=	0x10,
	MSG_TYPE_PROP_GET_POSITION		=	0x11,
	MSG_TYPE_PROP_GOTOXY			=	0x12,
	MSG_TYPE_PROP_GOLINEAR			=	0x13,
	MSG_TYPE_PROP_ROTATE			=	0x14,
	MSG_TYPE_PROP_SET_ANGLE			=	0x15,
	MSG_TYPE_PROP_SET_POSITION		=	0x16,
	MSG_TYPE_PROP_STOP				=	0x17,
	MSG_TYPE_PROP_SET_STATE			=	0x18,
	MSG_TYPE_PROP_GET_STATUS		=	0x19,

	// Servo messages
	MSG_TYPE_SERVO_INIT				=	0x20,
	MSG_TYPE_SERVO_SET_ANGLE		=	0x21,
	MSG_TYPE_SERVO_SET_SPEED		=	0x22,
	MSG_TYPE_SERVO_GET_STATUS		=	0x23,

	// Stepper motor messages
	MSG_TYPE_MPP_INIT				=	0x30,
	MSG_TYPE_MPP_SET_SPEED			=	0x31,
	MSG_TYPE_MPP_SET_STATE			=	0x32,
	MSG_TYPE_MPP_MOVE				=	0x33,
	MSG_TYPE_MPP_RUN				=	0x34,
	MSG_TYPE_MPP_GET_STATUS			=	0x35,

	// GPIO messages
	MSG_TYPE_GPIO_INIT				=	0x40,
	MSG_TYPE_GPIO_READ_ALL_DIG_IN	=	0x41,
	MSG_TYPE_GPIO_READ_ALL_AN_IN	=	0x42,
	MSG_TYPE_GPIO_GET_OUTPUT		=	0x43,
	MSG_TYPE_GPIO_SET_OUTPUT		=	0x44,
	MSG_TYPE_GPIO_GET_STATUS		=	0x45,


	// Barillet messages
	MSG_TYPE_BAR_INIT				=	0x50,
	MSG_TYPE_BAR_MOVE_INDEX			=	0x51,
	MSG_TYPE_BAR_GET_STATUS			=	0x52,

	// Mask
	MSG_TYPE_MODULE_MASK            =   0xF0,
}MESSAGE_TYPE;

/**
 * @brief Reset command param
 */
typedef struct
{
	uint32_t key;
}MSG_PARAM_RESET;

/**
 * @brief Boot mode command param
 */
typedef struct
{
	uint32_t key;
}MSG_PARAM_BOOT_MODE;

/**
 * @brief Ping command param
 */
typedef struct
{
	uint32_t key;
}MSG_PARAM_PING;

/**
 * @brief Change address command param
 */
typedef struct
{
	uint8_t addr;
}MSG_PARAM_CHANGE_ADDR;

/**
 * @brief Init commande param
 */
typedef struct
{
	uint8_t id;
}MSG_PARAM_INIT;

/**
 * @brief Get status commande param
 */
typedef struct
{
	uint16_t status;
}MSG_PARAM_GET_STATUS;


/**
 * @brief Get position command param
 */
typedef struct
{
	int16_t posX;
	int16_t posY;
	int16_t angle;
}MSG_PARAM_PROP_GET_POSITION;

/**
 * @brief GotoXY command param
 */
typedef struct
{
	int16_t posX;
	int16_t posY;
}MSG_PARAM_PROP_GOTOXY;

/**
 * @brief GoLinear command param
 */
typedef struct
{
	int16_t distance;
}MSG_PARAM_PROP_GOLINEAR;

/**
 * @brief Rotate command param
 */
typedef struct
{
	int16_t angle;
}MSG_PARAM_PROP_ROTATE;

/**
 * @brief Set angle command param
 */
typedef struct
{
	int16_t angle;
}MSG_PARAM_PROP_SET_ANGLE;

/**
 * @brief Set position command param
 */
typedef struct
{
	int16_t posX;
	int16_t posY;
}MSG_PARAM_PROP_SET_POSITION;

/**
 * @brief Stopcommand param
 */
typedef struct
{
	uint8_t mode;
}MSG_PARAM_PROP_STOP;


/**
 * @brief Set state command param
 */
typedef struct
{
	uint8_t state;
}MSG_PARAM_PROP_SET_STATE;

/**
 * @brief Prop command param union
 */
typedef union
{
	MSG_PARAM_INIT					Init;
	MSG_PARAM_GET_STATUS			GetStatus;
	MSG_PARAM_PROP_GET_POSITION 	GetPosition;
	MSG_PARAM_PROP_GOTOXY			GotoXY;
	MSG_PARAM_PROP_GOLINEAR			GoLinear;
	MSG_PARAM_PROP_ROTATE			Rotate;
	MSG_PARAM_PROP_SET_ANGLE		SetAngle;
	MSG_PARAM_PROP_SET_POSITION		SetPosition;
	MSG_PARAM_PROP_STOP				Stop;
	MSG_PARAM_PROP_SET_STATE		SetState;
}MSG_PARAM_PROP;

/**
 * @brief Servo set angle command param
 */
typedef struct
{
	uint8_t ID;
	int16_t angle;
}MSG_PARAM_SERVO_SET_ANGLE;

/**
 * @brief Servo set speed command param
 */
typedef struct
{
	uint8_t ID;
	int16_t speed;
}MSG_PARAM_SERVO_SET_SPEED;

/**
 * @brief Servo command param union
 */
typedef union
{
	MSG_PARAM_INIT				Init;
	MSG_PARAM_GET_STATUS		GetStatus;
	MSG_PARAM_SERVO_SET_ANGLE	SetAngle;
	MSG_PARAM_SERVO_SET_SPEED	SetSpeed;
}MSG_PARAM_SERVO;

/**
 * @brief MPP set speed command param
 */
typedef struct
{
	uint8_t ID;
	int16_t speed;
}MSG_PARAM_MPP_SET_SPEED;

/**
 * @brief MPP set state command param
 */
typedef struct
{
	uint8_t ID;
	uint8_t state;
}MSG_PARAM_MPP_SET_STATE;

/**
 * @brief MPP move command param
 */
typedef struct
{
	uint8_t ID;
	uint8_t direction;
	uint16_t nbSteps;
}MSG_PARAM_MPP_MOVE;

/**
 * @brief MPP run command param
 */
typedef struct
{
	uint8_t ID;
	uint8_t direction;
}MSG_PARAM_MPP_RUN;

/**
 * @brief MPP command param union
 */
typedef union
{
	MSG_PARAM_INIT			Init;
	MSG_PARAM_GET_STATUS	GetStatus;
	MSG_PARAM_MPP_SET_SPEED	SetSpeed;
	MSG_PARAM_MPP_SET_STATE	SetState;
	MSG_PARAM_MPP_MOVE		Move;
	MSG_PARAM_MPP_RUN		Run;
}MSG_PARAM_MPP;

/**
 * @brief GPIO read all digital input command param
 */
typedef struct
{
	uint16_t inputs;
}MSG_PARAM_GPIO_READ_ALL_DIG_IN;

/**
 * @brief GPIO read all analog input command param
 */
typedef struct
{
	uint16_t input[MSG_PARAM_GPIO_MAX_ANALOG_INPUTS];
}MSG_PARAM_GPIO_READ_ALL_AN_IN;

/**
 * @brief  GPIO get output command param
 */
typedef struct
{
	uint8_t outputs;
}MSG_PARAM_GPIO_GET_OUTPUT;

/**
 * @brief  GPIO set output command param
 */
typedef struct
{
	uint8_t id;
	uint8_t state;
}MSG_PARAM_GPIO_SET_OUTPUT;

/**
 * @brief MPP command param union
 */
typedef union
{
	MSG_PARAM_INIT					Init;
	MSG_PARAM_GET_STATUS			GetStatus;
	MSG_PARAM_GPIO_READ_ALL_DIG_IN	ReadAllDigInput;
	MSG_PARAM_GPIO_READ_ALL_AN_IN	ReadAllAnInput;
	MSG_PARAM_GPIO_GET_OUTPUT		Getoutput;
	MSG_PARAM_GPIO_SET_OUTPUT		SetOutput;
}MSG_PARAM_GPIO;

/**
 * @brief  Barillet move to index command param
 */
typedef struct
{
	uint8_t id;
	uint8_t index;
}MSG_PARAM_BAR_MOVE_INDEX;

/**
 * @brief  Barillet get status command param
 */
typedef struct
{
	uint16_t status;
}MSG_PARAM_BAR_GET_STATUS;

/**
 * @brief Barillet command param union
 */
typedef union
{
	MSG_PARAM_INIT				Init;
	MSG_PARAM_BAR_MOVE_INDEX 	MoveIndex;
	MSG_PARAM_BAR_GET_STATUS	GetStatus;
}MSG_PARAM_BAR;

/**
 * @brief Commands param union
 */
typedef union
{
	MSG_PARAM_RESET			Reset;
	MSG_PARAM_BOOT_MODE		BootMode;
	MSG_PARAM_PING			Ping;
	MSG_PARAM_CHANGE_ADDR 	ChangeAddress;

	MSG_PARAM_PROP			Prop;
	MSG_PARAM_SERVO			Servo;
	MSG_PARAM_MPP			MPP;
	MSG_PARAM_GPIO			GPIO;
	MSG_PARAM_BAR			Barillet;
}MSG_PARAM;

/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{

	class Message
	{
	public :
		/**
		 * @brief Create a new message without message type (unknown by default)
		 */
		Message ();

		/**
		 * @brief Create a new message with a specific message type
		 * @param type : Message type
		 */
		Message (MESSAGE_TYPE type);

		/**
		 * @brief Encode message
		 * @param frame : Encoded I2C frame
		 * @return 0 if no error, < 0 else
		 */
		int32_t	Encode (uint8_t* data, uint8_t length);

		/**
		 * @brief Decode message
		 * @param frame : Received I2C frame
		 * @return 0 if no error, < 0 else
		 */
		int32_t Decode (uint8_t* data, uint8_t length);

		/**
		 * @brief Current message type
		 */
		MESSAGE_TYPE Type;

		/**
		 * @brief Current message param
		 */
		MSG_PARAM Param;

	private :

		int32_t GetType (uint8_t* data, uint8_t length);

		int32_t GetParam (uint8_t* data, uint8_t length);
	};
}

#endif /* INC_MESSAGE_HPP_ */
