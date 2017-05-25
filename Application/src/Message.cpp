/**
 * @file	Message.hpp
 * @author	Kevin Wysocki
 * @date	20 avr. 2017
 * @brief	Message Class
 *
 *	Used to encode and decode messages
 *	written or read on the I2C bus
 */

#include <stddef.h>
#include "Message.hpp"
#include <string>
#include <stdlib.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MSG_OPCODE_FW						("F")
#define MSG_OPCODE_PROP						("P")

#define MSG_OPCODE_FW_RESET					("FRST")
#define MSG_OPCODE_FW_PING					("FPNG")

#define MSG_OPCODE_PROP_INIT				("PINIT")
#define MSG_OPCODE_PROP_GETPOSITION			("PGPO")
#define MSG_OPCODE_PROP_GOTOXY				("PGXY")
#define MSG_OPCODE_PROP_GOLINEAR			("PGLI")
#define MSG_OPCODE_PROP_ROTATE				("PROT")
#define MSG_OPCODE_PROP_SETANGLE			("PSAN")
#define MSG_OPCODE_PROP_SETPOSITION			("PSPO")
#define MSG_OPCODE_PROP_STOP				("PSTP")
#define MSG_OPCODE_PROP_SETSTATE			("PSST")
#define MSG_OPCODE_PROP_GETSTATUS			("PGST")

#define MSG_FRAME_INDEX_OPCODE		(0u)
#define MSG_FRAME_INDEX_NB_DATA		(1u)
#define MSG_FRAME_INDEX_FIRST_DATA	(2u)

// Message nb of data bytes - ENCODE
#define MSG_NB_DATA_ENCODE_FW_RESET					(0u)
#define MSG_NB_DATA_ENCODE_FW_BOOT_MODE			    (0u)
#define MSG_NB_DATA_ENCODE_FW_PING					(4u)
#define MSG_NB_DATA_ENCODE_FW_CHANGE_ADDR			(0u)
#define MSG_NB_DATA_ENCODE_FW_CHECKUP				(0u)

#define MSG_NB_DATA_ENCODE_PROP_INIT				(0u)
#define MSG_NB_DATA_ENCODE_PROP_GET_POSITION		(6u)
#define MSG_NB_DATA_ENCODE_PROP_GOTO_XY				(0u)
#define MSG_NB_DATA_ENCODE_PROP_GO_LINEAR			(0u)
#define MSG_NB_DATA_ENCODE_PROP_ROTATE				(0u)
#define MSG_NB_DATA_ENCODE_PROP_SET_ANGLE			(0u)
#define MSG_NB_DATA_ENCODE_PROP_SET_POSITION		(0u)
#define MSG_NB_DATA_ENCODE_PROP_STOP				(0u)
#define MSG_NB_DATA_ENCODE_PROP_SET_POS_CONTROL		(0u)
#define MSG_NB_DATA_ENCODE_PROP_GET_STATUS			(2u)

#define MSG_NB_DATA_ENCODE_SERVO_INIT				(0u)
#define MSG_NB_DATA_ENCODE_SERVO_SET_ANGLE			(0u)
#define MSG_NB_DATA_ENCODE_SERVO_SET_SPEED			(0u)
#define MSG_NB_DATA_ENCODE_SERVO_GET_STATUS			(2u)

#define MSG_NB_DATA_ENCODE_MPP_INIT					(0u)
#define MSG_NB_DATA_ENCODE_MPP_SET_SPEED			(0u)
#define MSG_NB_DATA_ENCODE_MPP_SET_STATE			(0u)
#define MSG_NB_DATA_ENCODE_MPP_MOVE					(0u)
#define MSG_NB_DATA_ENCODE_MPP_RUN					(2u)
#define MSG_NB_DATA_ENCODE_MPP_GET_STATUS			(2u)


#define MSG_NB_DATA_ENCODE_GPIO_INIT				(0u)
#define MSG_NB_DATA_ENCODE_GPIO_READ_ALL_DIG_IN		(2u)
#define MSG_NB_DATA_ENCODE_GPIO_READ_ALL_AN_IN		(20u)
#define MSG_NB_DATA_ENCODE_GPIO_GET_OUTPUTS			(1u)
#define MSG_NB_DATA_ENCODE_GPIO_SET_OUTPUTS			(0u)
#define MSG_NB_DATA_ENCODE_GPIO_GET_STATUS			(2u)

#define MSG_NB_DATA_ENCODE_BAR_INIT					(0u)
#define MSG_NB_DATA_ENCODE_BAR_MOVE_INDEX			(0u)
#define MSG_NB_DATA_ENCODE_BAR_GET_STATUS			(2u)

// Message nb of data bytes - Decode
#define MSG_NB_DATA_DECODE_FW_RESET					(4u)
#define MSG_NB_DATA_DECODE_FW_BOOT_MODE				(4u)
#define MSG_NB_DATA_DECODE_FW_PING					(0u)
#define MSG_NB_DATA_DECODE_FW_CHANGE_ADDR			(1u)
#define MSG_NB_DATA_DECODE_FW_CHECKUP				(0u)

#define MSG_NB_DATA_DECODE_PROP_INIT				(1u)
#define MSG_NB_DATA_DECODE_PROP_GET_POSITION		(0u)
#define MSG_NB_DATA_DECODE_PROP_GOTO_XY				(4u)
#define MSG_NB_DATA_DECODE_PROP_GO_LINEAR			(2u)
#define MSG_NB_DATA_DECODE_PROP_ROTATE				(2u)
#define MSG_NB_DATA_DECODE_PROP_SET_ANGLE			(2u)
#define MSG_NB_DATA_DECODE_PROP_SET_POSITION		(4u)
#define MSG_NB_DATA_DECODE_PROP_STOP				(1u)
#define MSG_NB_DATA_DECODE_PROP_SET_POS_CONTROL		(1u)
#define MSG_NB_DATA_DECODE_PROP_GET_STATUS			(0u)

#define MSG_NB_DATA_DECODE_SERVO_INIT				(0u)
#define MSG_NB_DATA_DECODE_SERVO_SET_ANGLE			(3u)
#define MSG_NB_DATA_DECODE_SERVO_SET_SPEED			(3u)
#define MSG_NB_DATA_DECODE_SERVO_GET_STATUS			(0u)

#define MSG_NB_DATA_DECODE_MPP_INIT					(0u)
#define MSG_NB_DATA_DECODE_MPP_SET_SPEED			(3u)
#define MSG_NB_DATA_DECODE_MPP_SET_STATE			(2u)
#define MSG_NB_DATA_DECODE_MPP_MOVE					(4u)
#define MSG_NB_DATA_DECODE_MPP_RUN					(2u)
#define MSG_NB_DATA_DECODE_MPP_GET_STATUS			(0u)


#define MSG_NB_DATA_DECODE_GPIO_INIT				(1u)
#define MSG_NB_DATA_DECODE_GPIO_READ_ALL_DIG_IN		(0u)
#define MSG_NB_DATA_DECODE_GPIO_READ_ALL_AN_IN		(0u)
#define MSG_NB_DATA_DECODE_GPIO_GET_OUTPUTS			(0u)
#define MSG_NB_DATA_DECODE_GPIO_SET_OUTPUTS			(2u)
#define MSG_NB_DATA_DECODE_GPIO_GET_STATUS			(0u)

#define MSG_NB_DATA_DECODE_BAR_INIT					(1u)
#define MSG_NB_DATA_DECODE_BAR_MOVE_INDEX			(2u)
#define MSG_NB_DATA_DECODE_BAR_GET_STATUS			(0u)



/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static void _decode_Reset (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Reset.key	 =	(uint32_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Reset.key	|=	(uint32_t)(atoi(s.c_str()) << 8u);
	s.erase(0,4);
	param->Reset.key	|=	(uint32_t)(atoi(s.c_str()) << 16u);
	s.erase(0,4);
	param->Reset.key	|=	(uint32_t)(atoi(s.c_str()) << 24u);
	s.erase(0,4);
}

static void _decode_BootMode (std::string s, MSG_PARAM * param)
{
//	assert(frame != NULL);
//	assert(param != NULL);
//
//	param->BootMode.key	 =	(uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->BootMode.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
//	param->BootMode.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u] << 16u);
//	param->BootMode.key	|=	((uint32_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u] << 24u);
}

static void _decode_ChangeAddr (std::string s, MSG_PARAM * param)
{
//	assert(frame != NULL);
//	assert(param != NULL);
//
//	param->ChangeAddress.addr	 =	(uint8_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
}

static void _decode_Prop_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.Init.id		 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_Prop_GotoXY (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.GotoXY.posX		 =	(uint16_t)(atoi(s.c_str()));
	param->Prop.GotoXY.posX		|=	((uint16_t)(atoi(s.c_str()) << 8u);
	param->Prop.GotoXY.posY		 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u];
	param->Prop.GotoXY.posY		|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u] << 8u);
}

static void _decode_Prop_GotoLinear (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Prop.GoLinear.distance		 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Prop.GoLinear.distance		|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
}

static void _decode_Prop_Rotate (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Prop.Rotate.angle 		 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Prop.Rotate.angle		|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
}

static void _decode_Prop_SetAngle (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Prop.SetAngle.angle	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Prop.SetAngle.angle	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
}

static void _decode_Prop_SetPosition (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Prop.SetPosition.posX	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Prop.SetPosition.posX	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u] << 8u);
//	param->Prop.SetPosition.posY	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u];
//	param->Prop.SetPosition.posY	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u] << 8u);
}

static void _decode_Prop_Stop (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Prop.Stop.mode	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
}

static void _decode_Prop_SetState (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Prop.SetState.state	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
}

static void _decode_Servo_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Servo.Init.id	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
}

static void _decode_Servo_SetAngle (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Servo.SetAngle.ID	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Servo.SetAngle.angle	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
//	param->Servo.SetAngle.angle	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u] << 8u);
}


static void _decode_Servo_SetSpeed (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Servo.SetSpeed.ID	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Servo.SetSpeed.speed	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
//	param->Servo.SetSpeed.speed	|=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u] << 8u);
}

static void _decode_MPP_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->MPP.Init.id	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
}

static void _decode_MPP_SetSpeed (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->MPP.SetSpeed.ID	 	=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->MPP.SetSpeed.speed	=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
//	param->MPP.SetSpeed.speed   |=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u] << 8u);
}

static void _decode_MPP_SetState (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->MPP.SetState.ID	 	=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->MPP.SetState.state	=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
}

static void _decode_MPP_Move (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->MPP.Move.ID	 		=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->MPP.Move.direction	=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
//	param->MPP.Move.nbSteps		=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 2u];
//	param->MPP.Move.nbSteps    |=	((uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 3u] << 8u);
}

static void _decode_MPP_Run (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->MPP.Run.ID	 		=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->MPP.Run.direction	=	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
}

static void _decode_GPIO_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->GPIO.Init.id	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
}

static void _decode_GPIO_SetOutput (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->GPIO.SetOutput.id	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->GPIO.SetOutput.state	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
}

static void _decode_BAR_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Barillet.Init.id	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Barillet.MoveIndex.index	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
}

static void _decode_BAR_MoveIndex (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	param->Barillet.MoveIndex.id	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA];
//	param->Barillet.MoveIndex.index	 =	(uint16_t)frame->Data[MSG_FRAME_INDEX_FIRST_DATA + 1u];
}

static void _encode_Ping (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

//	frame->Data[frame->Length++]	=	(uint8_t)(param->Ping.key & 0x000000FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Ping.key >> 8u) & 0x000000FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Ping.key >> 16u) & 0x000000FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Ping.key >> 24u) & 0x000000FFu);
}

static void _encode_Checkup(std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	param->Checkup.cmdStatus;
//	frame->Data[frame->Length++]	=	param->Checkup.cmdStatus;
}

static void _encode_GetDistance (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->GetDistance.distance & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->GetDistance.distance >> 8u) & 0x00FFu);
//	frame->Data[frame->Length++]	=	param->GetDistance.status;
}

static void _encode_Prop_GetPosition (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->Prop.GetPosition.posX & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Prop.GetPosition.posX >> 8u) & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)(param->Prop.GetPosition.posY & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Prop.GetPosition.posY >> 8u) & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)(param->Prop.GetPosition.angle & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Prop.GetPosition.angle >> 8u) & 0x00FFu);
}

static void _encode_Prop_GetStatus (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->Prop.GetStatus.status & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Prop.GetStatus.status >> 8u) & 0x00FFu);
}

static void _encode_Servo_GetStatus (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->Servo.GetStatus.status & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Servo.GetStatus.status >> 8u) & 0x00FFu);
}

static void _encode_MPP_GetStatus (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->MPP.GetStatus.status & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->MPP.GetStatus.status >> 8u) & 0x00FFu);
}

static void _encode_GPIO_ReadAllDigInput (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->GPIO.ReadAllDigInput.inputs & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->GPIO.ReadAllDigInput.inputs >> 8u) & 0x00FFu);
}

static void _encode_GPIO_ReadAllAnInput(std::string s, MSG_PARAM * param)
{
	uint8_t i = 0u;

	for(i=0u; i<MSG_PARAM_GPIO_MAX_ANALOG_INPUTS; i++)
	{
//		frame->Data[frame->Length++]	=	(uint8_t)(param->GPIO.ReadAllAnInput.input[i] & 0x00FFu);
	}
}

static void _encode_GPIO_GetOutput (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->GPIO.Getoutput.outputs & 0x00FFu);
}

static void _encode_GPIO_GetStatus (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->GPIO.GetStatus.status & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->GPIO.GetStatus.status >> 8u) & 0x00FFu);
}

static void _encode_BAR_GetStatus (std::string s, MSG_PARAM * param)
{
//	frame->Data[frame->Length++]	=	(uint8_t)(param->Barillet.GetStatus.status & 0x00FFu);
//	frame->Data[frame->Length++]	=	(uint8_t)((param->Barillet.GetStatus.status >> 8u) & 0x00FFu);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

namespace Communication
{
	Message::Message()
	{
		this->Type = MSG_TYPE_UNKNOWN;
	}

	Message::Message(MESSAGE_TYPE type)
	{
		this->Type = type;
	}

	int32_t Message::Decode(uint8_t* data, uint8_t length)
	{
		int32_t rval = NO_ERROR;
//
		assert(data != NULL);
		assert(length > 0);

		// 1. Get message type
		if(rval == NO_ERROR)
		{
			rval = this->GetType(data, length);;
		}
//
//		// 2. Verify CRC
//		if(rval == NO_ERROR)
//		{
//			// If frame contains a CRC
//			if(frame->Length > MSG_FRAME_INDEX_NB_DATA)
//			{
//				if(frame->Data[frame->Length - 1u] != frame->CRCval)
//				{
//					rval = MSG_ERROR_WRONG_CRC;
//				}
//			}
//		}
//
//		// 3. Get param
		if(rval == NO_ERROR)
		{
			rval = this->GetParam(data, length);
		}

		return rval;
	}

	int32_t Message::Encode(uint8_t* data, uint8_t length)
	{
		int32_t rval = 0;

//		assert(frame != NULL);
//
//		// 1. Set Opcode field
//		if(rval == NO_ERROR)
//		{
//			if(this->Type != MSG_TYPE_UNKNOWN)
//			{
//				frame->Length = MSG_FRAME_INDEX_OPCODE;
//				frame->Data[frame->Length++]	=	(uint8_t)this->Type;
//			}
//			else
//			{
//				rval = MSG_ERROR_UNKNOWN_TYPE;
//			}
//		}
//
//		// 2. Set Nb Data field
//		if(rval == NO_ERROR)
//		{
//			switch(this->Type)
//			{
//			case MSG_TYPE_FW_PING:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_FW_PING;
//				break;
////			case MSG_TYPE_CHECKUP:
////				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_CHECKUP;
////				break;
////			case MSG_TYPE_GET_DISTANCE:
////				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_GET_DISTANCE;
////				break;
//
//			case MSG_TYPE_PROP_GET_POSITION:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_PROP_GET_POSITION;
//				break;
//			case MSG_TYPE_PROP_GET_STATUS:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_PROP_GET_STATUS;
//				break;
//			case MSG_TYPE_SERVO_GET_STATUS:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_SERVO_GET_STATUS;
//				break;
//			case MSG_TYPE_MPP_GET_STATUS:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_MPP_GET_STATUS;
//				break;
//			case MSG_TYPE_GPIO_READ_ALL_DIG_IN:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_GPIO_READ_ALL_DIG_IN;
//				break;
//			case MSG_TYPE_GPIO_READ_ALL_AN_IN:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_GPIO_READ_ALL_AN_IN;
//				break;
//			case MSG_TYPE_GPIO_GET_OUTPUT:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_GPIO_GET_OUTPUTS;
//				break;
//			case MSG_TYPE_GPIO_GET_STATUS:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_GPIO_GET_STATUS;
//				break;
//			case MSG_TYPE_BAR_GET_STATUS:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_BAR_GET_STATUS;
//				break;
//
//			// Other commands doesn't need an answer
//			default:
//				rval = MSG_ERROR_NO_ANSWER_NEEDED;
//				break;
//			}
//		}
//
//		// 3. Encode data
//		if(rval == NO_ERROR)
//		{
//			switch(this->Type)
//			{
//			case MSG_TYPE_FW_PING:
//				_encode_Ping(frame, &this->Param);
//				break;
////			case MSG_TYPE_CHECKUP:
////				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_CHECKUP;
////				break;
////			case MSG_TYPE_GET_DISTANCE:
////				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_GET_DISTANCE;
////				break;
//
//			case MSG_TYPE_PROP_GET_POSITION:
//				_encode_Prop_GetPosition(frame, &this->Param);
//				break;
//			case MSG_TYPE_PROP_GET_STATUS:
//				_encode_Prop_GetStatus(frame, &this->Param);
//				break;
//			case MSG_TYPE_SERVO_GET_STATUS:
//				_encode_Servo_GetStatus(frame, &this->Param);
//				break;
//			case MSG_TYPE_MPP_GET_STATUS:
//				_encode_MPP_GetStatus(frame, &this->Param);
//				break;
//			case MSG_TYPE_GPIO_READ_ALL_DIG_IN:
//				_encode_GPIO_ReadAllDigInput(frame, &this->Param);
//				break;
//			case MSG_TYPE_GPIO_READ_ALL_AN_IN:
//				_encode_GPIO_ReadAllAnInput(frame, &this->Param);
//				break;
//			case MSG_TYPE_GPIO_GET_OUTPUT:
//				_encode_GPIO_GetOutput(frame, &this->Param);
//				break;
//			case MSG_TYPE_GPIO_GET_STATUS:
//				_encode_GPIO_GetStatus(frame, &this->Param);
//				break;
//			case MSG_TYPE_BAR_GET_STATUS:
//				_encode_BAR_GetStatus(frame, &this->Param);
//				break;
//
//		// Other commands doesn't need an answer
//		default:
//			rval = MSG_ERROR_NO_ANSWER_NEEDED;
//			break;
//}
//		}

		return rval;
	}

	int32_t Message::GetType(uint8_t* data, uint8_t length)
	{
		int32_t rval = 0;
		std::string s((const char*)data, (size_t)length);

		assert(data != NULL);
		assert(length > 0);

		// Firmware module
		if(s.find(MSG_OPCODE_FW, MSG_FRAME_INDEX_OPCODE, 1) != std::string::npos)
		{
			if(s.find(MSG_OPCODE_FW_RESET, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_FW_RESET;
			}
			else if(s.find(MSG_OPCODE_FW_PING, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_FW_PING;
			}
		}
		// Prop module
		else if(s.find(MSG_OPCODE_PROP, MSG_FRAME_INDEX_OPCODE, 1) != std::string::npos)
		{
			if(s.find(MSG_OPCODE_PROP_INIT, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_INIT;
			}
			else if(s.find(MSG_OPCODE_PROP_GETPOSITION, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_GET_POSITION;
			}
			else if(s.find(MSG_OPCODE_PROP_GOTOXY, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_GOTOXY;
			}
			else if(s.find(MSG_OPCODE_PROP_GOLINEAR, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_GOLINEAR;
			}
			else if(s.find(MSG_OPCODE_PROP_ROTATE, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_ROTATE;
			}
			else if(s.find(MSG_OPCODE_PROP_SETANGLE, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_SET_ANGLE;
			}
			else if(s.find(MSG_OPCODE_PROP_SETPOSITION, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_SET_POSITION;
			}
			else if(s.find(MSG_OPCODE_PROP_STOP, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_STOP;
			}
			else if(s.find(MSG_OPCODE_PROP_SETSTATE, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_SET_STATE;
			}
			else if(s.find(MSG_OPCODE_PROP_GETSTATUS, MSG_FRAME_INDEX_OPCODE, 4) != std::string::npos)
			{
				this->Type = MSG_TYPE_PROP_GET_STATUS;
			}
		}
		else
		{
			this->Type = MSG_TYPE_UNKNOWN;
			rval = MSG_ERROR_UNKNOWN_TYPE;
		}

		return rval;
	}

	int32_t Message::GetParam(uint8_t* data, uint8_t length)
	{
		int32_t rval = NO_ERROR;
		std::string s((const char*)data, (size_t)length);
		uint32_t frameLength = 0u;

		s.erase(0, 5);
		frameLength = atoi(s.c_str());

		// 1. Check Nb data
		if(rval == NO_ERROR)
		{
			switch(this->Type)
			{
			case MSG_TYPE_FW_RESET:
			    if(frameLength != MSG_NB_DATA_DECODE_FW_RESET)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_FW_BOOT_MODE:
                if(frameLength != MSG_NB_DATA_DECODE_FW_RESET)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_FW_PING:
                if(frameLength != MSG_NB_DATA_DECODE_FW_PING)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_FW_CHANGE_ADDR:
                if(frameLength != MSG_NB_DATA_DECODE_FW_CHANGE_ADDR)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_INIT:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_INIT)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_GOTOXY:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_GOTO_XY)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_GOLINEAR:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_GO_LINEAR)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_ROTATE:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_ROTATE)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_SET_ANGLE:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_SET_ANGLE)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_SET_POSITION:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_SET_POSITION)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_STOP:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_STOP)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_PROP_SET_STATE:
                if(frameLength != MSG_NB_DATA_DECODE_PROP_SET_POS_CONTROL)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_SERVO_INIT:
                if(frameLength != MSG_NB_DATA_DECODE_SERVO_INIT)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_SERVO_SET_ANGLE:
                if(frameLength != MSG_NB_DATA_DECODE_SERVO_SET_ANGLE)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_SERVO_SET_SPEED:
                if(frameLength != MSG_NB_DATA_DECODE_SERVO_SET_SPEED)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_MPP_INIT:
                if(frameLength != MSG_NB_DATA_DECODE_MPP_INIT)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_MPP_SET_SPEED:
                if(frameLength != MSG_NB_DATA_DECODE_MPP_SET_SPEED)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_MPP_SET_STATE:
                if(frameLength != MSG_NB_DATA_DECODE_MPP_SET_STATE)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_MPP_MOVE:
                if(frameLength != MSG_NB_DATA_DECODE_MPP_MOVE)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_MPP_RUN:
                if(frameLength != MSG_NB_DATA_DECODE_MPP_RUN)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_GPIO_INIT:
                if(frameLength != MSG_NB_DATA_DECODE_GPIO_INIT)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_GPIO_SET_OUTPUT:
                if(frameLength != MSG_NB_DATA_DECODE_GPIO_SET_OUTPUTS)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_BAR_INIT:
                if(frameLength != MSG_NB_DATA_DECODE_BAR_INIT)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;
            case MSG_TYPE_BAR_MOVE_INDEX:
                if(frameLength != MSG_NB_DATA_DECODE_BAR_MOVE_INDEX)
                    rval = MSG_ERROR_WRONG_NB_DATA;
                break;


			default:
				rval = MSG_ERROR_UNKNOWN_TYPE;
				break;
			}
		}

		// 2. Decode param
		if((rval == NO_ERROR) && (frameLength> 0u))
		{
			s.erase(0, 3);

			switch(this->Type)
			{
			case MSG_TYPE_FW_RESET:
				_decode_Reset(s, &this->Param);
				break;
			case MSG_TYPE_FW_BOOT_MODE:
				_decode_BootMode(s, &this->Param);
				break;
			case MSG_TYPE_FW_CHANGE_ADDR:
				_decode_ChangeAddr(s, &this->Param);
				break;
			case MSG_TYPE_PROP_INIT:
				_decode_Prop_Init(s, &this->Param);
				break;
			case MSG_TYPE_PROP_GOTOXY:
				_decode_Prop_GotoXY(s, &this->Param);
				break;
			case MSG_TYPE_PROP_GOLINEAR:
				_decode_Prop_GotoLinear(s, &this->Param);
				break;
			case MSG_TYPE_PROP_ROTATE:
				_decode_Prop_Rotate(s, &this->Param);
				break;
			case MSG_TYPE_PROP_SET_ANGLE:
				_decode_Prop_SetAngle(s, &this->Param);
				break;
			case MSG_TYPE_PROP_SET_POSITION:
				_decode_Prop_SetPosition(s, &this->Param);
				break;
			case MSG_TYPE_PROP_STOP:
				_decode_Prop_Stop(s, &this->Param);
				break;
			case MSG_TYPE_PROP_SET_STATE:
				_decode_Prop_SetState(s, &this->Param);
				break;
			case MSG_TYPE_SERVO_INIT:
				_decode_Servo_Init(s, &this->Param);
				break;
			case MSG_TYPE_SERVO_SET_ANGLE:
				_decode_Servo_SetAngle(s, &this->Param);
				break;
			case MSG_TYPE_SERVO_SET_SPEED:
				_decode_Servo_SetSpeed(s, &this->Param);
				break;
			case MSG_TYPE_MPP_INIT:
				_decode_MPP_Init(s, &this->Param);
				break;
			case MSG_TYPE_MPP_SET_SPEED:
				_decode_MPP_SetSpeed(s, &this->Param);
				break;
			case MSG_TYPE_MPP_SET_STATE:
				_decode_MPP_SetState(s, &this->Param);
				break;
			case MSG_TYPE_MPP_MOVE:
				_decode_MPP_Move(s, &this->Param);
				break;
			case MSG_TYPE_MPP_RUN:
				_decode_MPP_Run(s, &this->Param);
				break;
			case MSG_TYPE_GPIO_INIT:
				_decode_GPIO_Init(s, &this->Param);
				break;
			case MSG_TYPE_GPIO_SET_OUTPUT:
				_decode_GPIO_SetOutput(s, &this->Param);
				break;
			case MSG_TYPE_BAR_INIT:
				_decode_BAR_Init(s, &this->Param);
				break;
			case MSG_TYPE_BAR_MOVE_INDEX:
				_decode_BAR_MoveIndex(s, &this->Param);
				break;
			default:	// Other messages doesn't need param
				break;
			}
		}

		return rval;
	}
}
