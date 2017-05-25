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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define MSG_OPCODE_FW						("F")
#define MSG_OPCODE_PROP						("P")
#define MSG_OPCODE_SERVO					("S")
#define MSG_OPCODE_MPP						("M")
#define MSG_OPCODE_GPIO						("I")
#define MSG_OPCODE_BAR						("B")

#define MSG_OPCODE_FW_RESET					("FRST")
#define MSG_OPCODE_FW_PING					("FPNG")

#define MSG_OPCODE_PROP_INIT				("PINI")
#define MSG_OPCODE_PROP_GETPOSITION			("PGPO")
#define MSG_OPCODE_PROP_GOTOXY				("PGXY")
#define MSG_OPCODE_PROP_GOLINEAR			("PGLI")
#define MSG_OPCODE_PROP_ROTATE				("PROT")
#define MSG_OPCODE_PROP_SETANGLE			("PSAN")
#define MSG_OPCODE_PROP_SETPOSITION			("PSPO")
#define MSG_OPCODE_PROP_STOP				("PSTP")
#define MSG_OPCODE_PROP_SETSTATE			("PSST")
#define MSG_OPCODE_PROP_GETSTATUS			("PGST")

#define MSG_OPCODE_SERVO_INIT				("SINI")
#define MSG_OPCODE_SERVO_SETANGLE			("SSAN")
#define MSG_OPCODE_SERVO_SETSPEED			("SSPD")
#define MSG_OPCODE_SERVO_GETSTATUS			("SGST")

#define MSG_OPCODE_MPP_INIT					("MINI")
#define MSG_OPCODE_MPP_SETSPEED				("MSPD")
#define MSG_OPCODE_MPP_SETSTATE				("MSST")
#define MSG_OPCODE_MPP_MOVE					("MMOV")
#define MSG_OPCODE_MPP_RUN					("MRUN")
#define MSG_OPCODE_MPP_GETSTATUS			("MGST")

#define MSG_OPCODE_GPIO_INIT				("IINI")
#define MSG_OPCODE_GPIO_READALLDIGIN		("IRDI")
#define MSG_OPCODE_GPIO_READALLANIN			("IRAI")
#define MSG_OPCODE_GPIO_GETOUTPUTS			("IGOT")
#define MSG_OPCODE_GPIO_SETOUTPUTS			("ISOT")
#define MSG_OPCODE_GPIO_GETSTATUS			("IGST")

#define MSG_OPCODE_BAR_INIT					("BINI")
#define MSG_OPCODE_BAR_MOVEINDEX			("BMIX")
#define MSG_OPCODE_BAR_GETSTATUS			("BGST")


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
	s.erase(0,4);
	param->Prop.GotoXY.posX		|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
	param->Prop.GotoXY.posY		 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Prop.GotoXY.posY		|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_Prop_GotoLinear (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.GoLinear.distance		 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Prop.GoLinear.distance		|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_Prop_Rotate (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.Rotate.angle 		 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Prop.Rotate.angle		|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_Prop_SetAngle (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.SetAngle.angle	 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Prop.SetAngle.angle	|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_Prop_SetPosition (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.SetPosition.posX	 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Prop.SetPosition.posX	|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
	param->Prop.SetPosition.posY	 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Prop.SetPosition.posY	|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_Prop_Stop (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.Stop.mode	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_Prop_SetState (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Prop.SetState.state	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_Servo_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Servo.Init.id	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_Servo_SetAngle (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Servo.SetAngle.ID	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Servo.SetAngle.angle	 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Servo.SetAngle.angle	|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_Servo_SetSpeed (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Servo.SetSpeed.ID	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Servo.SetSpeed.speed	 =	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Servo.SetSpeed.speed	|=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_MPP_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->MPP.Init.id	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_MPP_SetSpeed (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->MPP.SetSpeed.ID	 	=	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->MPP.SetSpeed.speed	=	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->MPP.SetSpeed.speed   |=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_MPP_SetState (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->MPP.SetState.ID	 	=	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->MPP.SetState.state	=	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_MPP_Move (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->MPP.Move.ID	 		=	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->MPP.Move.direction	=	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->MPP.Move.nbSteps		=	(uint16_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->MPP.Move.nbSteps    |=	((uint16_t)(atoi(s.c_str()) << 8u));
	s.erase(0,4);
}

static void _decode_MPP_Run (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->MPP.Run.ID	 		=	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->MPP.Run.direction	=	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_GPIO_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->GPIO.Init.id	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_GPIO_SetOutput (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->GPIO.SetOutput.id	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->GPIO.SetOutput.state	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_BAR_Init (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Barillet.Init.id	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static void _decode_BAR_MoveIndex (std::string s, MSG_PARAM * param)
{
	assert(param != NULL);

	param->Barillet.MoveIndex.id	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
	param->Barillet.MoveIndex.index	 =	(uint8_t)(atoi(s.c_str()));
	s.erase(0,4);
}

static std::string _encode_Ping (MSG_PARAM * param)
{
	assert(param != NULL);

	std::string s(MSG_OPCODE_FW_PING);
	s.append(",04,170,085,165,090");

	return s;
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

static std::string _encode_Prop_GetPosition (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_PROP_GETPOSITION);

	sprintf(buffer, "06,%03d,%03d,%03d,%03d,%03d,%03d", (uint8_t)(param->Prop.GetPosition.posX & 0xFFu),
														((uint8_t)(param->Prop.GetPosition.posX >> 8u) & 0xFFu),
														(uint8_t)(param->Prop.GetPosition.posY & 0xFFu),
														((uint8_t)(param->Prop.GetPosition.posY >> 8u) & 0xFFu),
														(uint8_t)(param->Prop.GetPosition.angle & 0xFFu),
														((uint8_t)(param->Prop.GetPosition.angle >> 8u) & 0xFFu));

	s.append(buffer);

	return s;
}

static std::string _encode_Prop_GetStatus (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_PROP_GETSTATUS);

	sprintf(buffer, "02,%03d,%03d", (uint8_t)(param->Prop.GetStatus.status & 0xFFu),
									((uint8_t)(param->Prop.GetStatus.status >> 8u) & 0xFFu));

	s.append(buffer);

	return s;
}

static std::string _encode_Servo_GetStatus (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_SERVO_GETSTATUS);

	sprintf(buffer, "02,%03d,%03d", (uint8_t)(param->Servo.GetStatus.status & 0xFFu),
									((uint8_t)(param->Servo.GetStatus.status >> 8u) & 0xFFu));

	s.append(buffer);

	return s;
}

static std::string _encode_MPP_GetStatus (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_MPP_GETSTATUS);

	sprintf(buffer, "02,%03d,%03d", (uint8_t)(param->MPP.GetStatus.status & 0xFFu),
									((uint8_t)(param->MPP.GetStatus.status >> 8u) & 0xFFu));

	s.append(buffer);

	return s;
}


static std::string _encode_GPIO_ReadAllDigInput (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_GPIO_READALLDIGIN);

	sprintf(buffer, "02,%03d,%03d", (uint8_t)(param->GPIO.ReadAllDigInput.inputs & 0xFFu),
									((uint8_t)(param->GPIO.ReadAllDigInput.inputs >> 8u) & 0xFFu));

	s.append(buffer);

	return s;
}


static std::string _encode_GPIO_ReadAllAnInput(MSG_PARAM * param)
{
//	char buffer[32];
//
//	assert(param != NULL);
//
//	std::string s(MSG_OPCODE_GPIO_READALLDIGIN);
//
//	sprintf(buffer, "04,%03d,%03d", (uint8_t)(param->GPIO.ReadAllDigInput.inputs & 0xFFu),
//									((uint8_t)(param->GPIO.ReadAllDigInput.inputs >> 8u) & 0xFFu));
//
//	return s;
}


static std::string _encode_GPIO_GetOutput (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_GPIO_GETOUTPUTS);

	sprintf(buffer, "02,%03d,%03d", (uint8_t)(param->GPIO.Getoutput.outputs & 0xFFu),
									((uint8_t)(param->GPIO.Getoutput.outputs >> 8u) & 0xFFu));

	s.append(buffer);

	return s;
}


static std::string _encode_GPIO_GetStatus (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_GPIO_GETSTATUS);

	sprintf(buffer, "02,%03d,%03d", (uint8_t)(param->GPIO.GetStatus.status & 0xFFu),
									((uint8_t)(param->GPIO.GetStatus.status >> 8u) & 0xFFu));

	s.append(buffer);

	return s;

}

static std::string _encode_BAR_GetStatus (MSG_PARAM * param)
{
	char buffer[32];

	assert(param != NULL);

	std::string s(MSG_OPCODE_BAR_GETSTATUS);

	sprintf(buffer, "02,%03d,%03d", (uint8_t)(param->Barillet.GetStatus.status & 0xFFu),
									((uint8_t)(param->Barillet.GetStatus.status >> 8u) & 0xFFu));

	s.append(buffer);

	return s;
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

	int32_t Message::Encode(uint8_t* data)
	{
		int32_t rval = 0;
		std::string s;

		assert(data != NULL);

		// 3. Encode data
		if(rval == NO_ERROR)
		{
			switch(this->Type)
			{
			case MSG_TYPE_FW_PING:
				s = _encode_Ping(&this->Param);
				break;
//			case MSG_TYPE_CHECKUP:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_CHECKUP;
//				break;
//			case MSG_TYPE_GET_DISTANCE:
//				frame->Data[frame->Length++]=	MSG_NB_DATA_ENCODE_GET_DISTANCE;
//				break;

			case MSG_TYPE_PROP_GET_POSITION:
				s = _encode_Prop_GetPosition(&this->Param);
				break;
			case MSG_TYPE_PROP_GET_STATUS:
				s = _encode_Prop_GetStatus(&this->Param);
				break;
			case MSG_TYPE_SERVO_GET_STATUS:
				s = _encode_Servo_GetStatus(&this->Param);
				break;
			case MSG_TYPE_MPP_GET_STATUS:
				s = _encode_MPP_GetStatus(&this->Param);
				break;
			case MSG_TYPE_GPIO_READ_ALL_DIG_IN:
				s = _encode_GPIO_ReadAllDigInput(&this->Param);
				break;
			case MSG_TYPE_GPIO_READ_ALL_AN_IN:
				s = _encode_GPIO_ReadAllAnInput(&this->Param);
				break;
			case MSG_TYPE_GPIO_GET_OUTPUT:
				s = _encode_GPIO_GetOutput(&this->Param);
				break;
			case MSG_TYPE_GPIO_GET_STATUS:
				s = _encode_GPIO_GetStatus(&this->Param);
				break;
			case MSG_TYPE_BAR_GET_STATUS:
				s = _encode_BAR_GetStatus(&this->Param);
				break;

			// Other commands doesn't need an answer
			default:
				rval = MSG_ERROR_NO_ANSWER_NEEDED;
				break;
			}
		}

		if(rval == NO_ERROR)
		{
			memcpy(data, s.c_str(), s.length());
		}

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
