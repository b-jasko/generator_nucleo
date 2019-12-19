/*
 * Commands.c
 *
 *  Created on: Dec 02, 2019
 *      Author: Wojciech Grzeliński
 */

#include <Commands_Fun.h>

uint16_t Source_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
	ERROR_CODE errorCode = 0x00;

	switch (CommandNumbers[1])
	{
		case 0x53:		//FM
		{
			errorCode = Source_FM_Command(tableWithMessagePieces, numberOfPieces, CommandNumbers, SCPI_buffer);
			break;
		}
		default:
		{
			errorCode = COMMAND_NOT_RECOGNIZED;
			break;
		}
	}

	return errorCode;
}

uint16_t Source_FM_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
	ERROR_CODE errorCode = 0x00;

	switch (CommandNumbers[2])
	{
		case 0x7D:		//INT, INTernal
		{
			errorCode = Source_FM_INT_Command(tableWithMessagePieces, numberOfPieces, CommandNumbers, SCPI_buffer);
			break;
		}
		default:
		{
			errorCode = COMMAND_NOT_RECOGNIZED;
			break;
		}
	}

	return errorCode;
}

uint16_t Source_FM_INT_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
	ERROR_CODE errorCode = 0x00;

	switch (CommandNumbers[3])
	{
		case 0xA0:		//WAVE
		{
			errorCode = Source_FM_INT_WAVE_Command(tableWithMessagePieces, numberOfPieces, CommandNumbers, SCPI_buffer);
			break;
		}
		default:
		{
			errorCode = COMMAND_NOT_RECOGNIZED;
			break;
		}
	}

	return errorCode;
}

uint16_t Source_FM_INT_WAVE_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
	ERROR_CODE errorCode = 0x00;

	switch (CommandNumbers[4])
	{
		case 0x98:		//SINE
		{
			setSignalType = SIN;
			setSignalFlag = 1;
			break;
		}
		case 0x99:		//TRIANGLE
		{
			setSignalType = TRIANGLE;
			setSignalFlag = 1;
			break;
		}
		case 0x9A:		//SQUARE
		{
			setSignalType = SQUARE;
			setSignalFlag = 1;
			break;
		}

		// case 0x123 	//FREQ
		// {
		// 	if CommandNumbers[5] == IS_A_NUMERIC_VALUE //0x00
		// 	{
		// 		set_freq(tableWithMessagePieces[5]);
		// 	}
		// }

		default:
		{
			errorCode = COMMAND_NOT_RECOGNIZED;
			break;
		}
	}

	return errorCode;
}


uint16_t Measure_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
//	ERROR_CODE errorCode = 0x00;
//
//	switch (CommandNumbers[1])
//	{
//		case 0x50:		//CURR?, CURRENT?
//		{
//			if (3 == *numberOfPieces)
//			{
//				if (0x87 == CommandNumbers[2])
//				{
//					errorCode = Get_Current_Reading(SCPI_buffer);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//			}
//			else
//			{
//				if (2 == *numberOfPieces)
//				{
//					errorCode = Get_Current_Reading(SCPI_buffer);
//				}
//				else
//					errorCode = INVALID_PARAMETER;
//			}
//			break;
//		}
//		case 0x51:		//VOLT?, VOLTAGE?
//		{
//			if (3 == *numberOfPieces)
//			{
//				errorCode = Get_Voltage_Reading(SCPI_buffer, CommandNumbers[2]);
//			}
//			else
//			{
//				if (*numberOfPieces > 3)
//				{
//					errorCode = TOO_MANY_PARAMETERS;
//				}
//				else
//				{
//					errorCode = NO_PARAMETER;
//				}
//			}
//			break;
//		}
//		default:
//		{
//			errorCode = COMMAND_NOT_RECOGNIZED;
//			break;
//		}
//	}
//
//	return errorCode;
}


uint16_t Set_Output_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
//	ERROR_CODE errorCode = 0x00;
//
//	switch (CommandNumbers[2])
//	{
//		case 0x80:		//3,3V output relay
//		{
//			if (3 == *numberOfPieces)
//			{
//				if (0x7D == CommandNumbers[2])	//ON
//				{
//					HAL_GPIO_WritePin(RELAY_3V3_GPIO_Port, RELAY_3V3_Pin, GPIO_PIN_SET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//
//				if (0x7C == CommandNumbers[2])	//OFF
//				{
//					HAL_GPIO_WritePin(RELAY_3V3_GPIO_Port, RELAY_3V3_Pin, GPIO_PIN_RESET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//			}
//			else
//			{
//					errorCode = INVALID_PARAMETER;
//			}
//			break;
//		}
//		case 0x81:		//5V output relay
//		{
//			if (3 == *numberOfPieces)
//			{
//				if (0x7D == CommandNumbers[2])	//ON
//				{
//					HAL_GPIO_WritePin(RELAY_5V_GPIO_Port, RELAY_5V_Pin, GPIO_PIN_SET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//
//				if (0x7C == CommandNumbers[2])	//OFF
//				{
//					HAL_GPIO_WritePin(RELAY_5V_GPIO_Port, RELAY_5V_Pin, GPIO_PIN_RESET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//			}
//			else
//			{
//					errorCode = INVALID_PARAMETER;
//			}
//			break;
//		}
//		case 0x82:		//12V output relay
//		{
//			if (3 == *numberOfPieces)
//			{
//				if (0x7D == CommandNumbers[2])	//ON
//				{
//					HAL_GPIO_WritePin(RELAY_12V_GPIO_Port, RELAY_12V_Pin, GPIO_PIN_SET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//
//				if (0x7C == CommandNumbers[2])	//OFF
//				{
//					HAL_GPIO_WritePin(RELAY_12V_GPIO_Port, RELAY_12V_Pin, GPIO_PIN_RESET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//			}
//			else
//			{
//					errorCode = INVALID_PARAMETER;
//			}
//			break;
//		}
//		case 0x83:		//VOUT output relay
//		{
//			if (3 == *numberOfPieces)
//			{
//				if (0x7D == CommandNumbers[2])	//ON
//				{
//					HAL_GPIO_WritePin(RELAY_VOUT_GPIO_Port, RELAY_VOUT_Pin, GPIO_PIN_SET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//
//				if (0x7C == CommandNumbers[2])	//OFF
//				{
//					HAL_GPIO_WritePin(RELAY_VOUT_GPIO_Port, RELAY_VOUT_Pin, GPIO_PIN_RESET);
//				}
//				else
//				{
//					errorCode = INVALID_PARAMETER;
//				}
//			}
//			else
//			{
//					errorCode = INVALID_PARAMETER;
//			}
//			break;
//		}
//		default:
//		{
//			errorCode = COMMAND_NOT_RECOGNIZED;
//			break;
//		}
//	}
//
//	return errorCode;
}


uint16_t Set_Voltage_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
//	ERROR_CODE errorCode = 0x00;
////	IS_A_NUMERIC_VALUE
//	switch (CommandNumbers[2])
//	{
//		case 0x8A:		//Measure_VOUT
//		{
//			errorCode = Set_Output_Voltage_VOUT(tableWithMessagePieces[3]);
//			break;
//		}
//		default:
//		{
//			errorCode = COMMAND_NOT_RECOGNIZED;
//			break;
//		}
//	}
//
//	return errorCode;
}
//
uint16_t Set_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
//	ERROR_CODE errorCode = 0x00;
//
//	switch (CommandNumbers[1])
//	{
//		case 0x26:		//OUTP, OUTPUT
//		{
//			errorCode = Set_Output_Command(tableWithMessagePieces, numberOfPieces, CommandNumbers, SCPI_buffer);
//			break;
//		}
//		case 0x44:		//VOLT, VOLTAGE
//		{
//			errorCode = Set_Voltage_Command(tableWithMessagePieces, numberOfPieces, CommandNumbers, SCPI_buffer);
//			break;
//		}
//		default:
//		{
//			errorCode = COMMAND_NOT_RECOGNIZED;
//			break;
//		}
//	}
//
//	return errorCode;
}








