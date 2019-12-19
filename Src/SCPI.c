/*
 * SCPI.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Wojciech Grzeliński
 */


#include "SCPI.h"


#define MAX_COMMAND_NAME_LENGTH		(25u)
#define IS_A_NUMERIC_VALUE			(0x00)

typedef struct {
	char name[MAX_COMMAND_NAME_LENGTH];
	uint16_t commandNumber;
}COMMANDS;


COMMANDS MandatedCommands[] = {
		{"*CLS",				0x01},		/* Clear Status - Device Status */
		{"*ESE",				0x02},		/* Event Status - Device Status */
		{"*ESE?",				0x03},		/* Event Status ??? */
		{"*ESR?",				0x04},		/* Event Status Register - Device Status */
		{"*IDN?",				0x05},		/* Identification - System Interface */
		{"*OPC",				0x06},		/* Operation Complete - Device Status */
		{"*OPC?",				0x07},		/* Operation Complete ??? */
		{"*PSC",				0x08},		/* Option Identification Query */
		{"*PSC?",				0x09},		/* Power-on Status Clear - Device Initialization */
		{"*RCL",				0x0A},		/* Recall - Device Status */
		{"*RST",				0x0B},		/* Reset - Device State */
		{"*SAV",				0x0C},		/* SAVE - Device State */
		{"*SRE",				0x0D},		/* Service Request Enable - Device Interface */
		{"*SRE?",				0x0E},		/* Service Request Enable ??? */
		{"*STB?",				0x0F},		/* Status Byte - Device Status */
		{"*TRG",				0x10},		/* Trigger - Device Trigger */
		{"*TST",				0x11},		/* Test - Device Test */
		{"*WAI",				0x12}		/* Wait to Continue - Device Status */
};

COMMANDS Commands_1[] = {
		{"ABOR",				0x20},
		{"ABORT",				0x20},
		{"CAL",					0x21},
		{"CALIBRATION",			0x21},
		{"SOUR",				0x22},
		{"SOURCE",				0x22},
		{"DISP",				0x23},
		{"DISPLAY",				0x23},
		{"INIT",				0x24},
		{"INITIATE",			0x24},
		{"MEAS",				0x25},
		{"MEASURE",				0x25},
		{"OUTP",				0x26},
		{"OUTPUT",				0x26},
		{"STAT",				0x27},
		{"STATUS",				0x27},
		{"SYST",				0x28},
		{"SYSTEM",				0x28},
		{"TRIG",				0x29},
		{"TRIGGER",				0x29},
		{"SET",					0x2A}
};

COMMANDS Commands_2[] = {
		{"CURR",				0x40},
		{"CURRENT",				0x40},
		{"DIG",					0x41},
		{"DIGITAL",				0x41},
		{"WIND",				0x42},
		{"WINDOW",				0x42},
		{"CONT",				0x43},
		{"CONTINUOUS",			0x43},
		{"VOLT",				0x44},
		{"VOLTAGE",				0x44},
		{"STAT",				0x45},
		{"STATE",				0x45},
		{"PROT",				0x46},
		{"PROTECTION",			0x46},
		{"REL",					0x47},
		{"RELAY",				0x47},
		{"OPER",				0x48},
		{"OPERATION",			0x48},
		{"PRES",				0x49},
		{"PRESET",				0x49},
		{"QUES",				0x4A},
		{"QUESTIONABLE",		0x4A},
		{"ERR",					0x4B},
		{"ERROR",				0x4B},
		{"LANG",				0x4C},
		{"LANGUAGE",			0x4C},
		{"VERS",				0x4D},
		{"VERSION",				0x4D},
		{"IMM",					0x4E},
		{"IMMEDIATE",			0x4E},
		{"SOUR",				0x4F},
		{"SOURCE",				0x4F},
		{"CURR?",				0x50},
		{"CURRENT?",			0x50},
		{"VOLT?",				0x51},
		{"VOLTAGE?",			0x52},
		{"FM",					0x53}
};

COMMANDS Commands_3[] = {
		{"LEV",						0x60},
		{"LEVEL",					0x60},
		{"PROT",					0x61},
		{"PROTOCOL",				0x61},
		{"DATA",					0x62},
		{"MODE",					0x63},
		{"STAT",					0x64},
		{"STATE",					0x64},
		{"TEXT",					0x65},
		{"DC",						0x66},
		{"CLE",						0x67},
		{"CLEAR",					0x67},
		{"DEL",						0x68},
		{"DELETE",					0x68},
		{"POL",						0x69},
		{"POLAR",					0x69},
		{"COND",					0x6A},
		{"CONDITION",				0x6A},
		{"ENAB",					0x6B},
		{"ENABLE",					0x6B},
		{"EVEN",					0x6C},
		{"EVENT",					0x6C},
		{"NTR",						0x6D},
		{"NTRANSITION",				0x6D},
		{"PTR",						0x6E},
		{"PTRANSITION",				0x6E},
		{"MIN",						0x7E},
		{"MAX",						0x7F},
		{"ON",						0x7D},
		{"OFF",						0x7C},
		{"INT",						0x7D},
		{"INTERNAL",				0x7D}
};

COMMANDS Commands_4[] = {
		{"WAVE",						0xA0}
};

COMMANDS Ports[] = {
		{"PC13",						0x80},
		{"GPIO_3V3",					0x80},
		{"PC14",						0x81},
		{"GPIO_5V",						0x81},
		{"PC15",						0x82},
		{"GPIO_12V",					0x82},
		{"PH0",							0x83},
		{"GPIO_VOUT",					0x83},
		{"PC0",							0x84},
		{"ANI_3V3",						0x84},
		{"PC1",							0x85},
		{"ANI_5V",						0x85},
		{"PC2",							0x86},
		{"ANI_12V",						0x86},
		{"PA0",							0x87},
		{"Current_Measurement",			0x87},
		{"PA3",							0x88},
		{"OPAMP1_VOUT",					0x88},
		{"PA5",							0x89},
		{"PA5_NTC",						0x89},
		{"PA6",							0x8A},
		{"MEASURE_VOUT",				0x8A},
		{"PC4",							0x8B},
		{"PC4_NTC",						0x8B},
		{"PC5",							0x8C},
		{"PC5_NTC",						0x8C},
		{"PB0",							0x8D},
		{"OPAMP2_VOUT",					0x8D},
		{"PB1",							0x8E},
		{"PB1_NTC",						0x8E},
		{"PB2",							0x8F},
		{"RUN",							0x8F},
		{"PB12",						0x90},
		{"BUTTON_3V3",					0x90},
		{"PB13",						0x91},
		{"BUTTON_5V",					0x91},
		{"PB14",						0x92},
		{"BUTTON_12V",					0x92},
		{"PB15",						0x93},
		{"BUTTON_VOUT",					0x93},
		{"PA10",						0x94},
		{"PGOOD",						0x94},
		{"PB5",							0x95},
		{"ENC_S",						0x95},
		{"PB6",							0x96},
		{"ENC_B",						0x96},
		{"PB7",							0x97},
		{"ENC_A",						0x97},
		{"SINE",						0x98},
		{"TRI",							0x99},
		{"TRIANGLE",					0x99},
		{"SQU",							0x9A},
		{"SQUARE",						0x9A},
		{"GAU",							0x9B},
		{"GAUSIAN",						0x9B},
		{"RUP",							0x9C},
		{"PWM",							0x9D},
};


const uint16_t MandatedCommands_LENGTH = sizeof(MandatedCommands) / sizeof(MandatedCommands[0]);
const uint16_t Commands_1_LENGTH = sizeof(Commands_1) / sizeof(Commands_1[0]);
const uint16_t Commands_2_LENGTH = sizeof(Commands_2) / sizeof(Commands_2[0]);
const uint16_t Commands_3_LENGTH = sizeof(Commands_3) / sizeof(Commands_3[0]);
const uint16_t Commands_4_LENGTH = sizeof(Commands_4) / sizeof(Commands_4[0]);
const uint16_t Ports_LENGTH = sizeof(Ports) / sizeof(Ports[0]);



static uint16_t Get_Message_Length(const char *MESSAGE)
{
	ERROR_CODE errorCode = 0x00;
	messageLength = strlen(MESSAGE);

	if(0 == messageLength)
		errorCode = MESSAGE_HAS_LENGTH_ZERO;

	return errorCode;
}


static uint16_t Message_To_Upper_Letters(char* message)
{
	ERROR_CODE errorCode = 0x00;
	uint16_t i = 0;

	for (i = 0; i < messageLength; ++i)
	{
		if (message[i] >= 97 && message[i] <= 122)
		{
			message[i] = message[i] - 32;
		}
	}

	return errorCode;
}


static uint16_t Find_Separators(const char *MESSAGE, uint16_t *separatorIndexesTable, uint8_t *howManySeparatorsWereFound)
{
	ERROR_CODE errorCode = 0x00;
	uint16_t i = 0;
	uint16_t j = 0;

	for (i = 0; i < messageLength; ++i)
	{
		if (':' == MESSAGE[i] || ' ' == MESSAGE[i] || ',' == MESSAGE[i])
		{
			separatorIndexesTable[j] = i;
			++*howManySeparatorsWereFound;

			if (*howManySeparatorsWereFound > MAX_NUMBER_OF_SEPARATORS)
			{
				errorCode = TOO_MANY_SEPARATORS;
				break;
			}

			if (0 != j)	//if it is not the first colon found
			{
				if (separatorIndexesTable[j] - separatorIndexesTable[j-1] <= 1)	//check if there is something between colons
				{
					errorCode = MESSAGE_NOT_CORRECT;
					break;
				}
			}
			++j;	// move to next position in separatorIndexesTable to remember next colons
		}
	}

	return errorCode;
}


static uint16_t Cut_Message_Into_Pieces(const char *MESSAGE, char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], uint8_t *numberOfPieces)
{
	ERROR_CODE errorCode = 0x00;
	uint16_t iteratorInMessage = 0;
	uint16_t jteratorInseparatorIndexesTable = 0;
	uint16_t lastPosition = 0;
	uint16_t pieceIterator = 0;
	uint8_t howManySeparatorsWereFound = 0;
	uint16_t separatorIndexesTable[MAX_NUMBER_OF_SEPARATORS] = {0};

	/* We look for separators between pieces of messages. The separators are: ":", " " and ",". */
	errorCode = Find_Separators(MESSAGE, separatorIndexesTable, &howManySeparatorsWereFound);

	*numberOfPieces = howManySeparatorsWereFound + 1;

	if (0 != howManySeparatorsWereFound && 0 == errorCode)
	{
		for (jteratorInseparatorIndexesTable = 0; jteratorInseparatorIndexesTable < howManySeparatorsWereFound; ++jteratorInseparatorIndexesTable)
		{
			if (errorCode != 0)
				break;

			for (iteratorInMessage = lastPosition; iteratorInMessage < separatorIndexesTable[jteratorInseparatorIndexesTable]; ++iteratorInMessage)
			{
				tableWithMessagePieces[jteratorInseparatorIndexesTable][pieceIterator] = MESSAGE[iteratorInMessage];
				++pieceIterator;

				if (pieceIterator >= MAX_LENGTH_OF_A_PIECE-1)
				{
					errorCode = PIECE_TOO_LONG;
					break;
				}
			}

			lastPosition = separatorIndexesTable[jteratorInseparatorIndexesTable]+1;
			pieceIterator = 0;

			if (lastPosition >= messageLength)
			{
				errorCode = SEPARATOR_AT_THE_END_OF_MESSAGE;
				break;
			}
		}

		if (0 == errorCode)
		{
			for (iteratorInMessage = lastPosition; iteratorInMessage < messageLength; ++iteratorInMessage)
			{
				tableWithMessagePieces[jteratorInseparatorIndexesTable][pieceIterator] = MESSAGE[iteratorInMessage];
				++pieceIterator;

				if (pieceIterator >= MAX_LENGTH_OF_A_PIECE-1)
				{
					errorCode = PIECE_TOO_LONG;
					break;
				}
			}
		}
	}
	else
	{
		strcpy(tableWithMessagePieces[0], MESSAGE);
	}

	return errorCode;
}


static uint16_t Recognize_Piece_Of_Message(char *Piece, uint16_t *foundCommand)
{
	ERROR_CODE errorCode = 0x00;
	uint16_t i = 0;
	uint8_t FLAG_match = 0;
	uint8_t FLAG_isDigit = 0;

	if ('*' == Piece[0])
	{
		for (i = 0; i < MandatedCommands_LENGTH; ++i)
		{
			FLAG_match = (strcmp(Piece, MandatedCommands[i].name) ? 0 : 1);

			if (1 == FLAG_match)
			{
				*foundCommand = MandatedCommands[i].commandNumber;
				break;
			}
		}
	}
	else
	{
		if (0 == FLAG_match)
		{
			for (i = 0; i < Commands_1_LENGTH; ++i)
			{
				FLAG_match = (strcmp(Piece, Commands_1[i].name) ? 0 : 1);

				if (1 == FLAG_match)
				{
					*foundCommand = Commands_1[i].commandNumber;
					break;
				}
			}
		}

		if (0 == FLAG_match)
		{
			for (i = 0; i < Commands_2_LENGTH; ++i)
			{
				FLAG_match = (strcmp(Piece, Commands_2[i].name) ? 0 : 1);

				if (1 == FLAG_match)
				{
					*foundCommand = Commands_2[i].commandNumber;
					break;
				}
			}
		}

		if (0 == FLAG_match)
		{
			for (i = 0; i < Commands_3_LENGTH; ++i)
			{
				FLAG_match = (strcmp(Piece, Commands_3[i].name) ? 0 : 1);

				if (1 == FLAG_match)
				{
					*foundCommand = Commands_3[i].commandNumber;
					break;
				}
			}
		}

		if (0 == FLAG_match)
		{
			for (i = 0; i < Commands_4_LENGTH; ++i)
			{
				FLAG_match = (strcmp(Piece, Commands_4[i].name) ? 0 : 1);

				if (1 == FLAG_match)
				{
					*foundCommand = Commands_4[i].commandNumber;
					break;
				}
			}
		}

		if (0 == FLAG_match)
		{
			for (i = 0; i < Ports_LENGTH; ++i)
			{
				FLAG_match = (strcmp(Piece, Ports[i].name) ? 0 : 1);

				if (1 == FLAG_match)
				{
					*foundCommand = Ports[i].commandNumber;
					break;
				}
			}
		}

		if (0 == FLAG_match)
		{
			for (i = 0; i < strlen(Piece); ++i)
			{
				if ((isdigit(Piece[i])) || ('e' == Piece[i]) || ('E' == Piece[i]) || ('.' == Piece[i]))
				{
					FLAG_isDigit = 1;
				}
				else
				{
					FLAG_isDigit = 0;
					break;
				}
			}

			if (1 == FLAG_isDigit)
			{
				FLAG_match = 1;
				*foundCommand = IS_A_NUMERIC_VALUE;
			}
		}
	}

	if (0 == FLAG_match)
		errorCode = COMMAND_NOT_RECOGNIZED;

	return errorCode;
}


static uint16_t Recognize_Message(char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, uint16_t *CommandNumbers)
{
	ERROR_CODE errorCode = 0x00;
	uint16_t i = 0;
	uint16_t command = 0;

	for (i = 0; i < *numberOfPieces; ++i)
	{
		errorCode = Recognize_Piece_Of_Message(tableWithMessagePieces[i], &command);
		CommandNumbers[i] = command;

		if (0 != errorCode)
			break;
	}

	return errorCode;
}


static uint16_t Do_The_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer)
{
	ERROR_CODE errorCode = 0x00;
	uint8_t command_found_FLAG = 0;

	if (0 == errorCode)
	{
		if ((1 == *numberOfPieces) && (0 == command_found_FLAG))
		{
			switch(CommandNumbers[0])
			{
				case 0x01:		//*CLS
				{
					//TODO
		//			errorCode = Clear_Status_Data_Structures();
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "CLS Done!");
					command_found_FLAG = 1;
					break;
				}
				case 0x02:		//*ESE
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*ESE\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x03:		//*ESE?
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*ESE?\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x04:		//*ESR?
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*ESR?\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x05:		//*IDN?
				{
					sprintf(SCPI_buffer, "Manufacturer:	Bartosz Jasko\n\r"
							"E-mail: bjasko97@gmail.com\n\r"
							"Version: 1.1\n\r"
							"Date: 21.11.2019\n\r"
							"This device is mostly compatible with SCPI protocol. Due to limited functionality there was no need to implement all SCPI commands and queries.\n\r"
							"In case of any bugs and mistakes, please contact the author of this code and-or device via E-mail.\n\r");
					command_found_FLAG = 1;
					break;
				}
				case 0x06:		//*OPC
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*OPC\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x07:		//*OPC?
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*OPC?\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x08:		//*PSC
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*PSC\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x09:		//*PSC?
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*PSC?\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x0A:		//*RCL
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*RCL\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x0B:		//*RST
				{
					//TODO
					//	errorCode = Reset_all_settings_to_default();
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "RST Done!");
					command_found_FLAG = 1;
					break;
				}
				case 0x0C:		//*SAV
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*SAV\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x0D:		//*SRE
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*SRE\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x0E:		//*SRE?
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*SRE?\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x0F:		//*STB?
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*STB?\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x10:		//*TRG
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*TRG\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				case 0x11:		//*TST
				{
					//TODO
					//	errorCode = Test_Device(SCPI_buffer);
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "RST Done!");
					command_found_FLAG = 1;
					break;
				}
				case 0x12:		//*WAI
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Command \"*WAI\" is not applicable for this device!");
					command_found_FLAG = 1;
					break;
				}
				default:
				{
					command_found_FLAG = 0;
					break;
				}
			}
		}

		if (0 == command_found_FLAG)
		{
			switch(CommandNumbers[0])
			{
				case 0x20:		//ABOR, ABORT
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x21:		//CAL, CALIBRATION
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x22:		//SOUR, SOURCE
				{
					errorCode = Source_Command(tableWithMessagePieces, numberOfPieces, CommandNumbers, SCPI_buffer);
					if (SEND_DEBUG_ANSWERS)
					command_found_FLAG = 1;
					break;
				}
				case 0x23:		//DISP, DISPLAY
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x24:		//INIT, INITIATE
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x25:		//MEAS, MEASURE
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x26:		//OUTP, OUTPUT
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x27:		//STAT, STATUS
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x28:		//SYST, SYSTEM
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x29:		//TRIG, TRIGGER
				{
					if (SEND_DEBUG_ANSWERS)
						sprintf(SCPI_buffer, "Not applicable");
					command_found_FLAG = 1;
					break;
				}
				case 0x2A:		//SET
				{
					errorCode = Set_Command(tableWithMessagePieces, numberOfPieces, CommandNumbers, SCPI_buffer);
					command_found_FLAG = 1;
					break;
				}
				default:
				{
					errorCode = COMMAND_NOT_RECOGNIZED;
					command_found_FLAG = 0;
					break;
				}
			}
		}
	}

	return errorCode;
}


uint16_t Process_SCPI_Message(const char *MESSAGE, char *SCPI_buffer)
{
	ERROR_CODE errorCode = 0x00;
	char messageCopy[MAX_MESSAGE_LENGTH] = "";
	char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE] = {""};
	uint8_t numberOfPieces = 0;
	uint16_t CommandNumbers[MAX_NUMBER_OF_PIECES] = {};

	strcpy(messageCopy, MESSAGE);

	if (0 == errorCode)
		/* Sets value of messageLength */
		errorCode = Get_Message_Length(messageCopy);

	if (0 == errorCode)
		/* Makes whole message to upper letters (because SCPI standard says that we should be case insensitive!) */
		errorCode = Message_To_Upper_Letters(messageCopy);

	if (0 == errorCode)
		/* We cut the received message into pieces (commands). */
		errorCode = Cut_Message_Into_Pieces(messageCopy, tableWithMessagePieces, &numberOfPieces);

	if (0 == errorCode)
		/* After we get message in pieces we can start recognizing the pieces. This function finds the pieces
		 * in the lists at the top and sets command numbers for them (as there are multiple combinations of
		 * names for the same command <<ex. "volt?" and "voltage?">> and there is just one number for them all.
		 * It allows then to easily operate with switch-case statements, as we can add more recognizable names
		 * with no hassle. */
		errorCode = Recognize_Message(tableWithMessagePieces, &numberOfPieces, CommandNumbers);

	if (0 == errorCode)
		/* This function is responsible for doing the recognized commands. They are implemented differently
		 * so please be careful while editing. Some of the functions are declared as extern and defined in
		 * Commands_Fun.c and main.c files. */
		errorCode = Do_The_Command(tableWithMessagePieces, &numberOfPieces, CommandNumbers, SCPI_buffer);

	return errorCode;
}








