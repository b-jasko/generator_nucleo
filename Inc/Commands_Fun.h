/*
 * Commands.h
 *
 *  Created on: Dec 3, 2019
 *      Author: Wojciech Grzeliński
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "main.h"
#include "Errors.h"

#define MAX_MESSAGE_LENGTH						(128u)
#define MAX_NUMBER_OF_SEPARATORS 				(10u)

#define MAX_NUMBER_OF_PIECES					(MAX_NUMBER_OF_SEPARATORS + 1u)
#define MAX_LENGTH_OF_A_PIECE					(20u)

typedef enum {
	SIN,
	TRIANGLE,
	SQUARE
} signal_type_T;

extern uint8_t setSignalFlag;
extern signal_type_T setSignalType;

extern uint16_t Set_Output_Voltage_VOUT(const char *value);

uint16_t Source_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_FM_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_FM_INT_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_FM_INT_WAVE_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);


uint16_t Measure_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Set_Output_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Set_Voltage_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);


#endif /* COMMANDS_H_ */
