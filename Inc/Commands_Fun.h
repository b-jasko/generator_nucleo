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

typedef enum {
	SAWTOOTH,
	GAUSSIAN
} dac_signal_type_T;

typedef enum {
	AD9833,
	CA_CH1,
	PWM
} multiplexer_select_T;

extern uint8_t setSignalFlag;
extern signal_type_T setSignalType;
extern uint32_t setSignalFreq;

extern uint8_t PWM_Duty;
extern uint32_t PWM_Freq;

extern uint16_t dacStep;
extern dac_signal_type_T dacSignalSelect;

extern uint16_t gauss_value;
extern uint16_t gauss_mean;
extern uint16_t gauss_std_dev;

extern multiplexer_select_T multiplexerChannelSelect;

extern uint16_t Set_Output_Voltage_VOUT(const char *value);

uint16_t Source_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_INT_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_INT_WAVE_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_INT_FREQ_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_INT_PWMDUTY_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Source_INT_GAUSS_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);


uint16_t Measure_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Set_Output_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
uint16_t Set_Voltage_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);


#endif /* COMMANDS_H_ */
