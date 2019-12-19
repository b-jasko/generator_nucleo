/*
 * SCPI.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Wojciech Grzeliński
 */

#ifndef SCPI_H_
#define SCPI_H_

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "Errors.h"
#include "Commands_Fun.h"

/* Constants */
#define MAX_MESSAGE_LENGTH						(128u)
#define MAX_NUMBER_OF_SEPARATORS 				(10u)
#define MAX_NUMBER_OF_PIECES					(MAX_NUMBER_OF_SEPARATORS + 1u)
#define MAX_LENGTH_OF_A_PIECE					(20u)

#define SEND_DEBUG_ANSWERS						(1u)


/* Value set by "Get_Message_Length" function */
static uint16_t messageLength = 0;

extern uint16_t Get_Current_Reading(char *message_Buffer);
extern uint16_t Get_Voltage_Reading(char *message_Buffer, const uint16_t channel_number);

extern uint16_t Measure_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
extern uint16_t Set_Output_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);
extern uint16_t Set_Voltage_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);


static uint16_t Get_Message_Length(const char *MESSAGE);
static uint16_t Message_To_Upper_Letters(char* message);
static uint16_t Find_Separators(const char *MESSAGE, uint16_t *separatorIndexesTable, uint8_t *howManyColonsWereFound);
static uint16_t Cut_Message_Into_Pieces(const char *MESSAGE, char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], uint8_t *numberOfPieces);
static uint16_t Recognize_Piece_Of_Message(char *Piece, uint16_t *foundCommand);
static uint16_t Recognize_Message(char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, uint16_t *CommandNumbers);
static uint16_t Do_The_Command(const char tableWithMessagePieces[MAX_NUMBER_OF_PIECES][MAX_LENGTH_OF_A_PIECE], const uint8_t *numberOfPieces, const uint16_t *CommandNumbers, char *SCPI_buffer);

uint16_t Process_SCPI_Message(const char *MESSAGE, char *SCPI_buffer);

#endif /* SCPI_H_ */
