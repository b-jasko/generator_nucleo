/*
 * Errors.h
 *
 *  Created on: Nov 10, 2019
 *      Author: Wojciech Grzeliński
 */

#ifndef ERRORS_H_
#define ERRORS_H_

#include <stdio.h>


typedef enum {
	NO_ERROR =									(0x00),			//0

	RECEIVED_MESSAGE_IS_TOO_LONG = 				(0x01),			//1
	MESSAGE_TO_SEND_IS_TOO_LONG =				(0x02),			//2

	MESSAGE_NOT_CORRECT =						(0x20u),		//32
	MESSAGE_HAS_LENGTH_ZERO = 					(0x21u),		//33
	TOO_MANY_SEPARATORS =						(0x22u),		//34
	SEPARATOR_AT_THE_END_OF_MESSAGE =			(0x23u),		//35
	PIECE_TOO_LONG =							(0x24u),		//36
	COMMAND_NOT_RECOGNIZED = 					(0x25u),		//37
	INVALID_PARAMETER = 						(0x26u),		//38
	NO_PARAMETER = 								(0x27u),		//39
	TOO_MANY_PARAMETERS = 						(0x28u)			//40
}ERROR_CODE;

char* Return_Error_Description(const uint16_t errorCode);


#endif /* ERRORS_H_ */
