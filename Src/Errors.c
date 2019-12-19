/*
 * Errors.c
 *
 *  Created on: Nov 11, 2019
 *      Author: Wojciech Grzeliński
 */


#include "Errors.h"


char* Return_Error_Description(const uint16_t errorCode)
{
	char *description;

	switch(errorCode)
	{
		case 0x00:
		{
			description = "No error was detected.";
			break;
		}
		case 0x01:
		{
			description = "Received message is too long - shorten the message or increase the receive buffer size!";
			break;
		}
		case 0x02:
		{
			description = "Message prepared to send is too long - increase the send buffer size!";
			break;
		}
		case 0x20:
		{
			description = "Received message is incorrect - check spelling!";
			break;
		}
		case 0x21:
		{
			description = "Received message has length 0! Check if message is sent correctly.";
			break;
		}
		case 0x22:
		{
			description = "Received message has too many separators! Try shortening message or increase maximum allowed number of separators.";
			break;
		}
		case 0x23:
		{
			description = "Wrong end of the message! There is a separator at the end - try deleting it.";
			break;
		}
		case 0x24:
		{
			description = "A command in received message is too long! Try shortening (use abbreviation?) or increase maximum allowed command length.";
			break;
		}
		case 0x25:
		{
			description = "Received message has an unrecognized command inside! Check spelling.";
			break;
		}
		default:
		{
			description = "!!Unknown error!!";
			break;
		}
	}
}
