/*-----------------------------------------------------------------------------
File Name   : aviom_receiver.h
Author      : Sam Soundar
Date        : 2020-12-22
Description : Handles reception of audio packets over ethernet from aviom system
------------------------------------------------------------------------------*/
#ifndef AVIOM_RECEIVER_H
#define AVIOM_RECEIVER_H

/* Includes ------------------------------------------------------------------*/


/* Public Macros/Constants/Structures ----------------------------------------*/


/* Public Variables ----------------------------------------------------------*/


/* Public Function Declarations ----------------------------------------------*/
void AviomReceiver_Init(void);
void AviomReceiver_MainLoopProcess(void);


#endif
