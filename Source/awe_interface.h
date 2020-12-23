/*-----------------------------------------------------------------------------
File Name   :
Author      : Sam Soundar
Date        : yyyy-mm-dd
Description :
------------------------------------------------------------------------------*/
#ifndef AWE_INTERFACE_H
#define AWE_INTERFACE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Public Macros/Constants/Structures ----------------------------------------*/
/* Define number of input and output channels */
#define AWE_INTERFACE_NUM_INPUT_CHANNELS    (16)
#define AWE_INTERFACE_NUM_OUTPUT_CHANNELS   (2)
#define AWE_INTERFACE_AUDIO_BLOCK_SIZE      (32)


/* Public Variables ----------------------------------------------------------*/


/* Public Function Declarations ----------------------------------------------*/
void AweInterface_Init(void);
void AweInterface_MainLoopProcess(void);
void AweInterface_HandleInputSamplesBlock(int32_t *inputSamplesBlock);


#endif
