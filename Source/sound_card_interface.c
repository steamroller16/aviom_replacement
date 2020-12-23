/*-----------------------------------------------------------------------------
File Name   : sound_card_interface.c
Author      : Sam Soundar
Date        : 2020-12-22
Description : Interface to I2S sound card
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "awe_interface.h"


/* Local Macros/Constants/Structures -----------------------------------------*/


/* Public Global Variables ---------------------------------------------------*/


/* Private Global Variables --------------------------------------------------*/
static int32_t AudioSamplesBuffer[AWE_INTERFACE_AUDIO_BLOCK_SIZE][AWE_INTERFACE_NUM_OUTPUT_CHANNELS];


/* Private Function Prototypes -----------------------------------------------*/


/* Function Implementations --------------------------------------------------*/
int32_t * SoundCardInterface_GetAudioSamplesBuffer(void)
{
    return &AudioSamplesBuffer[0][0];
}

void SoundCardInterface_NotifySamplesBufferFull(int32_t *samplesBuffer)
{
    return;
}
