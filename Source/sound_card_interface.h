/*-----------------------------------------------------------------------------
File Name   : sound_card_interface.h
Author      : Sam Soundar
Date        : 2020-12-22
Description : Interface to I2S sound card
------------------------------------------------------------------------------*/
#ifndef SOUND_CARD_INTERFACE_H
#define SOUND_CARD_INTERFACE_H

/* Includes ------------------------------------------------------------------*/


/* Public Macros/Constants/Structures ----------------------------------------*/


/* Public Variables ----------------------------------------------------------*/


/* Public Function Declarations ----------------------------------------------*/
int32_t * SoundCardInterface_GetAudioSamplesBuffer(void);
void SoundCardInterface_NotifySamplesBufferFull(int32_t *samplesBuffer);


#endif
