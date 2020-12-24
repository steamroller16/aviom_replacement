/*-----------------------------------------------------------------------------
File Name   : sound_card_interface.c
Author      : Sam Soundar
Date        : 2020-12-22
Description : Interface to I2S sound card
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>

#include "main.h"

#include "awe_interface.h"


/* Local Macros/Constants/Structures -----------------------------------------*/
typedef enum {
    BUFFER1,
    BUFFER2,
} BufferNum_t;


/* Public Global Variables ---------------------------------------------------*/
extern I2S_HandleTypeDef hi2s2;


/* Private Global Variables --------------------------------------------------*/
static int32_t SamplesBufferZeros[AWE_INTERFACE_AUDIO_BLOCK_SIZE/2][AWE_INTERFACE_NUM_OUTPUT_CHANNELS];
static int32_t SamplesBuffer1[AWE_INTERFACE_AUDIO_BLOCK_SIZE][AWE_INTERFACE_NUM_OUTPUT_CHANNELS];
static int32_t SamplesBuffer2[AWE_INTERFACE_AUDIO_BLOCK_SIZE][AWE_INTERFACE_NUM_OUTPUT_CHANNELS];

static bool SamplesBuffer1Full = false;
static bool SamplesBuffer2Full = false;

static BufferNum_t LastFilledBuffer = BUFFER2;
static BufferNum_t LastTransmittedBuffer = BUFFER2;

static bool TransmissionIsActive = false;


/* Private Function Prototypes -----------------------------------------------*/


/* Function Implementations --------------------------------------------------*/
void SoundCardInterface_Init(void)
{
    SamplesBuffer1Full = false;
    SamplesBuffer2Full = false;

    LastFilledBuffer = BUFFER2;
    LastTransmittedBuffer = BUFFER2;

    TransmissionIsActive = false;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (LastTransmittedBuffer == BUFFER1)
    {
        if (SamplesBuffer2Full)
        {
            HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)&SamplesBuffer2[0][0], AWE_INTERFACE_AUDIO_BLOCK_SIZE * AWE_INTERFACE_NUM_OUTPUT_CHANNELS);
            SamplesBuffer2Full = false;
            LastTransmittedBuffer = BUFFER2;
        }
        else
        {
            TransmissionIsActive = false;
        }
    }
    else if (LastTransmittedBuffer == BUFFER2)
    {
        if (SamplesBuffer1Full)
        {
            HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)&SamplesBuffer1[0][0], AWE_INTERFACE_AUDIO_BLOCK_SIZE * AWE_INTERFACE_NUM_OUTPUT_CHANNELS);
            SamplesBuffer1Full = false;
            LastTransmittedBuffer = BUFFER1;
        }
        else
        {
            TransmissionIsActive = false;
        }
    }
}

int32_t * SoundCardInterface_GetAudioSamplesBuffer(void)
{
    if (LastFilledBuffer == BUFFER1)
    {
        return &SamplesBuffer2[0][0];
    }
    else if (LastFilledBuffer == BUFFER2)
    {
        return &SamplesBuffer1[0][0];
    }
    else
    {
        // What!?
        return &SamplesBuffer1[0][0];
    }
}

void SoundCardInterface_NotifySamplesBufferFull(int32_t *samplesBuffer)
{
    if (samplesBuffer == &SamplesBuffer1[0][0])
    {
        uint8_t i,j;
        for (i=0; i<AWE_INTERFACE_AUDIO_BLOCK_SIZE; i++)
        {
            for (j=0; j<AWE_INTERFACE_NUM_OUTPUT_CHANNELS; j++)
            {
                SamplesBuffer1[i][j] = (((uint32_t)SamplesBuffer1[i][j]) >> 16) | (SamplesBuffer1[i][j] << 16);
            }
        }
        LastFilledBuffer = BUFFER1;
        SamplesBuffer1Full = true;
    }
    else if (samplesBuffer == &SamplesBuffer2[0][0])
    {
        uint8_t i,j;
        for (i=0; i<AWE_INTERFACE_AUDIO_BLOCK_SIZE; i++)
        {
            for (j=0; j<AWE_INTERFACE_NUM_OUTPUT_CHANNELS; j++)
            {
                SamplesBuffer2[i][j] = (((uint32_t)SamplesBuffer2[i][j]) >> 16) | (SamplesBuffer2[i][j] << 16);
            }
        }
        LastFilledBuffer = BUFFER2;
        SamplesBuffer2Full = true;
    }
    else
    {
        // What!?
    }

    if (!TransmissionIsActive)
    {
        if (LastFilledBuffer == BUFFER1)
        {
            LastTransmittedBuffer = BUFFER2;
        }
        else if (LastFilledBuffer == BUFFER2)
        {
            LastTransmittedBuffer = BUFFER1;
        }
        else
        {
            // What!?
        }

        TransmissionIsActive = true;
        HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)&SamplesBufferZeros[0][0], AWE_INTERFACE_AUDIO_BLOCK_SIZE/2 * AWE_INTERFACE_NUM_OUTPUT_CHANNELS);
    }
}
