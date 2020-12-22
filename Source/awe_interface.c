/*-----------------------------------------------------------------------------
File Name   :
Author      : Sam Soundar
Date        : yyyy-mm-dd
Description :
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "AWECore.h"
#include "TargetProcessor.h"
#include "StandardDefs.h"
#include "Errors.h"
#include "ModuleList.h"


/* Local Macros/Constants/Structures -----------------------------------------*/
// Version Information
#define USER_VER 20190801

#define NUM_CORES 1
#define CORE_ID 0
#define CORE_SPEED 216e6f
#define SAMPLE_SPEED 216e6f
#define HAS_FLOAT_SUPPORT 1
#define HAS_FLASH_FILESYSTEM 1
#define HAS_GPIO 1
#define NO_INPUT_PINS 1
#define NO_OUTPUT_PINS 1
#define IS_SMP 0
#define NUM_AUDIO_THREADS 2
#define AUDIO_SAMPLE_RATE 48000.0f
#define AUDIO_BLOCK_SIZE 32
#define IS_COMPLEX 0
#define SAMPLE_SIZE_IN_BYTES 4


// Specify the size of each of the heaps on this target
#define MASTER_HEAP_SIZE		(1024*5)
#define FASTB_HEAP_SIZE			(1024*5)
#define SLOW_HEAP_SIZE			(1024*5)


/* Define the rest of the target for the instance initialization */
#define MAX_COMMAND_BUFFER_LEN	(256 + 8)	//UINT32s - space for 256 samples + header
#define NUM_INPUT_CHANNELS      2
#define NUM_OUTPUT_CHANNELS     2



/* Public Global Variables ---------------------------------------------------*/
// This awe instance
AWEInstance g_AWEInstance;

// Heap
UINT32 g_MasterHeap[MASTER_HEAP_SIZE];
UINT32 g_FastbHeap[FASTB_HEAP_SIZE];
UINT32 g_SlowHeap[SLOW_HEAP_SIZE];

// Module table
const void * g_module_descriptor_table[] =
{
    // List of modules from ModuleList.h
    (void *)LISTOFCLASSOBJECTS
};
UINT32 g_module_descriptor_table_size = sizeof(g_module_descriptor_table) / sizeof(g_module_descriptor_table[0]);

// Packet buffer
UINT32 g_packet_buffer[MAX_COMMAND_BUFFER_LEN];


/* Private Global Variables --------------------------------------------------*/
// The input pin
static IOPinDescriptor s_InputPin = { 0 };

// The output pin.
static IOPinDescriptor s_OutputPin = { 0 };


/* Private Function Prototypes -----------------------------------------------*/


/* Function Implementations --------------------------------------------------*/
//-----------------------------------------------------------------------------
// METHOD:  AWEInstanceInit
// PURPOSE: Initialize AWE Instance with target details
//-----------------------------------------------------------------------------
void AweInterface_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    memset(&g_AWEInstance, 0, sizeof(AWEInstance) );

    g_AWEInstance.pInputPin = &s_InputPin;
    g_AWEInstance.pOutputPin = &s_OutputPin;

    awe_initPin(&s_InputPin, NUM_INPUT_CHANNELS, NULL);
    awe_initPin(&s_OutputPin, NUM_OUTPUT_CHANNELS, NULL);

    // User version word
    g_AWEInstance.userVersion = USER_VER;

    g_AWEInstance.instanceId = CORE_ID;
    g_AWEInstance.coreSpeed = CORE_SPEED;
    g_AWEInstance.profileSpeed = CORE_SPEED;
    g_AWEInstance.pName = "ST32F767";
    g_AWEInstance.numThreads = NUM_AUDIO_THREADS;
    g_AWEInstance.pModuleDescriptorTable = g_module_descriptor_table;
    g_AWEInstance.numModules = g_module_descriptor_table_size;
    g_AWEInstance.sampleRate = AUDIO_SAMPLE_RATE;
    g_AWEInstance.fundamentalBlockSize = AUDIO_BLOCK_SIZE;

    // Define the heap sizes
    g_AWEInstance.fastHeapASize = MASTER_HEAP_SIZE;
    g_AWEInstance.fastHeapBSize = FASTB_HEAP_SIZE;
    g_AWEInstance.slowHeapSize  = SLOW_HEAP_SIZE;

    // Point to the heaps on this target
    g_AWEInstance.pFastHeapA = g_MasterHeap;
    g_AWEInstance.pFastHeapB = g_FastbHeap;
    g_AWEInstance.pSlowHeap  = g_SlowHeap;

    g_AWEInstance.pPacketBuffer = g_packet_buffer;
    g_AWEInstance.pReplyBuffer = g_packet_buffer;
    g_AWEInstance.packetBufferSize = MAX_COMMAND_BUFFER_LEN;

    // Initialize AWE signal processing instance
    awe_init(&g_AWEInstance);

}   // End AWEInstanceInit

#if 0
//-----------------------------------------------------------------------------
// METHOD:  BSP_AUDIO_IN_DMA_Handler
// PURPOSE: Interrupt handler - called each time a frame of audio received
//-----------------------------------------------------------------------------
void BSP_AUDIO_IN_DMA_Handler(void)
{
    int nSamplesAvail;
    INT16 * pUSBSamples;
    INT32 layoutMask;
    INT32 bAudioIsStarted;
    INT32 bLayoutValid;

    // Collected USB playback samples from an ASRC component
    pUSBSamples = USB_Playback_ASRCSamples;

    bAudioIsStarted = awe_audioIsStarted(&g_AWEInstance);
    bLayoutValid = awe_layoutIsValid(&g_AWEInstance);

    // If no audio processing running
    if (!bAudioIsStarted)
    {
        // Output zeros (silence)
        memset(AudioBufferOut, 0, sizeof(AudioBufferOut) );
    }
    else
    {
        // Audio playing but no AWD layout loaded
        if (!bLayoutValid)
        {
            // Copy the CODEC stereo input to the CODEC stereo output
            memcpy(&AudioBufferOut[nOutWriteNdx], &AudioBufferIn[nInReadNdx], STEREO_BLOCK_SIZE_IN_SAMPLES * PCM_SIZE_IN_BYTES);
        }
        else
        {
            // Insert the received USB samples into the AudioWeaver buffer
            awe_audioImportSamples(&g_AWEInstance, pUSBSamples, STRIDE2, CHANNEL1, Sample16bit);
            awe_audioImportSamples(&g_AWEInstance, &pUSBSamples[1], STRIDE2, CHANNEL2, Sample16bit);

            // Insert the received CODEC samples into the AudioWeaver buffer
            awe_audioImportSamples(&g_AWEInstance, &AudioBufferIn[nInReadNdx], STRIDE2, CHANNEL3, Sample16bit);
            awe_audioImportSamples(&g_AWEInstance, &AudioBufferIn[nInReadNdx + 1], STRIDE2, CHANNEL4, Sample16bit);

            // Insert the received Mic samples into the AudioWeaver buffer
            awe_audioImportSamples(&g_AWEInstance, &MicBufferIn[nMicReadBufferNdx], STRIDE1, CHANNEL5, Sample16bit);
            awe_audioImportSamples(&g_AWEInstance, &MicBufferIn[nMicReadBufferNdx + AUDIO_BLOCK_SIZE], STRIDE1, CHANNEL6, Sample16bit);
            awe_audioImportSamples(&g_AWEInstance, &MicBufferIn[nMicReadBufferNdx + (AUDIO_BLOCK_SIZE * 2)], STRIDE1, CHANNEL7, Sample16bit);
            awe_audioImportSamples(&g_AWEInstance, &MicBufferIn[nMicReadBufferNdx + (AUDIO_BLOCK_SIZE * 3)], STRIDE1, CHANNEL8, Sample16bit);

            layoutMask = awe_audioGetPumpMask(&g_AWEInstance);

            if (layoutMask > 0)
            {
                // If higher priority level processing ready pend an interrupt for it
                if (layoutMask & 1)
                {
                    if (!g_bAudioPump1Active)
                    {
                        NVIC_SetPendingIRQ(AudioWeaverPump_IRQ1);
                    }
                }

                // If lower priority level processing ready pend an interrupt for it
                if (layoutMask & 2)
                {
                    if (!g_bAudioPump2Active)
                    {
                        NVIC_SetPendingIRQ(AudioWeaverPump_IRQ2);
                    }
                }
            }

            // Insert the processed Audio Weaver samples into the CODEC output buffer
            awe_audioExportSamples(&g_AWEInstance,  &AudioBufferOut[nOutWriteNdx], STRIDE2, CHANNEL1, Sample16bit);
            awe_audioExportSamples(&g_AWEInstance,  &AudioBufferOut[nOutWriteNdx + 1], STRIDE2, CHANNEL2, Sample16bit);

            // Insert the processed Audio Weaver samples into the USB output buffer
            awe_audioExportSamples(&g_AWEInstance,  USB_Record_ASRCSamples, STRIDE2, CHANNEL3, Sample16bit);
            awe_audioExportSamples(&g_AWEInstance,  &USB_Record_ASRCSamples[1], STRIDE2, CHANNEL4, Sample16bit);
        }
    }

    // Switch double buffers
    nInReadNdx = (nInReadNdx + STEREO_BLOCK_SIZE_IN_SAMPLES) % INPUT_AUDIO_BUFFER_SIZE;

    nMicReadBufferNdx  = (nMicReadBufferNdx + NEW_MIC_BUFFER_SAMPLES) % MIC_BUFF_SIZE;

    nOutWriteNdx = (nOutWriteNdx + STEREO_BLOCK_SIZE_IN_SAMPLES) % OUTPUT_AUDIO_BUFFER_SIZE;

}   // End BSP_AUDIO_IN_DMA_Handler


//-----------------------------------------------------------------------------
// METHOD:  AudioWeaver Pump Interrupt Handler
// PURPOSE: Perform AudioWeaver Processing
//-----------------------------------------------------------------------------
void AudioWeaverPump_IRQHandler1(void)
{
    g_bAudioPump1Active = TRUE;

    NVIC_ClearPendingIRQ(AudioWeaverPump_IRQ1);

    g_bDeferredProcessingRequired |= awe_audioPump(&g_AWEInstance, 0);

    g_bAudioPump1Active = FALSE;

}   // End AudioWeaverPump_IRQHandler


//-----------------------------------------------------------------------------
// METHOD:  AudioWeaver Pump Interrupt Handler
// PURPOSE: Perform AudioWeaver Processing
//-----------------------------------------------------------------------------
void AudioWeaverPump_IRQHandler2(void)
{
    g_bAudioPump2Active = TRUE;

    NVIC_ClearPendingIRQ(AudioWeaverPump_IRQ2);

    g_bDeferredProcessingRequired |= awe_audioPump(&g_AWEInstance, 1);

    g_bAudioPump2Active = FALSE;

}   // End AudioWeaverPump_IRQHandler


//-----------------------------------------------------------------------------
// METHOD:  AWEIdleLoop
// PURPOSE: AWE Idle loop processing
//-----------------------------------------------------------------------------
void AWEIdleLoop(void)
{
    BOOL bMoreProcessingRequired = FALSE;

    while(TRUE)
    {
        // Set if a packet is received over USB HID
        if (g_bPacketReceived)
        {
            g_bPacketReceived = FALSE;

            // Process the received packet
            awe_packetProcess(&g_AWEInstance);

            // Send the reply over USB HID
            USBSendReply(&g_AWEInstance);
        }

       // Process any local controls
        if (awe_audioIsStarted(&g_AWEInstance) )
        {
            UINT32 classID;
            INT32 nValue;

            // Perform any needed deferred processing
            if (g_bDeferredProcessingRequired || bMoreProcessingRequired)
            {
                g_bDeferredProcessingRequired = FALSE;
                bMoreProcessingRequired = awe_deferredSetCall(&g_AWEInstance);
            }

            // Does the current AWE model have a SinkInt module with this control object ID?
            if (awe_ctrlGetModuleClass(&g_AWEInstance, AWE_SinkInt1_value_HANDLE, &classID) == OBJECT_FOUND)
            {
                // Check that module assigned this object ID is of module class SinkInt
                if (classID == AWE_SinkInt1_classID)
                {
                    // SinkInt module (gets nValue from the running layout)
                    awe_ctrlGetValue(&g_AWEInstance, AWE_SinkInt1_value_HANDLE, &nValue, 0, 1);

                }
            }

            // Does the current AWE model have a DCSourceInt module with this control object ID?
            if (awe_ctrlGetModuleClass(&g_AWEInstance, AWE_DC1_value_HANDLE, &classID) == OBJECT_FOUND)
            {
                // Check that module assigned this object ID is of module class DCSourceInt
                if (classID == AWE_DC1_classID)
                {
                    // DCSourceInt module (returns nValue to the running layout)
                    awe_ctrlSetValue(&g_AWEInstance, AWE_DC1_value_HANDLE, &nValue, 0, 1);
                }
            }
        }
    }   // End while

}   // End AWEIdleLoop

#endif


//-----------------------------------------------------------------------------
// METHOD:  aweuser_getCycleCount
// PURPOSE: Returns the current value in the counter
//-----------------------------------------------------------------------------
UINT32 aweuser_getCycleCount(void)
{
    return DWT->CYCCNT;

}   // End aweuser_getCycleCount


//-----------------------------------------------------------------------------
// METHOD:  aweuser_getElapsedCycles
// PURPOSE: Returns the cycle count between start time and end time
//-----------------------------------------------------------------------------
UINT32 aweuser_getElapsedCycles(UINT32 nStartTime, UINT32 nEndTime)
{
    return nEndTime - nStartTime;

}   // End aweuser_getElapsedCycles
