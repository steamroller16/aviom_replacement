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

#include "awe_tuning_hw_interface.h"


/* Local Macros/Constants/Structures -----------------------------------------*/
// Version Information
#define USER_VERSION            (20201222) // 2020/12/22

// Various parameters
#define INSTANCE_ID             (0)
#define CORE_SPEED              (192e6f)
#define PROFILE_SPEED           (192e6f)
#define NUM_AUDIO_THREADS       (1)
#define AUDIO_SAMPLE_RATE       (48000.0f)
#define AUDIO_BLOCK_SIZE        (32)

// Specify the size of each of the heaps on this target
#define MASTER_HEAP_SIZE        (1024*5)
#define FASTB_HEAP_SIZE         (1024*5)
#define SLOW_HEAP_SIZE          (1024*5)

/* Define number of input and output channels */
#define NUM_INPUT_CHANNELS      (2)
#define NUM_OUTPUT_CHANNELS     (2)


/* Public Global Variables ---------------------------------------------------*/


/* Private Global Variables --------------------------------------------------*/
// AWE instance
static AWEInstance AweInstance;

// Heap
static UINT32 MasterHeap[MASTER_HEAP_SIZE];
static UINT32 FastbHeap[FASTB_HEAP_SIZE];
static UINT32 SlowHeap[SLOW_HEAP_SIZE];

// Module table
static const void * ModuleDescriptorTable[] =
{
    // List of modules from ModuleList.h
    (void *)LISTOFCLASSOBJECTS
};
static const UINT32 ModuleDescriptorTableSize = sizeof(ModuleDescriptorTable) / sizeof(ModuleDescriptorTable[0]);

// AWE IO pins (not physical GPIO pins)
static IOPinDescriptor s_InputPin =  { 0 };
static IOPinDescriptor s_OutputPin = { 0 };


/* Private Function Prototypes -----------------------------------------------*/


/* Function Implementations --------------------------------------------------*/
void AweInterface_Init(void)
{
    // Setup execution cycle counter (ARM stuff)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Zero out AWE instance struct
    memset(&AweInstance, 0, sizeof(AWEInstance) );

    // Setup AWE IO pins
    AweInstance.pInputPin = &s_InputPin;
    AweInstance.pOutputPin = &s_OutputPin;
    awe_initPin(&s_InputPin, NUM_INPUT_CHANNELS, NULL);
    awe_initPin(&s_OutputPin, NUM_OUTPUT_CHANNELS, NULL);

    // Fill in various parameters
    AweInstance.userVersion = USER_VERSION;
    AweInstance.instanceId = INSTANCE_ID;
    AweInstance.coreSpeed = CORE_SPEED;
    AweInstance.profileSpeed = PROFILE_SPEED;
    AweInstance.pName = "ST32F767";
    AweInstance.numThreads = NUM_AUDIO_THREADS;
    AweInstance.pModuleDescriptorTable = ModuleDescriptorTable;
    AweInstance.numModules = ModuleDescriptorTableSize;
    AweInstance.sampleRate = AUDIO_SAMPLE_RATE;
    AweInstance.fundamentalBlockSize = AUDIO_BLOCK_SIZE;

    // Define the heap sizes and ram
    AweInstance.fastHeapASize = MASTER_HEAP_SIZE;
    AweInstance.fastHeapBSize = FASTB_HEAP_SIZE;
    AweInstance.slowHeapSize  = SLOW_HEAP_SIZE;
    AweInstance.pFastHeapA = MasterHeap;
    AweInstance.pFastHeapB = FastbHeap;
    AweInstance.pSlowHeap  = SlowHeap;

    // Tuning interface buffers
    AweInstance.pPacketBuffer = AweTuningHwInterface_GetPacketBufferPointer();
    AweInstance.pReplyBuffer = AweTuningHwInterface_GetReplyBufferPointer();
    AweInstance.packetBufferSize = AweTuningHwInterface_GetMaxPacketBufferSizeWords();

    // Init AWE signal processing instance
    awe_init(&AweInstance);

    // Init tuning hardware interface
    AweTuningHwInterface_Init(&AweInstance);
}

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

    bAudioIsStarted = awe_audioIsStarted(&AweInstance);
    bLayoutValid = awe_layoutIsValid(&AweInstance);

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
            awe_audioImportSamples(&AweInstance, pUSBSamples, STRIDE2, CHANNEL1, Sample16bit);
            awe_audioImportSamples(&AweInstance, &pUSBSamples[1], STRIDE2, CHANNEL2, Sample16bit);

            // Insert the received CODEC samples into the AudioWeaver buffer
            awe_audioImportSamples(&AweInstance, &AudioBufferIn[nInReadNdx], STRIDE2, CHANNEL3, Sample16bit);
            awe_audioImportSamples(&AweInstance, &AudioBufferIn[nInReadNdx + 1], STRIDE2, CHANNEL4, Sample16bit);

            // Insert the received Mic samples into the AudioWeaver buffer
            awe_audioImportSamples(&AweInstance, &MicBufferIn[nMicReadBufferNdx], STRIDE1, CHANNEL5, Sample16bit);
            awe_audioImportSamples(&AweInstance, &MicBufferIn[nMicReadBufferNdx + AUDIO_BLOCK_SIZE], STRIDE1, CHANNEL6, Sample16bit);
            awe_audioImportSamples(&AweInstance, &MicBufferIn[nMicReadBufferNdx + (AUDIO_BLOCK_SIZE * 2)], STRIDE1, CHANNEL7, Sample16bit);
            awe_audioImportSamples(&AweInstance, &MicBufferIn[nMicReadBufferNdx + (AUDIO_BLOCK_SIZE * 3)], STRIDE1, CHANNEL8, Sample16bit);

            layoutMask = awe_audioGetPumpMask(&AweInstance);

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
            awe_audioExportSamples(&AweInstance,  &AudioBufferOut[nOutWriteNdx], STRIDE2, CHANNEL1, Sample16bit);
            awe_audioExportSamples(&AweInstance,  &AudioBufferOut[nOutWriteNdx + 1], STRIDE2, CHANNEL2, Sample16bit);

            // Insert the processed Audio Weaver samples into the USB output buffer
            awe_audioExportSamples(&AweInstance,  USB_Record_ASRCSamples, STRIDE2, CHANNEL3, Sample16bit);
            awe_audioExportSamples(&AweInstance,  &USB_Record_ASRCSamples[1], STRIDE2, CHANNEL4, Sample16bit);
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

    g_bDeferredProcessingRequired |= awe_audioPump(&AweInstance, 0);

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

    g_bDeferredProcessingRequired |= awe_audioPump(&AweInstance, 1);

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
            awe_packetProcess(&AweInstance);

            // Send the reply over USB HID
            USBSendReply(&AweInstance);
        }

       // Process any local controls
        if (awe_audioIsStarted(&AweInstance) )
        {
            UINT32 classID;
            INT32 nValue;

            // Perform any needed deferred processing
            if (g_bDeferredProcessingRequired || bMoreProcessingRequired)
            {
                g_bDeferredProcessingRequired = FALSE;
                bMoreProcessingRequired = awe_deferredSetCall(&AweInstance);
            }

            // Does the current AWE model have a SinkInt module with this control object ID?
            if (awe_ctrlGetModuleClass(&AweInstance, AWE_SinkInt1_value_HANDLE, &classID) == OBJECT_FOUND)
            {
                // Check that module assigned this object ID is of module class SinkInt
                if (classID == AWE_SinkInt1_classID)
                {
                    // SinkInt module (gets nValue from the running layout)
                    awe_ctrlGetValue(&AweInstance, AWE_SinkInt1_value_HANDLE, &nValue, 0, 1);

                }
            }

            // Does the current AWE model have a DCSourceInt module with this control object ID?
            if (awe_ctrlGetModuleClass(&AweInstance, AWE_DC1_value_HANDLE, &classID) == OBJECT_FOUND)
            {
                // Check that module assigned this object ID is of module class DCSourceInt
                if (classID == AWE_DC1_classID)
                {
                    // DCSourceInt module (returns nValue to the running layout)
                    awe_ctrlSetValue(&AweInstance, AWE_DC1_value_HANDLE, &nValue, 0, 1);
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
