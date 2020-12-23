/*-----------------------------------------------------------------------------
File Name   :
Author      : Sam Soundar
Date        : yyyy-mm-dd
Description :
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "main.h"

#include "AWECore.h"
#include "TargetProcessor.h"
#include "StandardDefs.h"
#include "Errors.h"
#include "ModuleList.h"

#include "awe_interface.h"
#include "awe_tuning_hw_interface.h"
#include "sound_card_interface.h"


/* Local Macros/Constants/Structures -----------------------------------------*/
// Version Information
#define USER_VERSION            (20201222) // 2020/12/22

// Various parameters
#define INSTANCE_ID             (0)
#define CORE_SPEED              (192e6f)
#define PROFILE_SPEED           (192e6f)
#define NUM_AUDIO_THREADS       (1)
#define AUDIO_SAMPLE_RATE       (48000.0f)

// Specify the size of each of the heaps on this target
#define MASTER_HEAP_SIZE        (1024*5)
#define FASTB_HEAP_SIZE         (1024*5)
#define SLOW_HEAP_SIZE          (1024*5)


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

static bool DeferredProcessingRequired = false;


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
    awe_initPin(&s_InputPin, AWE_INTERFACE_NUM_INPUT_CHANNELS, NULL);
    awe_initPin(&s_OutputPin, AWE_INTERFACE_NUM_OUTPUT_CHANNELS, NULL);

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
    AweInstance.fundamentalBlockSize = AWE_INTERFACE_AUDIO_BLOCK_SIZE;

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


//-----------------------------------------------------------------------------
// METHOD:  BSP_AUDIO_IN_DMA_Handler
// PURPOSE: Interrupt handler - called each time a frame of audio received
//-----------------------------------------------------------------------------
void AweInterface_HandleInputSamplesBlock(int32_t *inputSamplesBlock)
{
    INT32 layoutMask;
    INT32 bAudioIsStarted;
    INT32 bLayoutValid;

    bAudioIsStarted = awe_audioIsStarted(&AweInstance);
    bLayoutValid = awe_layoutIsValid(&AweInstance);

    int32_t *outputSamplesBlock = SoundCardInterface_GetAudioSamplesBuffer();

    // If no audio processing running
    if (!bAudioIsStarted)
    {
        // Output zeros (silence)
        memset(outputSamplesBlock, 0, AWE_INTERFACE_AUDIO_BLOCK_SIZE * AWE_INTERFACE_NUM_OUTPUT_CHANNELS * sizeof(*outputSamplesBlock));
        SoundCardInterface_NotifySamplesBufferFull(outputSamplesBlock);
    }
    else
    {
        // Audio playing but no AWD layout loaded
        if (!bLayoutValid)
        {
            // TODO: properly copy first two channels to the output
            // memcpy(&outputSamplesBlock[0], &inputSamplesBlock[0], AWE_INTERFACE_AUDIO_BLOCK_SIZE * sizeof(int32_t));
            SoundCardInterface_NotifySamplesBufferFull(outputSamplesBlock);
        }
        else
        {
            // Insert the received samples into the AudioWeaver buffer
            uint8_t channelNum;
            for (channelNum = 0; channelNum < AWE_INTERFACE_NUM_INPUT_CHANNELS; channelNum++)
            {
                awe_audioImportSamples(&AweInstance, &inputSamplesBlock[channelNum], AWE_INTERFACE_NUM_INPUT_CHANNELS, channelNum, Sample32bit);
            }

            layoutMask = awe_audioGetPumpMask(&AweInstance);

            if (layoutMask > 0)
            {
                // If higher priority level processing ready pend an interrupt for it
                if (layoutMask & 1)
                {
                    // if (!g_bAudioPump1Active)
                    // {
                        // NVIC_SetPendingIRQ(AudioWeaverPump_IRQ1);
                    // }
                    DeferredProcessingRequired |= awe_audioPump(&AweInstance, 0);
                }

                // If lower priority level processing ready pend an interrupt for it
                if (layoutMask & 2)
                {
                    // if (!g_bAudioPump2Active)
                    // {
                        // NVIC_SetPendingIRQ(AudioWeaverPump_IRQ2);
                    // }
                    DeferredProcessingRequired |= awe_audioPump(&AweInstance, 1);
                }
            }

            for (channelNum = 0; channelNum < AWE_INTERFACE_NUM_OUTPUT_CHANNELS; channelNum++)
            {
                awe_audioExportSamples(&AweInstance, &outputSamplesBlock[channelNum], AWE_INTERFACE_NUM_OUTPUT_CHANNELS, channelNum, Sample32bit);
            }
            SoundCardInterface_NotifySamplesBufferFull(outputSamplesBlock);
        }
    }
}   // End BSP_AUDIO_IN_DMA_Handler

#if 0
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
#endif

//-----------------------------------------------------------------------------
// METHOD:  AWEIdleLoop
// PURPOSE: AWE Idle loop processing
//-----------------------------------------------------------------------------
void AweInterface_MainLoopProcess(void)
{
    bool moreProcessingRequired = false;

   // Process any local controls
    if (awe_audioIsStarted(&AweInstance))
    {
        UINT32 classID;
        INT32 nValue;

        // Perform any needed deferred processing
        while (DeferredProcessingRequired || moreProcessingRequired)
        {
            DeferredProcessingRequired = false;
            moreProcessingRequired = awe_deferredSetCall(&AweInstance);
        }

        #if 0
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
        #endif
    }

}   // End AWEIdleLoop


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
