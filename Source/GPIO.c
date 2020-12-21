/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     GPIO.c
********************************************************************************
*
*     Description:  General Purpose I/O module
*
*     Copyright:    (c) 2018 DSP Concepts, Inc. All rights reserved.
*                   3235 Kifer Road
*                   Santa Clara, CA 95054
*
*******************************************************************************/
#include "StandardDefs.h"

#define MAX_PINS 2

//-----------------------------------------------------------------------------
// METHOD:  awe_pltGPIOSetPinDir
// PURPOSE: Set GPIO pin direction
//-----------------------------------------------------------------------------
void awe_pltGPIOSetPinDir(UINT32 nPinNo, UINT32 nPinDir)
{
    // No general purpose I/O pins available on this board

}   // End awe_pltGPIOSetPinDir


//-----------------------------------------------------------------------------
// METHOD:  awe_pltGPIOSetPin
// PURPOSE: Set GBPIO pin value
//-----------------------------------------------------------------------------
void awe_pltGPIOSetPin(UINT32 nPinNo, UINT32 nValue)
{
    if (nPinNo < 1 || nPinNo > MAX_PINS)
    {
        return;
    }

    if (nPinNo == 1)
    {
        if (nValue > 0)
        {
            // BSP_LED_On(LED_RED);
        }
        else
        {
            // BSP_LED_Off(LED_RED);
        }
    }
    else
    {
        // User now using green LED to don't use for keep alive indicator
        // g_bBlinkGreenLEDForBoardAlive = FALSE;

        if (nValue > 0)
        {
            // BSP_LED_On(LED_GREEN);
        }
        else
        {
            // BSP_LED_Off(LED_GREEN);
        }
    }

}   // End awe_pltGPIOSetPin


//-----------------------------------------------------------------------------
// METHOD:  awe_pltGPIOGetPin
// PURPOSE: Get GPIO pin value
//-----------------------------------------------------------------------------
void awe_pltGPIOGetPin(UINT32 nPinNo, UINT32 * nValue)
{
    if (nPinNo != 1)
    {
        return;
    }

    // *nValue = BSP_PB_GetState(BUTTON_USER);

}   // End awe_pltGPIOGetPin
