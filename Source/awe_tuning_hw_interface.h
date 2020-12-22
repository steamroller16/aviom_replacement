/*-----------------------------------------------------------------------------
File Name   : awe_tuning_hw_interface.h
Author      : Sam Soundar
Date        : 2020-12-22
Description : Implements the tuning interface hardware components (UART)
------------------------------------------------------------------------------*/
#ifndef AWE_TUNING_HW_INTERFACE_H
#define AWE_TUNING_HW_INTERFACE_H

/* Includes ------------------------------------------------------------------*/


/* Public Macros/Constants/Structures ----------------------------------------*/


/* Public Variables ----------------------------------------------------------*/


/* Public Function Declarations ----------------------------------------------*/
void AweTuningHwInterface_Init(AWEInstance * AWEInstance);

UINT32 * AweTuningHwInterface_GetPacketBufferPointer(void);
UINT32 * AweTuningHwInterface_GetReplyBufferPointer(void);
UINT32 AweTuningHwInterface_GetMaxPacketBufferSizeWords(void);


#endif
