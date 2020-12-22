/*-----------------------------------------------------------------------------
File Name   : awe_tuning_hw_interface.c
Author      : Sam Soundar
Date        : 2020-12-22
Description : Implements the tuning interface hardware components (UART)
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "AWECore.h"
#include "AWECoreUtils.h"


/* Local Macros/Constants/Structures -----------------------------------------*/
#define MAX_PACKET_BUFFER_SIZE_WORDS            (256+8) //UINT32s - space for 256 samples + header
#define MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES    (MAX_PACKET_BUFFER_SIZE_WORDS*5 + 3) // See comment in tuningEncodeByteUART


/* Public Global Variables ---------------------------------------------------*/
extern UART_HandleTypeDef huart3;


/* Private Global Variables --------------------------------------------------*/
static AWEInstance *AweInstance;

// Packet buffers
static UINT8 PacketBufferEncoded[MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES];
static UINT8 ReplyBufferEncoded[MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES];

static UINT32 PacketBuffer[MAX_PACKET_BUFFER_SIZE_WORDS];
static UINT32 ReplyBuffer[MAX_PACKET_BUFFER_SIZE_WORDS];


/* Private Function Prototypes -----------------------------------------------*/


/* Function Implementations --------------------------------------------------*/
void AweTuningHwInterface_Init(AWEInstance * aweInstance)
{
    // Save pointer to AWE instance
    AweInstance = aweInstance;

    // Setup UART for reception
    HAL_UART_ReceiverTimeout_Config(&huart3, 20);
    HAL_UART_EnableReceiverTimeout(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RTO);
    HAL_UART_Receive_IT(&huart3, (UINT8 *)&PacketBufferEncoded[0], MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES);
}

UINT32 * AweTuningHwInterface_GetPacketBufferPointer(void)
{
    return PacketBuffer;
}

UINT32 * AweTuningHwInterface_GetReplyBufferPointer(void)
{
    return ReplyBuffer;
}

UINT32 AweTuningHwInterface_GetMaxPacketBufferSizeWords(void)
{
    return MAX_PACKET_BUFFER_SIZE_WORDS;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // Receiver timeout is the main method to indicate packet reception complete
    // Check for receive timeout
    if (HAL_UART_GetError(&huart3) == HAL_UART_ERROR_RTO)
    {
        // Iterate through received bytes and call tuningDecodeByteUART
        INT32 decodeReturn;
        UINT32 countReceive;
        for (countReceive = 0; countReceive < MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES; countReceive++)
        {
            decodeReturn = tuningDecodeByteUART(PacketBuffer, PacketBufferEncoded[countReceive]);

            // If complete packet decoded
            if (decodeReturn == COMPLETE_NEW_PACKET)
            {
                // Process packet
                awe_packetProcess(AweInstance);

                // Iterate through reply buffer and call tuningEncodeByteUART
                INT32 encodeReturn;
                UINT32 countTransmit;
                for (countTransmit = 0; countTransmit < MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES; countTransmit++)
                {
                    encodeReturn = tuningEncodeByteUART(ReplyBuffer, &ReplyBufferEncoded[countTransmit]);

                    // If complete packet encoded
                    if (encodeReturn == COMPLETE_NEW_PACKET)
                    {
                        // Transmit encoded packet
                        HAL_UART_Transmit_IT(&huart3, (UINT8 *)&ReplyBufferEncoded[0], countTransmit);
                        break;
                    }
                }
                break;
            }
            // If packet repeated
            else if (decodeReturn == COMPLETE_REPEATED_PACKET)
            {
                // DO NOT process packet again since it was repeated
                // Simply encode reply again and transmit

                // Iterate through reply buffer and call tuningEncodeByteUART
                INT32 encodeReturn;
                UINT32 countTransmit;
                for (countTransmit = 0; countTransmit < MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES; countTransmit++)
                {
                    encodeReturn = tuningEncodeByteUART(ReplyBuffer, &ReplyBufferEncoded[countTransmit]);

                    // If complete packet encoded
                    if (encodeReturn == COMPLETE_NEW_PACKET)
                    {
                        // Transmit encoded packet
                        HAL_UART_Transmit_IT(&huart3, (UINT8 *)&ReplyBufferEncoded[0], countTransmit);
                        break;
                    }
                }
                break;
            }
            // If error while decoding
            else if (decodeReturn == E_BADPACKET)
            {
                // Go back to listening
                HAL_UART_Receive_IT(&huart3, (UINT8 *)&PacketBufferEncoded[0], MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES);
                break;
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Not implemented. Don't think we will ever get here.
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // When transmission is complete, go back to listening again
    HAL_UART_Receive_IT(&huart3, (UINT8 *)&PacketBufferEncoded[0], MAX_PACKET_BUFFER_ENCODED_SIZE_BYTES);
}
