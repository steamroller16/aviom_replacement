/*-----------------------------------------------------------------------------
File Name   :
Author      : Sam Soundar
Date        : yyyy-mm-dd
Description :
------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Local Macros/Constants/Structures -----------------------------------------*/
#define AUDIO_SAMPLES_BUFFER_LENGTH (256)
#define NUM_INPUT_CHANNELS          (16)
#define NUM_OUTPUT_CHANNELS         (2)


/* Public Global Variables ---------------------------------------------------*/
extern ETH_HandleTypeDef heth;

extern I2S_HandleTypeDef hi2s2;

__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */
__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */
__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */
__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */


/* Private Global Variables --------------------------------------------------*/
static int32_t SinTable[] = {
//    0,7,13,20,27,33,40,47,53,60,67,73,80,87,93,100,106,113,119,126,132,139,145,152,158,164,171,177,183,189,196,202,208,214,220,226,232,238,244,250,255,261,267,273,278,284,289,295,300,306,311,316,322,327,332,337,342,347,352,357,361,366,371,375,380,384,389,393,397,401,405,409,413,417,421,425,429,432,436,439,443,446,449,452,455,458,461,464,467,470,472,475,477,479,482,484,486,488,490,492,494,495,497,498,500,501,502,504,505,506,507,507,508,509,509,510,510,511,511,511,511,511,511,511,510,510,509,509,508,507,507,506,505,504,502,501,500,498,497,495,494,492,490,488,486,484,482,479,477,475,472,470,467,464,461,458,455,452,449,446,443,439,436,432,429,425,421,417,413,409,405,401,397,393,389,384,380,375,371,366,361,357,352,347,342,337,332,327,322,316,311,306,300,295,289,284,278,273,267,261,256,250,244,238,232,226,220,214,208,202,196,189,183,177,171,164,158,152,145,139,132,126,119,113,106,100,93,87,80,73,67,60,53,47,40,33,27,20,13,7,0,-7,-13,-20,-27,-33,-40,-47,-53,-60,-67,-73,-80,-87,-93,-100,-106,-113,-119,-126,-132,-139,-145,-152,-158,-164,-171,-177,-183,-189,-196,-202,-208,-214,-220,-226,-232,-238,-244,-250,-255,-261,-267,-273,-278,-284,-289,-295,-300,-306,-311,-316,-322,-327,-332,-337,-342,-347,-352,-357,-361,-366,-371,-375,-380,-384,-389,-393,-397,-401,-405,-409,-413,-417,-421,-425,-429,-432,-436,-439,-443,-446,-449,-452,-455,-458,-461,-464,-467,-470,-472,-475,-477,-479,-482,-484,-486,-488,-490,-492,-494,-495,-497,-498,-500,-501,-502,-504,-505,-506,-507,-507,-508,-509,-509,-510,-510,-511,-511,-511,-511,-511,-511,-511,-510,-510,-509,-509,-508,-507,-507,-506,-505,-504,-502,-501,-500,-498,-497,-495,-494,-492,-490,-488,-486,-484,-482,-479,-477,-475,-472,-470,-467,-464,-461,-458,-455,-452,-449,-446,-443,-439,-436,-432,-429,-425,-421,-417,-413,-409,-405,-401,-397,-393,-389,-384,-380,-375,-371,-366,-361,-357,-352,-347,-342,-337,-332,-327,-322,-316,-311,-306,-300,-295,-289,-284,-278,-273,-267,-261,-256,-250,-244,-238,-232,-226,-220,-214,-208,-202,-196,-189,-183,-177,-171,-164,-158,-152,-145,-139,-132,-126,-119,-113,-106,-100,-93,-87,-80,-73,-67,-60,-53,-47,-40,-33,-27,-20,-13,-7
//    0, 13, 27, 40, 53, 67, 80, 93, 106, 119, 132, 145, 158, 171, 183, 196, 208, 220, 232, 244, 255, 267, 278, 289, 300, 311, 322, 332, 342, 352, 361, 371, 380, 389, 397, 405, 413, 421, 429, 436, 443, 449, 455, 461, 467, 472, 477, 482, 486, 490, 494, 497, 500, 502, 505, 507, 508, 509, 510, 511, 511, 511, 510, 509, 508, 507, 505, 502, 500, 497, 494, 490, 486, 482, 477, 472, 467, 461, 455, 449, 443, 436, 429, 421, 413, 405, 397, 389, 380, 371, 361, 352, 342, 332, 322, 311, 300, 289, 278, 267, 256, 244, 232, 220, 208, 196, 183, 171, 158, 145, 132, 119, 106, 93, 80, 67, 53, 40, 27, 13, 0, -13, -27, -40, -53, -67, -80, -93, -106, -119, -132, -145, -158, -171, -183, -196, -208, -220, -232, -244, -255, -267, -278, -289, -300, -311, -322, -332, -342, -352, -361, -371, -380, -389, -397, -405, -413, -421, -429, -436, -443, -449, -455, -461, -467, -472, -477, -482, -486, -490, -494, -497, -500, -502, -505, -507, -508, -509, -510, -511, -511, -511, -510, -509, -508, -507, -505, -502, -500, -497, -494, -490, -486, -482, -477, -472, -467, -461, -455, -449, -443, -436, -429, -421, -413, -405, -397, -389, -380, -371, -361, -352, -342, -332, -322, -311, -300, -289, -278, -267, -256, -244, -232, -220, -208, -196, -183, -171, -158, -145, -132, -119, -106, -93, -80, -67, -53, -40, -27, -13,
//    0, 0, 13, 13, 27, 27, 40, 40, 53, 53, 67, 67, 80, 80, 93, 93, 106, 106, 119, 119, 132, 132, 145, 145, 158, 158, 171, 171, 183, 183, 196, 196, 208, 208, 220, 220, 232, 232, 244, 244, 255, 255, 267, 267, 278, 278, 289, 289, 300, 300, 311, 311, 322, 322, 332, 332, 342, 342, 352, 352, 361, 361, 371, 371, 380, 380, 389, 389, 397, 397, 405, 405, 413, 413, 421, 421, 429, 429, 436, 436, 443, 443, 449, 449, 455, 455, 461, 461, 467, 467, 472, 472, 477, 477, 482, 482, 486, 486, 490, 490, 494, 494, 497, 497, 500, 500, 502, 502, 505, 505, 507, 507, 508, 508, 509, 509, 510, 510, 511, 511, 511, 511, 511, 511, 510, 510, 509, 509, 508, 508, 507, 507, 505, 505, 502, 502, 500, 500, 497, 497, 494, 494, 490, 490, 486, 486, 482, 482, 477, 477, 472, 472, 467, 467, 461, 461, 455, 455, 449, 449, 443, 443, 436, 436, 429, 429, 421, 421, 413, 413, 405, 405, 397, 397, 389, 389, 380, 380, 371, 371, 361, 361, 352, 352, 342, 342, 332, 332, 322, 322, 311, 311, 300, 300, 289, 289, 278, 278, 267, 267, 256, 256, 244, 244, 232, 232, 220, 220, 208, 208, 196, 196, 183, 183, 171, 171, 158, 158, 145, 145, 132, 132, 119, 119, 106, 106, 93, 93, 80, 80, 67, 67, 53, 53, 40, 40, 27, 27, 13, 13, 0, 0, -13, -13, -27, -27, -40, -40, -53, -53, -67, -67, -80, -80, -93, -93, -106, -106, -119, -119, -132, -132, -145, -145, -158, -158, -171, -171, -183, -183, -196, -196, -208, -208, -220, -220, -232, -232, -244, -244, -255, -255, -267, -267, -278, -278, -289, -289, -300, -300, -311, -311, -322, -322, -332, -332, -342, -342, -352, -352, -361, -361, -371, -371, -380, -380, -389, -389, -397, -397, -405, -405, -413, -413, -421, -421, -429, -429, -436, -436, -443, -443, -449, -449, -455, -455, -461, -461, -467, -467, -472, -472, -477, -477, -482, -482, -486, -486, -490, -490, -494, -494, -497, -497, -500, -500, -502, -502, -505, -505, -507, -507, -508, -508, -509, -509, -510, -510, -511, -511, -511, -511, -511, -511, -510, -510, -509, -509, -508, -508, -507, -507, -505, -505, -502, -502, -500, -500, -497, -497, -494, -494, -490, -490, -486, -486, -482, -482, -477, -477, -472, -472, -467, -467, -461, -461, -455, -455, -449, -449, -443, -443, -436, -436, -429, -429, -421, -421, -413, -413, -405, -405, -397, -397, -389, -389, -380, -380, -371, -371, -361, -361, -352, -352, -342, -342, -332, -332, -322, -322, -311, -311, -300, -300, -289, -289, -278, -278, -267, -267, -256, -256, -244, -244, -232, -232, -220, -220, -208, -208, -196, -196, -183, -183, -171, -171, -158, -158, -145, -145, -132, -132, -119, -119, -106, -106, -93, -93, -80, -80, -67, -67, -53, -53, -40, -40, -27, -27, -13, -13,
    0, 0, 27, 27, 53, 53, 80, 80, 106, 106, 132, 132, 158, 158, 183, 183, 208, 208, 232, 232, 255, 255, 278, 278, 300, 300, 322, 322, 342, 342, 361, 361, 380, 380, 397, 397, 413, 413, 429, 429, 443, 443, 455, 455, 467, 467, 477, 477, 486, 486, 494, 494, 500, 500, 505, 505, 508, 508, 510, 510, 511, 511, 510, 510, 508, 508, 505, 505, 500, 500, 494, 494, 486, 486, 477, 477, 467, 467, 455, 455, 443, 443, 429, 429, 413, 413, 397, 397, 380, 380, 361, 361, 342, 342, 322, 322, 300, 300, 278, 278, 256, 256, 232, 232, 208, 208, 183, 183, 158, 158, 132, 132, 106, 106, 80, 80, 53, 53, 27, 27, 0, 0, -27, -27, -53, -53, -80, -80, -106, -106, -132, -132, -158, -158, -183, -183, -208, -208, -232, -232, -255, -255, -278, -278, -300, -300, -322, -322, -342, -342, -361, -361, -380, -380, -397, -397, -413, -413, -429, -429, -443, -443, -455, -455, -467, -467, -477, -477, -486, -486, -494, -494, -500, -500, -505, -505, -508, -508, -510, -510, -511, -511, -510, -510, -508, -508, -505, -505, -500, -500, -494, -494, -486, -486, -477, -477, -467, -467, -455, -455, -443, -443, -429, -429, -413, -413, -397, -397, -380, -380, -361, -361, -342, -342, -322, -322, -300, -300, -278, -278, -256, -256, -232, -232, -208, -208, -183, -183, -158, -158, -132, -132, -106, -106, -80, -80, -53, -53, -27, -27,
};

static int32_t AudioSamplesBuffer[NUM_INPUT_CHANNELS][AUDIO_SAMPLES_BUFFER_LENGTH][NUM_OUTPUT_CHANNELS];
static uint8_t AudioSamplesBufferIndex = 0;
static uint8_t AudioSamplesBufferOutdex = 0;


/* Private Function Prototypes -----------------------------------------------*/


/* Function Implementations --------------------------------------------------*/
void AviomReceiver_Init(void)
{
    uint16_t i;
    for (i=0; i<sizeof(SinTable)/sizeof(SinTable[0]); i++)
    {
        SinTable[i] = ((SinTable[i] & 0x0000FFFF) << 16) + ((SinTable[i] & 0xFFFF0000) >> 16);
        SinTable[i] <<= 8; // Make sine wave level audible
    }

    /* Initialize Tx Descriptors list: Chain Mode */
    HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

    /* Initialize Rx Descriptors list: Chain Mode  */
    HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

    /* Enable MAC and DMA transmission and reception */
    HAL_ETH_Start(&heth);

    /* Read Register Configuration */
    uint32_t regvalue;
    HAL_ETH_ReadPHYRegister(&heth, PHY_ISFR, &regvalue);
    regvalue |= (PHY_ISFR_INT4);

    /* Enable Interrupt on change of link status */
    HAL_ETH_WritePHYRegister(&heth, PHY_ISFR , regvalue );

    /* Read Register Configuration */
    HAL_ETH_ReadPHYRegister(&heth, PHY_ISFR , &regvalue);
}

void AviomReceiver_MainLoopProcess(void)
{
    if ((HAL_I2S_GetState(&hi2s2) == HAL_I2S_STATE_READY) &&
        ( (AudioSamplesBufferIndex - AudioSamplesBufferOutdex >= 64) ||
          (AudioSamplesBufferIndex < AudioSamplesBufferOutdex) ))
    {
        HAL_GPIO_WritePin(DEBUG4_GPIO_Port, DEBUG4_Pin, GPIO_PIN_SET);
        HAL_I2S_Transmit_IT(&hi2s2, (uint16_t *)AudioSamplesBuffer[0][AudioSamplesBufferOutdex], 64*2);
        AudioSamplesBufferOutdex += 64;
        if (AudioSamplesBufferOutdex >= AUDIO_SAMPLES_BUFFER_LENGTH)
        {
            AudioSamplesBufferOutdex = 0;
        }
        HAL_GPIO_WritePin(DEBUG4_GPIO_Port, DEBUG4_Pin, GPIO_PIN_RESET);
    }
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *dummy)
{
    HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_SET);
    if (HAL_ETH_GetReceivedFrame_IT(&heth) == HAL_OK)
    {
        if (heth.RxFrameInfos.length == 100)
        {
            uint8_t *buffer = (uint8_t *)heth.RxFrameInfos.buffer;
            uint8_t sampleIndex = 0;
            uint8_t inputChannelNum = 0;

            for (sampleIndex = 0; sampleIndex < 32; sampleIndex++)
            {
                AudioSamplesBuffer[inputChannelNum][AudioSamplesBufferIndex][0] =
                    (buffer[4+sampleIndex*3+0] << 24) +
                    (buffer[4+sampleIndex*3+1] << 16) +
                    (buffer[4+sampleIndex*3+2] <<  8);
                AudioSamplesBuffer[inputChannelNum][AudioSamplesBufferIndex][0] <<= 6; // Volume adjust
                AudioSamplesBuffer[inputChannelNum][AudioSamplesBufferIndex][0] =
                    ((AudioSamplesBuffer[inputChannelNum][AudioSamplesBufferIndex][0] & 0x0000FFFF) << 16) +
                    ((AudioSamplesBuffer[inputChannelNum][AudioSamplesBufferIndex][0] & 0xFFFF0000) >> 16);
                AudioSamplesBuffer[inputChannelNum][AudioSamplesBufferIndex][1] =
                    AudioSamplesBuffer[inputChannelNum][AudioSamplesBufferIndex][0];

                inputChannelNum++;
                if (inputChannelNum == 16)
                {
                    inputChannelNum = 0;
                    AudioSamplesBufferIndex++;
                    if (AudioSamplesBufferIndex >= AUDIO_SAMPLES_BUFFER_LENGTH)
                    {
                        AudioSamplesBufferIndex = 0;
                    }
                }
            }
        }
        else
        {
            HAL_GPIO_TogglePin(DEBUG3_GPIO_Port, DEBUG3_Pin);
        }

        __IO ETH_DMADescTypeDef *dmarxdesc;
        /* Release descriptors to DMA */
        /* Point to first descriptor */
        dmarxdesc = heth.RxFrameInfos.FSRxDesc;
        /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
        uint32_t i;
        for (i=0; i< heth.RxFrameInfos.SegCount; i++)
        {
            dmarxdesc->Status |= ETH_DMARXDESC_OWN;
            dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
        }
        /* Clear Segment_Count */
        heth.RxFrameInfos.SegCount =0;
        /* When Rx Buffer unavailable flag is set: clear it and resume reception */
        if ((heth.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
        {
          /* Clear RBUS ETHERNET DMA flag */
          heth.Instance->DMASR = ETH_DMASR_RBUS;
          /* Resume DMA reception */
          heth.Instance->DMARPDR = 0;
        }
    }
    HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);
}
