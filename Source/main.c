/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

I2S_HandleTypeDef hi2s2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */
__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */
__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */
__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */

#define AUDIO_SAMPLES_BUFFER_LENGTH (256)
#define NUM_INPUT_CHANNELS          (16)
#define NUM_OUTPUT_CHANNELS         (2)
int32_t AudioSamplesBuffer[NUM_INPUT_CHANNELS][AUDIO_SAMPLES_BUFFER_LENGTH][NUM_OUTPUT_CHANNELS];
uint8_t AudioSamplesBufferIndex = 0;
uint8_t AudioSamplesBufferOutdex = 0;

static int32_t SinTable[] = {
//    0,7,13,20,27,33,40,47,53,60,67,73,80,87,93,100,106,113,119,126,132,139,145,152,158,164,171,177,183,189,196,202,208,214,220,226,232,238,244,250,255,261,267,273,278,284,289,295,300,306,311,316,322,327,332,337,342,347,352,357,361,366,371,375,380,384,389,393,397,401,405,409,413,417,421,425,429,432,436,439,443,446,449,452,455,458,461,464,467,470,472,475,477,479,482,484,486,488,490,492,494,495,497,498,500,501,502,504,505,506,507,507,508,509,509,510,510,511,511,511,511,511,511,511,510,510,509,509,508,507,507,506,505,504,502,501,500,498,497,495,494,492,490,488,486,484,482,479,477,475,472,470,467,464,461,458,455,452,449,446,443,439,436,432,429,425,421,417,413,409,405,401,397,393,389,384,380,375,371,366,361,357,352,347,342,337,332,327,322,316,311,306,300,295,289,284,278,273,267,261,256,250,244,238,232,226,220,214,208,202,196,189,183,177,171,164,158,152,145,139,132,126,119,113,106,100,93,87,80,73,67,60,53,47,40,33,27,20,13,7,0,-7,-13,-20,-27,-33,-40,-47,-53,-60,-67,-73,-80,-87,-93,-100,-106,-113,-119,-126,-132,-139,-145,-152,-158,-164,-171,-177,-183,-189,-196,-202,-208,-214,-220,-226,-232,-238,-244,-250,-255,-261,-267,-273,-278,-284,-289,-295,-300,-306,-311,-316,-322,-327,-332,-337,-342,-347,-352,-357,-361,-366,-371,-375,-380,-384,-389,-393,-397,-401,-405,-409,-413,-417,-421,-425,-429,-432,-436,-439,-443,-446,-449,-452,-455,-458,-461,-464,-467,-470,-472,-475,-477,-479,-482,-484,-486,-488,-490,-492,-494,-495,-497,-498,-500,-501,-502,-504,-505,-506,-507,-507,-508,-509,-509,-510,-510,-511,-511,-511,-511,-511,-511,-511,-510,-510,-509,-509,-508,-507,-507,-506,-505,-504,-502,-501,-500,-498,-497,-495,-494,-492,-490,-488,-486,-484,-482,-479,-477,-475,-472,-470,-467,-464,-461,-458,-455,-452,-449,-446,-443,-439,-436,-432,-429,-425,-421,-417,-413,-409,-405,-401,-397,-393,-389,-384,-380,-375,-371,-366,-361,-357,-352,-347,-342,-337,-332,-327,-322,-316,-311,-306,-300,-295,-289,-284,-278,-273,-267,-261,-256,-250,-244,-238,-232,-226,-220,-214,-208,-202,-196,-189,-183,-177,-171,-164,-158,-152,-145,-139,-132,-126,-119,-113,-106,-100,-93,-87,-80,-73,-67,-60,-53,-47,-40,-33,-27,-20,-13,-7
//    0, 13, 27, 40, 53, 67, 80, 93, 106, 119, 132, 145, 158, 171, 183, 196, 208, 220, 232, 244, 255, 267, 278, 289, 300, 311, 322, 332, 342, 352, 361, 371, 380, 389, 397, 405, 413, 421, 429, 436, 443, 449, 455, 461, 467, 472, 477, 482, 486, 490, 494, 497, 500, 502, 505, 507, 508, 509, 510, 511, 511, 511, 510, 509, 508, 507, 505, 502, 500, 497, 494, 490, 486, 482, 477, 472, 467, 461, 455, 449, 443, 436, 429, 421, 413, 405, 397, 389, 380, 371, 361, 352, 342, 332, 322, 311, 300, 289, 278, 267, 256, 244, 232, 220, 208, 196, 183, 171, 158, 145, 132, 119, 106, 93, 80, 67, 53, 40, 27, 13, 0, -13, -27, -40, -53, -67, -80, -93, -106, -119, -132, -145, -158, -171, -183, -196, -208, -220, -232, -244, -255, -267, -278, -289, -300, -311, -322, -332, -342, -352, -361, -371, -380, -389, -397, -405, -413, -421, -429, -436, -443, -449, -455, -461, -467, -472, -477, -482, -486, -490, -494, -497, -500, -502, -505, -507, -508, -509, -510, -511, -511, -511, -510, -509, -508, -507, -505, -502, -500, -497, -494, -490, -486, -482, -477, -472, -467, -461, -455, -449, -443, -436, -429, -421, -413, -405, -397, -389, -380, -371, -361, -352, -342, -332, -322, -311, -300, -289, -278, -267, -256, -244, -232, -220, -208, -196, -183, -171, -158, -145, -132, -119, -106, -93, -80, -67, -53, -40, -27, -13,
//    0, 0, 13, 13, 27, 27, 40, 40, 53, 53, 67, 67, 80, 80, 93, 93, 106, 106, 119, 119, 132, 132, 145, 145, 158, 158, 171, 171, 183, 183, 196, 196, 208, 208, 220, 220, 232, 232, 244, 244, 255, 255, 267, 267, 278, 278, 289, 289, 300, 300, 311, 311, 322, 322, 332, 332, 342, 342, 352, 352, 361, 361, 371, 371, 380, 380, 389, 389, 397, 397, 405, 405, 413, 413, 421, 421, 429, 429, 436, 436, 443, 443, 449, 449, 455, 455, 461, 461, 467, 467, 472, 472, 477, 477, 482, 482, 486, 486, 490, 490, 494, 494, 497, 497, 500, 500, 502, 502, 505, 505, 507, 507, 508, 508, 509, 509, 510, 510, 511, 511, 511, 511, 511, 511, 510, 510, 509, 509, 508, 508, 507, 507, 505, 505, 502, 502, 500, 500, 497, 497, 494, 494, 490, 490, 486, 486, 482, 482, 477, 477, 472, 472, 467, 467, 461, 461, 455, 455, 449, 449, 443, 443, 436, 436, 429, 429, 421, 421, 413, 413, 405, 405, 397, 397, 389, 389, 380, 380, 371, 371, 361, 361, 352, 352, 342, 342, 332, 332, 322, 322, 311, 311, 300, 300, 289, 289, 278, 278, 267, 267, 256, 256, 244, 244, 232, 232, 220, 220, 208, 208, 196, 196, 183, 183, 171, 171, 158, 158, 145, 145, 132, 132, 119, 119, 106, 106, 93, 93, 80, 80, 67, 67, 53, 53, 40, 40, 27, 27, 13, 13, 0, 0, -13, -13, -27, -27, -40, -40, -53, -53, -67, -67, -80, -80, -93, -93, -106, -106, -119, -119, -132, -132, -145, -145, -158, -158, -171, -171, -183, -183, -196, -196, -208, -208, -220, -220, -232, -232, -244, -244, -255, -255, -267, -267, -278, -278, -289, -289, -300, -300, -311, -311, -322, -322, -332, -332, -342, -342, -352, -352, -361, -361, -371, -371, -380, -380, -389, -389, -397, -397, -405, -405, -413, -413, -421, -421, -429, -429, -436, -436, -443, -443, -449, -449, -455, -455, -461, -461, -467, -467, -472, -472, -477, -477, -482, -482, -486, -486, -490, -490, -494, -494, -497, -497, -500, -500, -502, -502, -505, -505, -507, -507, -508, -508, -509, -509, -510, -510, -511, -511, -511, -511, -511, -511, -510, -510, -509, -509, -508, -508, -507, -507, -505, -505, -502, -502, -500, -500, -497, -497, -494, -494, -490, -490, -486, -486, -482, -482, -477, -477, -472, -472, -467, -467, -461, -461, -455, -455, -449, -449, -443, -443, -436, -436, -429, -429, -421, -421, -413, -413, -405, -405, -397, -397, -389, -389, -380, -380, -371, -371, -361, -361, -352, -352, -342, -342, -332, -332, -322, -322, -311, -311, -300, -300, -289, -289, -278, -278, -267, -267, -256, -256, -244, -244, -232, -232, -220, -220, -208, -208, -196, -196, -183, -183, -171, -171, -158, -158, -145, -145, -132, -132, -119, -119, -106, -106, -93, -93, -80, -80, -67, -67, -53, -53, -40, -40, -27, -27, -13, -13,
	0, 0, 27, 27, 53, 53, 80, 80, 106, 106, 132, 132, 158, 158, 183, 183, 208, 208, 232, 232, 255, 255, 278, 278, 300, 300, 322, 322, 342, 342, 361, 361, 380, 380, 397, 397, 413, 413, 429, 429, 443, 443, 455, 455, 467, 467, 477, 477, 486, 486, 494, 494, 500, 500, 505, 505, 508, 508, 510, 510, 511, 511, 510, 510, 508, 508, 505, 505, 500, 500, 494, 494, 486, 486, 477, 477, 467, 467, 455, 455, 443, 443, 429, 429, 413, 413, 397, 397, 380, 380, 361, 361, 342, 342, 322, 322, 300, 300, 278, 278, 256, 256, 232, 232, 208, 208, 183, 183, 158, 158, 132, 132, 106, 106, 80, 80, 53, 53, 27, 27, 0, 0, -27, -27, -53, -53, -80, -80, -106, -106, -132, -132, -158, -158, -183, -183, -208, -208, -232, -232, -255, -255, -278, -278, -300, -300, -322, -322, -342, -342, -361, -361, -380, -380, -397, -397, -413, -413, -429, -429, -443, -443, -455, -455, -467, -467, -477, -477, -486, -486, -494, -494, -500, -500, -505, -505, -508, -508, -510, -510, -511, -511, -510, -510, -508, -508, -505, -505, -500, -500, -494, -494, -486, -486, -477, -477, -467, -467, -455, -455, -443, -443, -429, -429, -413, -413, -397, -397, -380, -380, -361, -361, -342, -342, -322, -322, -300, -300, -278, -278, -256, -256, -232, -232, -208, -208, -183, -183, -158, -158, -132, -132, -106, -106, -80, -80, -53, -53, -27, -27,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_ETH_GetReceivedFrame(&heth) == HAL_OK)
    {
        HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_SET);
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
        HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);
    }

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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2S
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG1_Pin|DEBUG2_Pin|DEBUG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG4_GPIO_Port, DEBUG4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG1_Pin DEBUG2_Pin DEBUG3_Pin */
  GPIO_InitStruct.Pin = DEBUG1_Pin|DEBUG2_Pin|DEBUG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG4_Pin */
  GPIO_InitStruct.Pin = DEBUG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
