/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <m17.h>
#include <pccmd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DAC_IDLE		2048
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//hardware related stuff
typedef enum
{
	POT_AUD_OUT,
	POT_AUD_IN,
	POT_BSB_OUT,
	POT_BSB_IN
} pot_t;

typedef enum
{
	PTT_INACTIVE,
	PTT_ACTIVE
} ptt_t;

typedef enum
{
	ST_RX,
	ST_TX
} state_t;

uint16_t data[30*SYM_PER_FRA*10+1]={DAC_IDLE};	//one additional sample to set the idle voltage
volatile ptt_t ptt_in=PTT_INACTIVE;				//ptt state
state_t state=ST_RX;							//radio state
volatile uint8_t tx_done=0;						//baseband playback complete?
uint16_t total_samples=0;

//input data
struct settings_t
{
	char dst_raw[10];						//raw, unencoded destination address
	char src_raw[10];						//raw, unencoded source address
	uint8_t can;							//Channel Access Number
	char msg[798];							//text message (32*25-2 bytes)
	uint8_t phase;							//baseband phase 1-normal, 0-inverted
} settings;

//M17 stuff
lsf_t lsf;									//Link Setup Frame data

uint32_t pkt_sym_cnt=0;
uint16_t num_bytes=0;						//size of payload in bytes

uint8_t rf_bits[SYM_PER_PLD*2];				//type-4 bits for transmission
int8_t symbols[SYM_PER_FRA];				//frame symbols
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setWiper(uint8_t pot, uint16_t val)
{
	switch(pot)
	{
		case 0:
			HAL_I2C_Mem_Write(&hi2c1, 0x58, (0x00<<4)|(val>>8), 1, (uint8_t*)&val, 1, 100);
		break;

		case 1:
			HAL_I2C_Mem_Write(&hi2c1, 0x58, (0x01<<4)|(val>>8), 1, (uint8_t*)&val, 1, 100);
		break;

		case 2:
			HAL_I2C_Mem_Write(&hi2c1, 0x58, (0x06<<4)|(val>>8), 1, (uint8_t*)&val, 1, 100);
		break;

		case 3:
			HAL_I2C_Mem_Write(&hi2c1, 0x58, (0x07<<4)|(val>>8), 1, (uint8_t*)&val, 1, 100);
		break;
	}
}

//M17 - int8_t based functions (as opposed to libm17's float)
void send_preamble_i(int8_t out[SYM_PER_FRA], uint32_t *cnt)
{
	//only pre-LSF is supported
    for(uint16_t i=0; i<SYM_PER_FRA/2; i++) //40ms * 4800 = 192
    {
        out[(*cnt)++]=+3;
        out[(*cnt)++]=-3;
    }
}

void send_syncword_i(int8_t out[SYM_PER_SWD], uint32_t *cnt, const uint16_t syncword)
{
    for(uint8_t i=0; i<SYM_PER_SWD*2; i+=2)
    {
        out[(*cnt)++]=symbol_map[(syncword>>(14-i))&3];
    }
}

void send_data_i(int8_t out[SYM_PER_PLD], uint32_t *cnt, const uint8_t* in)
{
    for(uint16_t i=0; i<SYM_PER_PLD; i++) //40ms * 4800 - 8 (syncword)
    {
        out[(*cnt)++]=symbol_map[in[2*i]*2+in[2*i+1]];
    }
}

void send_eot_i(int8_t out[SYM_PER_FRA], uint32_t *cnt)
{
    for(uint16_t i=0; i<SYM_PER_FRA; i++) //40ms * 4800 = 192
    {
        out[(*cnt)++]=eot_symbols[i%8];
    }
}

void send_frame_i(int8_t out[SYM_PER_FRA], const uint8_t* data, const frame_t type, const lsf_t* lsf)
{
    uint8_t enc_bits[SYM_PER_PLD*2];    //type-2 bits, unpacked
    uint8_t rf_bits[SYM_PER_PLD*2];     //type-4 bits, unpacked
    uint32_t sym_cnt=0;                 //symbols written counter

    if(type==FRAME_LSF)
    {
        send_syncword_i(out, &sym_cnt, SYNC_LSF);
        conv_encode_LSF(enc_bits, lsf);
    }
    else if(type==FRAME_PKT)
    {
		(void)lsf;
        send_syncword_i(out, &sym_cnt, SYNC_PKT);
        conv_encode_packet_frame(enc_bits, data); //packet frames require 200-bit payload chunks plus a 6-bit counter
    }

    //common stuff
    reorder_bits(rf_bits, enc_bits);
    randomize_bits(rf_bits);
    send_data_i(out, &sym_cnt, rf_bits);
}

//filter symbols, flt is assumed to be 41 taps long
void filter_symbols(uint16_t* out, const int8_t* in, const float* flt, uint8_t phase_inv)
{
	static float last[81]; //memory for last samples

	for(uint8_t i=0; i<SYM_PER_FRA; i++)
	{
		for(uint8_t j=0; j<10; j++)
		{
			for(uint8_t k=0; k<80; k++)
				last[k]=last[k+1];

			if(j==0)
			{
				if(phase_inv)
					last[80]= in[i];
				else
					last[80]=-in[i];
			}
			else
				last[80]=0.0f;

			float acc=0.0f;
			for(uint8_t k=0; k<81; k++)
				acc+=last[k]*flt[k];

			if(out!=NULL) out[i*10+j]=sqrtf(10.0f)*(7168.0f/32768.0f*2048.0f)*acc+DAC_IDLE;
		}
	}
}

//generate baseband samples TODO: add more args here (LSF etc.)
void generate_baseband(char *msg, uint8_t phase_inv)
{
	uint8_t fn=0;
	uint8_t full_packet_data[32*25]={0};
	uint8_t packet_frame[26]={0};
	uint16_t num_bytes=0;

	//obtain data and append with CRC
	memset(full_packet_data, 0, sizeof(full_packet_data));
	full_packet_data[0]=0x05;
	num_bytes=sprintf((char*)&full_packet_data[1], msg)+2; //prepended 0x05 and appended 0x00
	uint16_t packet_crc=CRC_M17(full_packet_data, num_bytes);
	full_packet_data[num_bytes]  =packet_crc>>8;
	full_packet_data[num_bytes+1]=packet_crc&0xFF;
	num_bytes+=2; //count in 2-byte CRC too

	//flush the RRC filter
	int8_t flush[SYM_PER_FRA]={0.0f};
	filter_symbols(NULL, flush, rrc_taps_10, phase_inv);

	//generate preamble
	pkt_sym_cnt=0;
	send_preamble_i(symbols, &pkt_sym_cnt);
	filter_symbols(&data[0*SYM_PER_FRA*10], symbols, rrc_taps_10, phase_inv);

	//generate LSF
	send_frame_i(symbols, NULL, FRAME_LSF, &lsf);
	filter_symbols(&data[1*SYM_PER_FRA*10], symbols, rrc_taps_10, phase_inv);

	//generate frames
	while(num_bytes>25)
	{
		memcpy(packet_frame, &full_packet_data[fn*25], 25);
		packet_frame[25]=fn<<2;
		send_frame_i(symbols, packet_frame, FRAME_PKT, NULL);
		filter_symbols(&data[(2+fn)*SYM_PER_FRA*10], symbols, rrc_taps_10, phase_inv);
		fn++;
		num_bytes-=25;
	}

	memset(packet_frame, 0, sizeof(packet_frame));
	memcpy(packet_frame, &full_packet_data[fn*25], num_bytes);
	packet_frame[25]=0x80|(num_bytes<<2);
	send_frame_i(symbols, packet_frame, FRAME_PKT, NULL);
	filter_symbols(&data[(2+fn)*SYM_PER_FRA*10], symbols, rrc_taps_10, phase_inv);

	//generate EOT
	pkt_sym_cnt=0;
	send_eot_i(symbols, &pkt_sym_cnt);
	filter_symbols(&data[(3+fn)*SYM_PER_FRA*10], symbols, rrc_taps_10, phase_inv);

	//settle samples at mid-scale
	filter_symbols(&data[(4+fn)*SYM_PER_FRA*10], flush, rrc_taps_10, phase_inv);

	total_samples=(4+fn)*SYM_PER_FRA*10 + 80; //it is enough to make sure the output voltage will settle
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==PTT_IN_Pin)
	{
		if(HAL_GPIO_ReadPin(PTT_IN_GPIO_Port, PTT_IN_Pin))
		{
			ptt_in=PTT_ACTIVE;
		}
		else
		{
			ptt_in=PTT_INACTIVE;
		}
	}
}

//baseband playback end
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	tx_done=1;
}

//AF playback end
void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
	//muting doesn't work
	//HAL_GPIO_WritePin(AFOUT_MUTE_GPIO_Port, AFOUT_MUTE_Pin, 0);
}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(250);

  setWiper(POT_BSB_IN,  0x100); HAL_Delay(10);
  setWiper(POT_BSB_OUT, 0x0E0); HAL_Delay(10); //might need some more trimming
  setWiper(POT_AUD_IN,  0x080); HAL_Delay(10);
  setWiper(POT_AUD_OUT, 0x080); HAL_Delay(10);

  //set idle voltages
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)data, 1, DAC_ALIGN_12B_R);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)data, 1, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim7); //48kHz clock
  HAL_Delay(4750);
  HAL_TIM_Base_Stop(&htim7); //48kHz clock
  tx_done=0;

  if(HAL_DAC_Start(&hdac1, DAC_CHANNEL_1)+HAL_DAC_Start(&hdac1, DAC_CHANNEL_2)==HAL_OK)
  {
	  pcCmdWriteDisplay(&huart1, "Micro17 OK", "");
  }
  else
  {
	  pcCmdWriteDisplay(&huart1, "DAC ERR", "");
	  while(1);
  }

  //input data
  sprintf(settings.msg, "Борітеся - поборете.");
  sprintf(settings.dst_raw, "ALL");
  sprintf(settings.src_raw, "N0CALL");
  settings.phase=0; //inverted

  //encode dst, src for the lsf struct
  uint64_t dst_enc=0, src_enc=0;
  uint16_t type=0;
  encode_callsign_value(&dst_enc, (uint8_t*)settings.dst_raw);
  encode_callsign_value(&src_enc, (uint8_t*)settings.src_raw);
  for(int8_t i=5; i>=0; i--)
  {
	  lsf.dst[5-i]=(dst_enc>>(i*8))&0xFF;
	  lsf.src[5-i]=(src_enc>>(i*8))&0xFF;
  }

  type=((uint16_t)0x01<<1)|((uint16_t)settings.can<<7); //packet mode, content: data
  lsf.type[0]=(uint16_t)type>>8;
  lsf.type[1]=(uint16_t)type&0xFF;
  memset(&lsf.meta, 0, 112/8);

  //calculate LSF CRC
  uint16_t lsf_crc=LSF_CRC(&lsf);
  lsf.crc[0]=lsf_crc>>8;
  lsf.crc[1]=lsf_crc&0xFF;

  generate_baseband(settings.msg, settings.phase);

  pcCmdWriteDisplay(&huart1, "Micro17 OK", "BSB OK");

  /*while(1)
  {
	  HAL_GPIO_WritePin(AFOUT_MUTE_GPIO_Port, AFOUT_MUTE_Pin, 1);
	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)data, 1920+1, DAC_ALIGN_12B_R);
	  HAL_Delay(40);
	  HAL_GPIO_WritePin(AFOUT_MUTE_GPIO_Port, AFOUT_MUTE_Pin, 0);
	  HAL_Delay(1000-40);
  }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //PTT down
	  if(ptt_in==PTT_ACTIVE && state==ST_RX)
	  {
		  HAL_GPIO_WritePin(PTT_OUT_GPIO_Port, PTT_OUT_Pin, 1);
		  HAL_Delay(60);

		  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)data, total_samples, DAC_ALIGN_12B_R);
		  HAL_TIM_Base_Start(&htim7); //48kHz clock

		  state=ST_TX;
	  }

	  //PTT up
	  /*if(ptt_in==PTT_INACTIVE && state==ST_TX)
	  {
		  HAL_Delay(60);
		  HAL_GPIO_WritePin(PTT_OUT_GPIO_Port, PTT_OUT_Pin, 0);

		  state=ST_RX;
	  }*/

	  //stop TX - no more baseband
	  if(tx_done && state==ST_TX)
	  {
		  HAL_TIM_Base_Stop(&htim7); //48kHz clock
		  tx_done=0;
		  HAL_Delay(40);
		  HAL_GPIO_WritePin(PTT_OUT_GPIO_Port, PTT_OUT_Pin, 0);
		  ptt_in=PTT_INACTIVE;
		  state=ST_RX;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_160MHZ;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x50916E9F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 3499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMAMUX_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PTT_OUT_GPIO_Port, PTT_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AFOUT_MUTE_GPIO_Port, AFOUT_MUTE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PTT_IN_Pin */
  GPIO_InitStruct.Pin = PTT_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PTT_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PTT_OUT_Pin */
  GPIO_InitStruct.Pin = PTT_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(PTT_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AFOUT_MUTE_Pin */
  GPIO_InitStruct.Pin = AFOUT_MUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AFOUT_MUTE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
