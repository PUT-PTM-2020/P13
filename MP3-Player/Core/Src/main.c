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
#include "string.h"
#include <stdio.h>
#include"ff.h"
#include "lcd.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFSIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static FATFS FatFs;    //uchwyt do urządzenia FatFs (dysku, karty SD...)
FRESULT fresult;  //do przechowywania wyniku operacji na bibliotece


volatile FIL file;
WORD bytes_written;

WORD bytes_read;           //liczba odczytanych byte



uint8_t receiveUART[1];
uint16_t sizeReceiveUART = 1;
int i=0;
int j=-1;
uint8_t indeks_glosnosci = 4;
double glosnosc_guziczki [10] = {0,0.25,0.5,1,2,4,8,10,15,20};
uint16_t value[1];
uint8_t sizeutwor;
uint8_t bluecon;
uint16_t potencjometr;

double glosnosc;
double glosnosc2;
uint8_t zrodlo_gloscnoci=0;

DIR dir;

uint8_t eof;
char utwor[20];
int stan = 1; //0 pauza 1 start

uint16_t nr_utworu=0;

volatile uint8_t buf[BUFSIZE];
volatile uint8_t buf2[BUFSIZE];
uint8_t aktualny_bufor = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */

void read_song(){

FRESULT res;
    DIR dir;
    UINT i=0;
    UINT z;

    static FILINFO fno;


		res = f_opendir(&dir, "/");
    	if (res == FR_OK) {


  	  	  	do{
            		res = f_readdir(&dir, &fno);
            		if (res != FR_OK || fno.fname[0] == 0) {i=1; break;}
            		printf("%s\n", fno.fname);
                	z = strlen(fno.fname);
                	i++;
            	}
            	while(i<=nr_utworu ||(fno.fname[z-1]!='V') || (fno.fname[z-2]!='A')|| (fno.fname[z-3]!='W'));
  	  	  	//||
  	  	  	//(fno.fname[z-1]!='3') || (fno.fname[z-2]!='P')|| (fno.fname[z-3]!='M')
  	  	  		sprintf(utwor,"%s",fno.fname);
  	  	  		nr_utworu=i-1;
  	  	  		if(nr_utworu==0)read_song();
            	}


    			sizeutwor = strlen(utwor);
               	return;
}

void next(){
	HAL_TIM_Base_Stop_IT(&htim4);
	f_close(&file);
	nr_utworu++;
	read_song();
	fresult = f_open(&file, &utwor , FA_READ|FA_OPEN_EXISTING);
	f_read(&file, &buf, BUFSIZE, &bytes_read);
	i=0;
	j=0;
	 lcd_clear ();
	lcd_put_cur(0, 0);
	lcd_send_string(&utwor);
	lcd_put_cur(1, 0);
	lcd_send_string("PLAY");
	if(bluecon==1)
	HAL_UART_Transmit_IT(&huart2, &utwor, sizeutwor);

	 HAL_TIM_Base_Start_IT(&htim4);
}

void prev(){
	HAL_TIM_Base_Stop_IT(&htim4);
	f_close(&file);
	nr_utworu--;
	read_song();
	fresult = f_open(&file, &utwor , FA_READ|FA_OPEN_EXISTING);
	f_read(&file, &buf, BUFSIZE, &bytes_read);
	i=0;
	j=0;
	 lcd_clear ();
	lcd_put_cur(0, 0);
	lcd_send_string(&utwor);
	lcd_put_cur(1, 0);
	lcd_send_string("PLAY");
	if(bluecon==1)
	HAL_UART_Transmit_IT(&huart2, &utwor, sizeutwor);

	HAL_TIM_Base_Start_IT(&htim4);
}
/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == hadc1.Instance){
		  	  if(value[0]>0 && value[0] <= 410) indeks_glosnosci = 0;
		  	  else if(value[0]>410 && value[0] <= 819) indeks_glosnosci = 1;
		  	  else if(value[0]>819 && value[0] <= 1228) indeks_glosnosci = 2;
		  	else if(value[0]>1228 && value[0] <= 1647) indeks_glosnosci = 3;
		  	else if(value[0]>1647 && value[0] <= 2058) indeks_glosnosci = 4;
		  	else if(value[0]>2058 && value[0] <= 2467) indeks_glosnosci = 5;
		  	else if(value[0]>2467 && value[0] <= 2876) indeks_glosnosci = 6;
		  	else if(value[0]>2876 && value[0] <= 3285) indeks_glosnosci = 7;
		  	else if(value[0]>3285 && value[0] <= 3692) indeks_glosnosci = 8;
		  	else if(value[0]>3692 && value[0] <= 4095) indeks_glosnosci = 9;
	}
}*/


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	if(hadc->Instance == hadc1.Instance){

		if(abs( potencjometr - value[0] ) > 10){
			zrodlo_gloscnoci=1;
		}
		potencjometr = value[0];

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET){

		 //ciszej

		 if(indeks_glosnosci>0 && indeks_glosnosci<=9) {indeks_glosnosci--; zrodlo_gloscnoci=0;}

		  	}

	 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET){
			  		//prev song
		prev();


			}
	 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET){


		 //pause/start

		 if(stan==1){
		//HAL_TIM_Base_Start(&htim6);
		//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, &buf, BUFSIZE, DAC_ALIGN_12B_R);
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string(&utwor);
		lcd_put_cur(1, 0);
		lcd_send_string("PLAY");

		HAL_TIM_Base_Start_IT(&htim4);
		 //HAL_TIM_Base_Start_IT(&htim7);
		 stan = 0;
		 }
		 else
		 {
			 lcd_clear();
			 lcd_put_cur(1, 0);
			 lcd_send_string("PAUSE");
			 //HAL_TIM_Base_Start(&htim6);
			HAL_TIM_Base_Stop_IT(&htim4);
			 stan=1;
		 }

	 		}
	 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET){
		 	next();

		 	}
	 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET){

		 //glosniej

		 if(indeks_glosnosci>=0 && indeks_glosnosci<9)
					 indeks_glosnosci++;
				 	 zrodlo_gloscnoci=0;
					 	}
	 HAL_Delay(200);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM4)
	{
		switch (zrodlo_gloscnoci){
				case 0:
					glosnosc= glosnosc_guziczki[indeks_glosnosci];
					break;
				case 1:
					glosnosc = (value[0]/500);
				//	if(glosnosc2!=gloscnosc){
				//	lcd_clear ();
				//	lcd_put_cur(0, 0);
				//	lcd_send_string("GŁOŚNOŚĆ ", &glosnosc);
					//}
				//	glosnosc2 = (value[0]/500);
					break;

				}

		if(aktualny_bufor==0){
					HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,buf[i]*glosnosc);
					//HAL_I2S_Transmit(&hi2s2, &buf[i], BUFSIZE,100);
					eof=f_eof(&file);
					if(eof ==0) f_read(&file, &buf2[i],1, &bytes_read);
					else {next();}
					i++;
					if(i==BUFSIZE){
						aktualny_bufor = 1;
						j=0;
						//HAL_TIM_Base_Start_IT(&htim7);*glosnosc_guziczki[indeks_glosnosci]
					}
				}

			if(aktualny_bufor==1){
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,buf2[j]*glosnosc);
				//HAL_I2S_Transmit(&hi2s2, &buf2[j], BUFSIZE,100);
				eof=f_eof(&file);
				if(eof ==0) f_read(&file, &buf[j],1, &bytes_read);
				else {next();}
				j++;
				if(j==BUFSIZE){
					aktualny_bufor = 0;
					i=0;
					//HAL_TIM_Base_Start_IT(&htim7);
				}
			}
		/*if(i<BUFSIZE){
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,buf[i]);
			i++;
			}
			else bufforek();*/

	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
	{
		if(huart->Instance == USART2)
			{

			HAL_UART_Receive_IT(&huart2, receiveUART, sizeReceiveUART);
			if(receiveUART[0]==72){
				bluecon = 1;
			}
			if(receiveUART[0]==65){
							HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
							prev();
						}
						if(receiveUART[0]==66){

							HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
							if(stan==1){
								HAL_UART_Transmit_IT(&huart2, &utwor, sizeutwor);
									lcd_clear();
									lcd_put_cur(0, 0);
									lcd_send_string(&utwor);
									lcd_put_cur(1, 0);
									lcd_send_string("PLAY");
									 HAL_TIM_Base_Start_IT(&htim4);
									 stan = 0;
									 }
									 else
									 {
										 lcd_clear();
										 lcd_put_cur(1, 0);
										 lcd_send_string("PAUSE");
										 HAL_TIM_Base_Stop_IT(&htim4);
										 stan=1;
									 }
									}
						if(receiveUART[0]==67){
							HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
										next();
									}
			}
	}
/*
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6)
		{
			i++;

			if(i>=BUFSIZE){
				HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				if(aktualny_bufor==0){
					f_read(&file, &buf2,BUFSIZE, &bytes_read);
					HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, &buf, BUFSIZE, DAC_ALIGN_12B_R);
				}
				else if(aktualny_bufor==1){
					f_read(&file, &buf,BUFSIZE, &bytes_read);
					HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, &buf2, BUFSIZE, DAC_ALIGN_12B_R);
				}
			}
		}
}*/
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  HAL_DAC_Start(&hdac,DAC_CHANNEL_2);

  	  	HAL_UART_Receive_IT(&huart2, receiveUART, sizeReceiveUART);
    	fresult = f_mount(&FatFs, "", 1);
    	read_song();
        fresult = f_open(&file, &utwor , FA_READ|FA_OPEN_EXISTING|FA_OPEN_ALWAYS);
        fresult = f_read(&file, &buf2, 352, &bytes_read);

        f_read(&file, &buf,BUFSIZE, &bytes_read);
        HAL_ADC_Start(&hadc1);
        potencjometr = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)value, 1);

        lcd_init();

        lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 // lcd_send_string("Hello");
	     //lcd_send_cmd(0x80 | 0x03);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 126;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 14;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 126;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 114;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB11 PB12 PB13 PB14 
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
