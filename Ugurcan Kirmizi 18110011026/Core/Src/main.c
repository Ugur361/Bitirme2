/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "lcd_i2cModule.h"
#include <stdbool.h>

//#include <string.h>
//#include <stdio.h>
//#include <cstring>
//#include <string>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

		uint16_t readValue1;
		uint16_t readValue2;
	
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define VREFIN_Cal ((uint16_t*)((uint32_t)0x1FFF7A2A))  //Vrefin hafizadaki kalibrasyon degeri
#define button1_value HAL_GPIO_ReadPin(GPIOB,button1_Pin)
#define button2_value HAL_GPIO_ReadPin(GPIOB,button2_Pin)
#define button3_value HAL_GPIO_ReadPin(GPIOB,button3_Pin)
#define buttonhome_value HAL_GPIO_ReadPin(GPIOB,buttonhome_Pin)
#define Avg_slope 0.0025
#define V25 0.76
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_2
#define button5_value HAL_GPIO_ReadPin(GPIOE,button5_Pin)		
#define button6_value HAL_GPIO_ReadPin(GPIOE,button6_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
//void delay(uint16_t time){

//__HAL_TIM_SET_COUNTER(&htim4,0);						//timer 4 ü 0 dan baslattik. Timer 4 degeri, girecegimiz timer degiskeni suresi kadar
//																						// dongude kaliyor.Timer degiskenine ulastigi zaman delay islemini tamamlamis oluyoruz.					
//	while(__HAL_TIM_GET_COUNTER(&htim4)<time);



//}

//void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
//																						// pinlerin atamalarini yaptik.
//GPIO_InitTypeDef DHT11_DATA={0};
//DHT11_DATA.Pin=GPIO_Pin;
//DHT11_DATA.Mode=GPIO_MODE_OUTPUT_PP;
//DHT11_DATA.Pull=GPIO_NOPULL;
//DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;
//HAL_GPIO_Init(GPIOx,&DHT11_DATA);




//}
//void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

//GPIO_InitTypeDef DHT11_DATA={0};
//DHT11_DATA.Pin=GPIO_Pin;
//DHT11_DATA.Mode=GPIO_MODE_INPUT;
//DHT11_DATA.Pull=GPIO_NOPULL;
//DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

//HAL_GPIO_Init(GPIOx,&DHT11_DATA);




//}
//float Humidity=0.0,Temperature=0;
//uint8_t durum=0;
//uint16_t tempVal=0,humVal=0;
//uint8_t dhtVal[2];
//uint8_t mData[40];
//uint16_t mTime1=0,mTime2=0;
//uint16_t mbit=0;
//uint8_t parityVal=0,genParity=0;
	
//uint8_t DHT11_Read(void){
//for(int a=0;a<40;a++)mData[a]=0;
//	mTime1=0,mTime2=0,durum=0,tempVal=0,humVal=0,parityVal=0,genParity=0,mbit=0;
//	Set_Pin_Output(DHT11_PORT,DHT11_PIN);
//	HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);
//	delay(18000);
//	Set_Pin_Input(DHT11_PORT,DHT11_PIN);
//	
//	__HAL_TIM_SET_COUNTER(&htim4,0);
//	while(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)==GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim4)>500)return 0; 
//	
//	__HAL_TIM_SET_COUNTER(&htim4,0);
//	while(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)==GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim4)>500)return 0; 

//	__HAL_TIM_SET_COUNTER(&htim4,0);
//	while(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)==GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim4)>500)return 0; 
//	
//	mTime2=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
//	
//	if(mTime1<75&& mTime1>85&&mTime2<75&&mTime2>85){			//80 mikrosaniyeye cok yakin degerler oldugu icin islemcimiz bunu goremeyebilir.
//																												//eger bu degerlerin altinda veya ustunde ise yine return 0 dondurecektir.
//	return 0;
//	}
//	
//	for(int j=0;j<40;j++){
//	
//	
//	__HAL_TIM_SET_COUNTER(&htim4,0);
//		while(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)==GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim4)>500)return 0;
//	
//	__HAL_TIM_SET_COUNTER(&htim4,0);
//	while(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)==GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim4)>500)return 0; 
//		mTime1=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
//		
//		//gecis suresini kontrol ediyoruz.Eger zaman 25 us ise low olarak ayarliyoruz
//		if(mTime1>20&&mTime1<30){
//		mbit=0;
//		
//		
//			
//		}
//		//eger sure 70 us ise high olarak ayarliyoruz.
//		else if(mTime1>60&&mTime1<80){
//		
//		mbit=1;
//		}
//		mData[j]=mbit; //j deki bilgiyi, data bufferina atiyoruz.
//		
//	}
//	for(int i=0;i<8;i++){		//data bufferindan nem degerini aliyoruz.
//	
//	humVal+=mData[i];
//		humVal=humVal<<1;
//	
//	}
//	for(int i=16;i<24;i++){		//data bufferindan sicaklik degerini aliyoruz.
//	
//	tempVal+=mData[i];
//		tempVal=tempVal<<1;
//	
//	}
//	for(int i=32;i<40;i++){		//data bufferindan esitlik biti degerini aliyoruz.
//	
//	parityVal+=mData[i];
//		parityVal=parityVal<<1;
//	
//	}
//	parityVal=parityVal>>1;
//	humVal=humVal>>1;
//	tempVal=tempVal>>1;
//	
//	genParity=humVal+tempVal;
//	
//	dhtVal[0]=tempVal;
//	dhtVal[1]=humVal;
//	
//	return 1;
//	

//}		
		float analog_deger;
		uint16_t tempreadValue;
		float tCelsius;
		uint16_t dijital_deger;
		uint16_t sayac=0;
		
		uint16_t buton1basildi;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void basilanButon(uint16_t sayi);
	

void ledSure(){
			
			LCD_SetCursor(1,1);	
			LCD_Send_String("3 sn icin 5",STR_NOSLIDE);
			LCD_SetCursor(2,1);
			LCD_Send_String(" 8 sn icin 6",STR_NOSLIDE);
			
			if(HAL_GPIO_ReadPin(GPIOE,button5_Pin)==1){
				LCD_Clear();
			HAL_Delay(100);
			LCD_Send_String("3 saniye yaniyor",STR_NOSLIDE);
				HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_SET);
				HAL_Delay(3000);
				HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_RESET);
			LCD_SetCursor(2,1);
			LCD_Send_String(" Geri icin 4",STR_NOSLIDE);
			
			
				
			}
			else if(HAL_GPIO_ReadPin(GPIOE,button6_Pin)==1){
				LCD_Clear();
			HAL_Delay(100);
				LCD_Send_String("8 saniye yaniyor",STR_NOSLIDE);
				HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_SET);
				HAL_Delay(8000);			
				HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_RESET);
		
			LCD_SetCursor(2,1);
			LCD_Send_String(" Geri icin 4",STR_NOSLIDE);
			
			
				
			
			}
		}
//HAL_Delay(70000);
//			LCD_Clear();
//			HAL_Delay(20);

//}


void Batt_Level(double voltx)
{	
	  static int k=0;
	
	  //Vref=2.94 , voltage level=5 , step=2.94/5=0.588
		if(voltx >= 0.1 && voltx <= 0.66)       k = 1; //Batt. level 1		      
		else if(voltx > 0.66 && voltx <= 1.32) k = 2; //Batt. level 2      	
	  else if(voltx > 1.32 && voltx <= 1.98) k = 3; //Batt. level 3      
    else if(voltx > 1.98 && voltx <= 2.64) k = 4; //Batt. level 4	    
    else if(voltx > 2.64 && voltx <= 3.30) k = 5; //Batt. level 5 - Batt. Full		  
	  else
		{LCD_Write_Data(0x20);k=0;}	//Batt. level 0 - Batt. Empty	

    for(int i=1;i<=k;i++)
	LCD_Write_Data(0xFF);
}

void Read_ADC(){

	HAL_ADC_Start(&hadc1);

	if(HAL_ADC_PollForConversion(&hadc1,10000)==HAL_OK){
		dijital_deger=HAL_ADC_GetValue(&hadc1);
		analog_deger=dijital_deger*0.0008056640625;
	HAL_ADC_Stop(&hadc1);
	}


}
void Read_Temp(){

	HAL_ADC_Start(&hadc1);

	if(HAL_ADC_PollForConversion(&hadc1,10000)==HAL_OK){
		tempreadValue=HAL_ADC_GetValue(&hadc1);
		tCelsius= 0.02488 *tempreadValue;
		if(tCelsius<=34){
		HAL_GPIO_WritePin(GPIOE,temp_ledgreen_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,temp_ledred_Pin|temp_ledyellow_Pin, GPIO_PIN_RESET);
			HAL_Delay(20);
		}
		if(34<tCelsius&&tCelsius<=68){
		HAL_GPIO_WritePin(GPIOE,temp_ledyellow_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,temp_ledred_Pin, GPIO_PIN_RESET);
			HAL_Delay(20);
		}
		if(tCelsius>68){
		HAL_GPIO_WritePin(GPIOE,temp_ledred_Pin,GPIO_PIN_SET);
			HAL_Delay(20);
		}
	HAL_ADC_Stop(&hadc1);
	}


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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
LCD_i2cDeviceCheck();
		LCD_Init();
		LCD_BackLight(LCD_BL_ON);
		LCD_SetCursor(1,1);
//		HAL_ADC_Start(&hadc1);
		
		
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
	
		
		
		basilanButon(sayac);
		Read_ADC();

		if(buttonhome_value==1){
			
			sayac=0;

			basilanButon(sayac);
			}
	else		if(button1_value==1){
		
			sayac=1;

			basilanButon(sayac);
		
			
		
		}
	else	if(button2_value==1){
			sayac=2;

			
			basilanButon(sayac);
		}
	else	if(button3_value==1){
		
				sayac=3;

			
			basilanButon(sayac);
		
		}

		
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
	}
		
	}
  
  /* USER CODE END 3 */


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, temp_ledgreen_Pin|temp_ledyellow_Pin|temp_ledred_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_red_GPIO_Port, led_red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : button5_Pin button6_Pin */
  GPIO_InitStruct.Pin = button5_Pin|button6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : temp_ledgreen_Pin temp_ledyellow_Pin temp_ledred_Pin */
  GPIO_InitStruct.Pin = temp_ledgreen_Pin|temp_ledyellow_Pin|temp_ledred_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : led_red_Pin */
  GPIO_InitStruct.Pin = led_red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_red_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : button1_Pin button2_Pin button3_Pin buttonhome_Pin */
  GPIO_InitStruct.Pin = button1_Pin|button2_Pin|button3_Pin|buttonhome_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */





void basilanButon(uint16_t sayi){
	
	
switch(sayi){
	case 0: 
			
//	HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_RESET);
		LCD_SetCursor(1,1);
		LCD_Send_String("KSU EEM MENU APP",STR_NOSLIDE);
		LCD_SetCursor(2,1);
		LCD_Send_String("LED:1 BATARYA:2 SICAKLIK:3 ",STR_SLIDE);
HAL_Delay(700);
			LCD_Clear();
	
	break;
	case 1: 	ledSure();
			
	break;
		
	case 2: 
		
	HAL_Delay(700);
			LCD_Clear();
			HAL_Delay(20);
			
			LCD_SetCursor(1,1);
			
			LCD_Print("V1:%.2f V",analog_deger);
			Batt_Level(analog_deger);
			HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_RESET);
	
			LCD_SetCursor(2,1);
			LCD_Send_String("Geri icin 4 ",STR_NOSLIDE);			
break;
			case 3: 
				Read_Temp();
				HAL_Delay(700);
			LCD_Clear();
			HAL_Delay(20);
			
			LCD_SetCursor(1,1);
			
			LCD_Print("Sicaklik:%.f ",tCelsius);
			LCD_Send_String("C",STR_NOSLIDE);
			
			HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_RESET);
	
			LCD_SetCursor(2,1);
			LCD_Print("Geri 4 ",STR_NOSLIDE);			

break;
			case 4:
				HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_SET);
				HAL_Delay(3000);
				HAL_GPIO_WritePin(GPIOB,led_red_Pin,GPIO_PIN_RESET);
				LCD_Send_String("LED 5 SN YANACAK",STR_NOSLIDE);
				LCD_SetCursor(2,1);
				LCD_Send_String("Geri icin 4 ",STR_NOSLIDE);			
			
			HAL_Delay(700);
			LCD_Clear();

//	
	
	break;
	
		
	}
	
	
	
	
	
	
	
	
	
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
