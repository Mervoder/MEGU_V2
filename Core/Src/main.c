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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "LSM6DSLTR.h"
#include "./BME280/bme280.h"
#include <math.h>
#include "FIR_FILTER.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EGU_RX_BUFFER_SIZE 5
#define EGU_TX_BUFFER_SIZE 34


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

typedef union{
  float fVal;
  unsigned char array[4];
}float2unit8;




uint8_t EGU_RX_BUFFER[EGU_RX_BUFFER_SIZE];
uint8_t EGU_TX_BUFFER[EGU_TX_BUFFER_SIZE];
uint8_t rx_data_EGU=0;
uint8_t rx_index_EGU=0;
static uint8_t EGU_durum_sorgusu[5]={0x54,0x52,0x35,0x0D,0x0A};
static uint8_t EGU_motor_atesleme[5]={0x54,0x52,0x32,0x0D,0x0A};



uint32_t page =0;
uint8_t offset =0;
uint8_t flash_flag =1;
uint8_t flash_timer =0;


float zaman=0 , zaman2=0;

uint8_t MEGU_battery=0;
uint8_t MEGU_mod=0;

uint8_t BUTTON_STATE=0;


uint8_t writeData[50] = {0,1,1,1};
uint8_t readData[50] = {0};


uint8_t fitil_kontrol=0;
uint8_t motor_ates=0;
uint8_t sensor_flag=0;
uint8_t rampa_control=0;
uint8_t ariza_BME=0;
uint8_t ariza_LSM=0;
uint8_t ariza=0;

const uint8_t egu_byte_0=0x54;
const uint8_t egu_byte_1=0x52;
const uint8_t egu_byte_28=0x00;
const uint8_t egu_byte_31=0x00;
const uint8_t egu_byte_32=0x0D;
const uint8_t egu_byte_33=0x0A;

float temperature=0;
float humidity=0;
float altitude=0;
float max_altitude=0;
float offset_altitude=0;
float pressure=0;
float alt=0;
float P0 = 1013.25;
float prev_alt=0;
float speed=0;

uint8_t altitude_rampa_control=0;
float speed_max , x_max , altitude_max =0;


enum ZORLU2024
{
	RAMPA,UCUS_BASLADI,KADEME_AYRILDIMI,AYRILDI
};
enum ZORLU2024 MEGU;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
int16_t BME_I2C_Testsensor(void);
int16_t LSM_I2C_Testsensor(void);
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
float BME280_Get_Altitude(void);
void bme_config(void);
void union_converter(void);
void EGU_Buff_Load(void);
void Buzzer(int how_many , uint32_t how_long);
void Altitude_Offset();
void Flash_toplu_temizleme(int page);
int compare_arrays(uint8_t *array1, uint8_t *array2, uint16_t size);
void MEGU_TX_BUF_FILL(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

LSM6DSLTR Lsm_Sensor;

FIRFilter accy;
FIRFilter accz;
FIRFilter accx;
FIRFilter IMU_GYROX;
FIRFilter IMU_GYROY;
FIRFilter IMU_GYROZ;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart == &huart6){
		if(rx_data_EGU != '\n' && rx_index_EGU <5){
			EGU_RX_BUFFER[rx_index_EGU++]=rx_data_EGU;

		}
		else
		{
			rx_data_EGU=0;
			rx_index_EGU=0;

		}
	HAL_UART_Receive_IT(&huart6, &rx_data_EGU, 1);
		}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{



	if(htim==&htim10){ //50ms
	sensor_flag=1;
	}


}
/**********************************************/

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
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_TIM10_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  ///KURTARMA PORTLARI KAPALI EMIN OL
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);//A
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);//B
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);//C
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);//D


  HAL_UART_Receive_IT(&huart6, &rx_data_EGU, 1);

  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim6);

  MAFilter_Init(&accx);
  FIRFilter_Init(&IMU_GYROY);
  FIRFilter_Init(&IMU_GYROX);
  FIRFilter_Init(&IMU_GYROZ);



  LSM6DSLTR_Init();
  bme_config();
  Altitude_Offset();

  Buzzer(10, 100);

  ariza_BME=BME_I2C_Testsensor();
  ariza_LSM=LSM_I2C_Testsensor();

  ariza=ariza_BME+ariza_LSM;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  MEGU_TX_BUF_FILL();



/********************* Sensor Ölçüm **************************************************/
	if(sensor_flag==1)
	{
		 sensor_flag=0;
		 prev_alt=altitude;
		 rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
		/* �?��?�터 취�? */
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

		if(rslt == BME280_OK)
		{
		  temperature = comp_data.temperature;
		  humidity = comp_data.humidity;
		  pressure = comp_data.pressure;
		  altitude=BME280_Get_Altitude()-offset_altitude;
		  speed=(altitude-prev_alt)*20;
    	}

		 LSM6DSLTR_Read_Accel_Data(&Lsm_Sensor);
		 calculate_roll_pitch(&Lsm_Sensor);
		 LSM6DSLTR_Read_Gyro_Data(&Lsm_Sensor);
		 update_angles(&Lsm_Sensor);

		 Lsm_Sensor.Accel_X=FIRFilter_Update(&accx,  Lsm_Sensor.Accel_X);
		 Lsm_Sensor.Gyro_X=FIRFilter_Update(&IMU_GYROX,  Lsm_Sensor.Gyro_X);
		 Lsm_Sensor.Gyro_Y=FIRFilter_Update(&IMU_GYROY, Lsm_Sensor.Gyro_Y);
		 Lsm_Sensor.Gyro_Z=FIRFilter_Update(&IMU_GYROZ, Lsm_Sensor.Gyro_Z);

		  fitil_kontrol=  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

		 BUTTON_STATE=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);


	}





/************ MEGÜ durum sorgusu **************************************************/
// EGU_durum_sorgusu[5]={0x54,0x52,0x35,0x0D,0x0A};
	if( (compare_arrays(EGU_RX_BUFFER, EGU_durum_sorgusu, EGU_RX_BUFFER_SIZE)) )
	{
		HAL_UART_Transmit(&huart6, EGU_TX_BUFFER, EGU_TX_BUFFER_SIZE, 1000);

		for(uint8_t i=0;i<5;i++)
		{
		EGU_RX_BUFFER[i++]=0;
		}
	}





/***********************************END*************************************************/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			switch(MEGU){
		case RAMPA:
				MEGU_mod=1;
			  //RAMPA MODU ROKET RAMPADA EGÜ SWİTCHLERİ VE ALT KADEME HABERLE�?ME KONTROL ET

				if(Lsm_Sensor.Accel_X > 1 && altitude >0 )
				  {
					rampa_control=1;
					MEGU=UCUS_BASLADI;
					Buzzer(6, 300);
				  }




			  break;

		case UCUS_BASLADI:
				MEGU_mod=2;
				// FLASH MEMORYE KAYDETMEYE BASLA
				flash_flag =1;

				MEGU=KADEME_AYRILDIMI;

			 break;

		case KADEME_AYRILDIMI:
				MEGU_mod=3;

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
			{


				MEGU=AYRILDI;


			}

			 break;

		case AYRILDI:
				MEGU_mod=4;
				 if ((compare_arrays(EGU_RX_BUFFER, EGU_motor_atesleme, EGU_RX_BUFFER_SIZE)))
				{

					if(Lsm_Sensor.Pitch >= 25 ) // pozisyon kontrolü
					{
						motor_ates=1;
						for(uint8_t i=0;i<5;i++)
						{
						EGU_RX_BUFFER[i++]=0;
						}

					}

				}
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);


			 break;


		}

	/************************************************************************************/
			  if(altitude >30 && MEGU <3)
			  {
				  altitude_rampa_control =1;
			  }
	/*************************************************************************************/
			  if(altitude>altitude_max) altitude_max = altitude;

			  if(speed>speed_max) speed_max = speed;

			  if( Lsm_Sensor.Accel_X> x_max) x_max =  Lsm_Sensor.Accel_X;


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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  htim6.Init.Prescaler = 840;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1680;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Pin|BUZZER_Pin|GATE_D_Pin|GATE_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED1_Pin|GATE_B_Pin|GATE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 BUTTON_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin BUZZER_Pin GATE_D_Pin GATE_C_Pin */
  GPIO_InitStruct.Pin = CS_Pin|BUZZER_Pin|GATE_D_Pin|GATE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin GATE_B_Pin GATE_A_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|GATE_B_Pin|GATE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int16_t BME_I2C_Testsensor(void){

	HAL_StatusTypeDef status;

	status=HAL_I2C_IsDeviceReady(&hi2c1,0x76 <<1, 4, 100);


	if(HAL_OK==status){

		return 0;
		}
	else {
		return 1;
		}


}
int16_t LSM_I2C_Testsensor(void){

	HAL_StatusTypeDef status;

	status=HAL_I2C_IsDeviceReady(&hi2c1,0x6A <<1, 4, 100);


	if(HAL_OK==status){

		return 0;
		}
	else {
		return 1;
		}


}

float BME280_Get_Altitude(void)
{
	float press = comp_data.pressure / 10000.0;
//	float temp = comp_data.temperature / 100.0;
	alt = 44330 * (1 - pow((press / 1013.25),(1/5.255)));
	//alt = ((pow((P0/press), (1/5.257))-1) * (temp + 273.15)) / 0.0065;

	return (alt);
}
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}

void Buzzer(int how_many , uint32_t how_long)
{
	if((how_many %2) == 1) how_many++;

	  for(uint8_t i=0;i<how_many; i++)
	  {
	  	    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
	  	    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
	  	    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
	  		HAL_Delay(how_long);

	  }

	  fitil_kontrol=  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	  if(!fitil_kontrol)
	  		{
	  			HAL_Delay(1000);
	  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	  			HAL_Delay(3000);
	  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	  		}
}

void Altitude_Offset()
{
	for(uint8_t i=0;i<5;i++)
	{
		HAL_Delay(40);
	  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	  if(rslt == BME280_OK)
	  { pressure = comp_data.pressure;
	    offset_altitude=BME280_Get_Altitude();
	  }
	}
}


int compare_arrays(uint8_t *array1, uint8_t *array2, uint16_t size) {
    for (uint16_t i = 0; i < size-1; i++) {
        if (array1[i] != array2[i]) {
            return 0; // Diziler farklı
        }
    }
    return 1; // Diziler aynı
}

void MEGU_TX_BUF_FILL(void)
{
	  EGU_TX_BUFFER[0]=egu_byte_0;
	  EGU_TX_BUFFER[1]=egu_byte_1;


	  float2unit8 f2u8_zaman;
	  f2u8_zaman.fVal=zaman;
     	 for(uint8_t i=0;i<4;i++)
	 	 {
     		EGU_TX_BUFFER[i+2]=f2u8_zaman.array[i];
	 	 }
   	  float2unit8 f2u8_battery;
   	  f2u8_battery.fVal=MEGU_battery;
        	 for(uint8_t i=0;i<4;i++)
   	 	 {
        		EGU_TX_BUFFER[i+6]=f2u8_battery.array[i];
   	 	 }
	 float2unit8 f2u8_alt;
	 f2u8_alt.fVal=altitude;
			 for(uint8_t i=0;i<4;i++)
		 {
				EGU_TX_BUFFER[i+10]=f2u8_alt.array[i];
		 }
	 float2unit8 f2u8_max_alt;
	 f2u8_max_alt.fVal=max_altitude;
	 	 	 for(uint8_t i=0;i<4;i++)
		 {
				EGU_TX_BUFFER[i+14]=f2u8_max_alt.array[i];
		 }
	 float2unit8 f2u8_x;
	 f2u8_x.fVal=Lsm_Sensor.Accel_X;
	 	 	 for(uint8_t i=0;i<4;i++)
		 {
				EGU_TX_BUFFER[i+18]=f2u8_x.array[i];
		 }
	  float2unit8 f2u8_pitch;
	  f2u8_pitch.fVal=Lsm_Sensor.Pitch;
	  	 	 for(uint8_t i=0;i<4;i++)
		 {
	 			EGU_TX_BUFFER[i+22]=f2u8_pitch.array[i];
		 }




	  EGU_TX_BUFFER[26]=rampa_control;//uçuş başladı mı
	  EGU_TX_BUFFER[27]=motor_ates;//motor ateşleme sinyali geldi mi
	  EGU_TX_BUFFER[28]=egu_byte_28;
	  EGU_TX_BUFFER[29]=ariza;//ariza tespit
	  EGU_TX_BUFFER[30]=fitil_kontrol;
	  EGU_TX_BUFFER[31]=egu_byte_31;
	  EGU_TX_BUFFER[32]=egu_byte_32;
	  EGU_TX_BUFFER[33]=egu_byte_33;



}
void bme_config(void)
{
	  dev.dev_id = BME280_I2C_ADDR_PRIM;
	  dev.intf = BME280_I2C_INTF;
	  dev.read = user_i2c_read;
	  dev.write = user_i2c_write;
	  dev.delay_ms = user_delay_ms;

	  rslt = bme280_init(&dev);

	  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	  dev.settings.osr_p = BME280_OVERSAMPLING_4X;
	  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	  dev.settings.filter = BME280_FILTER_COEFF_16;
	  rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);


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
