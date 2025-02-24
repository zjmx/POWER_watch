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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "math.h"
#include "can_receive.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
//运放采样校准系数
adc_data_t adc_data= 
{//y=k*x+b
	.chas_v_m=1.0126f, .chas_v_a=0.0323f,
	.chas_i_m=1.1638f, .chas_i_a=-0.3145f,   
};

float P;
int16_t V_f,I_f,P_f;
int16_t data[4]={};
int gpp=0,gdd=0;
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	//can
	CAN_Init();
	//oled
	OLED_Init();
	//tim
	HAL_TIM_Base_Start_IT(&htim3);
	//adc
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_data.adc1_list_record,2*10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		P=adc_data.chas_v*adc_data.chas_i;
		V_f=100*(fabs(adc_data.chas_v-(int)adc_data.chas_v));
		I_f=100*(fabs(adc_data.chas_i-(int)adc_data.chas_i));
		P_f=100*(fabs((P-(int)P)));
		
		OLED_ShowString(1,1,"V:");
		OLED_ShowSignedNum(1,3,(int)adc_data.chas_v,3);
		OLED_ShowChar(1,7,'.');
		OLED_ShowNum(1,8,V_f,2);
		
		OLED_ShowString(2,1,"I:");
		OLED_ShowSignedNum(2,3,(int)adc_data.chas_i,3);
		OLED_ShowChar(2,7,'.');
		OLED_ShowNum(2,8,I_f,2);
		
		OLED_ShowString(3,1,"P:");
		OLED_ShowSignedNum(3,3,(int)P,3);
		OLED_ShowChar(3,7,'.');
		OLED_ShowNum(3,8,P_f,2);
		
		data[0]=float_to_uint(adc_data.chas_v,-48,48,16);
		data[1]=float_to_uint(adc_data.chas_i,-48,48,12);
		data[2]=float_to_uint(adc_data.chas_v*adc_data.chas_i,-48,48,12);
		
	  CAN_SendData(&hcan,0x00,data);
		gpp=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
		gdd=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
		HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int count;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{//系统定时器1000Hz
  if(htim->Instance == TIM3)
  {
		//计数器变量
		count++;
		
		adc_data.chas_v=(float)adc_data.adc_list_aver[0]/4096*3.3f*15*adc_data.chas_v_m+adc_data.chas_v_a;
		adc_data.chas_i=-((float)(adc_data.adc_list_aver[1])/4096*3.3f-1.65f)/0.075f*adc_data.chas_i_m+adc_data.chas_i_a;
		//清零
		for(uint8_t i=0;i<2;i++)
		{
			adc_data.adc_list_sum[i]=0;
		}
		//求和
		for(uint8_t i=0;i<10;i++)
		{
			for(uint8_t j=0;j<2;j++)
			{
				adc_data.adc_list_sum[j]+=adc_data.adc1_list_record[i][j];
			}
		}
		//取平均
		for(uint8_t i=0;i<2;i++)
		{
			adc_data.adc_list_aver[i]=adc_data.adc_list_sum[i]/10;
		}
		
	
		
		if(count%500==0)
		{

		}
		if(count%1000==0)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
			count=0;
		}
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
