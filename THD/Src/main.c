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
#include "dma.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define length 4096
#define TIMCLK 409600
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void getthd(float *input,uint32_t pos,uint8_t n,float *THD);
float getfftcmplx(float *input,uint8_t n);
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint8_t adcflag;
uint32_t dma_adcvalue[length];
float adcinput[length*2];
float adcoutput[length];
float maxvalue;
uint32_t max_pos;
float THD;
float thd_fm;
float thd_fz;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1,dma_adcvalue,length);
	HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(adcflag)
		{
			adcflag=0;
			for(int i=0;i<length;i++)
			{
				adcinput[i*2]=dma_adcvalue[i]*3.3f/4095.0f;
				adcinput[i*2+1]=0.0f;
			}
			arm_cfft_f32(&arm_cfft_sR_f32_len4096,adcinput,0,1);//FFT变换
			arm_cmplx_mag_f32(adcinput,adcoutput,length);//求频谱幅度
			adcoutput[0]=0.0f;//直流分量置0
			arm_max_f32(adcoutput,length,&maxvalue,&max_pos);//找基波位置	
			if(max_pos>length/2) max_pos=length-max_pos;
//			max_pos1=max_pos;
//			V1=getfftcmplx(&adcoutput[max_pos],2)*2.0f/length;
//			
//			max_pos+=max_pos1;
//			if(max_pos>length/2) max_pos=length-max_pos;
//			V2=adcoutput[max_pos]*2.0f/length;
//			
//			max_pos+=max_pos1;
//			if(max_pos>length/2) max_pos=length-max_pos;
//			V3=adcoutput[max_pos]*2.0f/length;
//			
//			max_pos+=max_pos1;
//			if(max_pos>length/2) max_pos=length-max_pos;
//			V4=adcoutput[max_pos]*2.0f/length;
//			
//			max_pos+=max_pos1;
//			if(max_pos>length/2) max_pos=length-max_pos;
//			V5=adcoutput[max_pos]*2.0f/length;
//			
//			thd_fz=V2*V2+V3*V3+V4*V4+V5*V5;
//			thd_fz=sqrt(thd_fz);
//			thd_fm=V1;
//			THD=thd_fz*100.0f/thd_fm;
			getthd(adcoutput,max_pos,5,&THD);
			for(int i=0;i<length;i++)
			{
				printf("%.3f,%d\n\r",adcoutput[i],i);
			}
			HAL_ADC_Start_DMA(&hadc1,dma_adcvalue,length);
		}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//频谱数组，基波位置，谐波次数,THD结果
void getthd(float *input,uint32_t pos,uint8_t n,float *THD)
{
	float u1,u2=0.0f;
	uint32_t pos1=pos;
	u1=getfftcmplx(&input[pos1],2)*2.0f/length;
	for(int i=1;i<n;i++)
	{
		pos1+=pos;
		if(pos1>length/2) pos1=length-pos1;
		u2+=pow(input[pos1]*2.0f/length,2);
	}
	*THD=sqrt(u2)*100.0f/u1;
}
//求基波幅度值，输入数组，需要聚集的相邻点数
float getfftcmplx(float *input,uint8_t n)
{
	float result=0.0f;
	for(int i=1;i<=n;i++)
	{
		result+=*(input+n)+*(input-n);
	}
	result+=*input;
	return result;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcflag=1;
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
