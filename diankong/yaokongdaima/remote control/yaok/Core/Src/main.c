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
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "NF.h"
#include "math.h"
#define PI 3.14159265
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
 uint16_t ADC_Value;
    uint16_t  adc_buff[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern unsigned char INIT_ADDR[5]= {0x00,0x1A,0xB1,0xB1,0x01}; //�ڵ��ַ
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
       int k=1;
int c=1;
int key3=0;
int key1=32;//配置按键8按下进行下一步操作
int key8=32;//配置按键8按下进行下一步操作
int key5=0;
int key7=0;
int key2=0;
int key4=0;
uint8_t array[40] ; //第一位接收不到
int yaw1=0;
int yaw2=0;
int key6=0, key6_stuas=0;
float yaw=0;

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  
   while(NRF24L01_Check())
	{
		
		k=0;
 		HAL_Delay(1000);
	}
     
	 OLED_Init();
	
      
   NRF24L01_TX_Mode();//设置为发送模式
    
	                 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 
	  k++;
	  //W_MOSI(1);
	   //OLED_ShowNum(0,0,20,4,16);
	  
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, 4);
	  
	
	if(key6==1){array[27]=1;key6_stuas=1; HAL_Delay(2); }
		if(key6==0){array[27]=0;} 
	
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==GPIO_PIN_RESET)
		{
		   HAL_Delay(20);
		   if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==GPIO_PIN_RESET)
		{
			
			
			key4=1;
			key7=0;
		}
		
		
		}		
		
		
     if(key6_stuas==1){
		 if(NRF24L01_TxPacket(array)==TX_OK)
    {
      c=3;
    }
    else
    {
        c=2;
    } } 
      if(key4==1&&key7==0){array[29]=1;}
		if(key7==1&&key4==0){array[29]=0;}
 
	  if(key1==1&&key2==0){array[28]=1;
							array[20]=2;}	
	    if(key1==0&&key2==1){array[28]=0;array[20]=1;}	
	   if(key3==1){
		array[30]=7;
		
	  for(int g=0;g<=4;g++) 
	  {
		  
		  array[g+1] = adc_buff[g]/20;   
	  
	  }
	     HAL_Delay(2);
      
      if(NRF24L01_TxPacket(array)==TX_OK)
    {
      c=3;
    }
    else
    {
        c=2;
    }  
		
		
		
		}
	   
	  
	  if(key8==1){
		array[30]=6;	
	  for(int g=0;g<=4;g++) 
	  {
		  
		  array[g+1] = adc_buff[g]/20;   
	  
	  }
	     HAL_Delay(2);
      
      if(NRF24L01_TxPacket(array)==TX_OK)
    {
      c=3;
    }
    else
    {
        c=2;
    }  
		
		
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_DMA_ConcludeCallback(DMA_HandleTypeDef *hdma)
{
    if(hdma->Instance == DMA1_Channel1)
    {
		     OLED_ShowNum(0,0,3,4,16);
         HAL_ADC_Stop_DMA(&hadc1);
    }
}  
void delay_us(uint32_t udelay)
{
  uint32_t startval,tickn,delays,wait;
 
  startval = SysTick->VAL;
  tickn = HAL_GetTick();
  //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
  delays =udelay * 48; //sysc / 1000 * udelay;
  if(delays > startval)
    {
      while(HAL_GetTick() == tickn)
        {
 
        }
      wait = 48000 + startval - delays;
      while(wait < SysTick->VAL)
        {
 
        }
    }
  else
    {
      wait = startval - delays;
      while(wait < SysTick->VAL && HAL_GetTick() == tickn)
        {
 
        }
    }
}
//按键中断

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==KEY1_Pin)
  {
	  if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET)
    {
	   key1=1;
		key2=0;
	}
	__HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin);
  }
	 if(GPIO_Pin==KEY2_Pin)
  {
	  if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==GPIO_PIN_RESET)
    {
	   key1=0;
		key2=1;
	}
	__HAL_GPIO_EXTI_CLEAR_IT(KEY2_Pin);
  }
	if(GPIO_Pin==KEY3_Pin)
  {
	  if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==GPIO_PIN_RESET)
    {
	    key3=1;
	   key8=0;
	}
	__HAL_GPIO_EXTI_CLEAR_IT(KEY3_Pin);
	  
  }
  if(GPIO_Pin==KEY8_Pin)
  {
  
    if(HAL_GPIO_ReadPin(KEY8_GPIO_Port,KEY8_Pin)==GPIO_PIN_RESET)
    {
    
	key8=1;
	key3=0;
		
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY8_Pin);
  }
//   if(GPIO_Pin==KEY5_Pin)
//  {
//  
//    if(HAL_GPIO_ReadPin(KEY5_GPIO_Port,KEY5_Pin)==GPIO_PIN_RESET)
//    {
//    
//	key5=1;
//	key7=0;	
//    }
//    __HAL_GPIO_EXTI_CLEAR_IT(KEY5_Pin);
//  }
   if(GPIO_Pin==KEY7_Pin)
  {
  
    if(HAL_GPIO_ReadPin(KEY7_GPIO_Port,KEY7_Pin)==GPIO_PIN_RESET)
    {
		key4=0;
	key7=1;
		
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY7_Pin);
  }
  
   if(GPIO_Pin==KEY6_Pin)
  {
  
    if(HAL_GPIO_ReadPin(KEY6_GPIO_Port,KEY6_Pin)==GPIO_PIN_RESET)
    {
		// key6++;
			if(key6==0){key6=1;}
			else if(key6==1){key6=0;}
	
		
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY6_Pin);
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
