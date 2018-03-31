/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t     TIM4CH1_CAPTURE_STA=0X80;		//输入捕获状态		    				
float       TIM_FREQ;                       //频率值
uint32_t	  TIM1_COUNTER_TEMP;	            //定时器1计数值
uint32_t    TIM2_COUNTER_TEMP;              //定时器2计数值
/* USER CODE END 0 */

int main(void)
{

  /* U SER CODE BEGIN 1 */
    uint8_t f=0xff;
    char x='\"';
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1); //使能TIM4输入捕获中断
  __HAL_TIM_ENABLE_IT(&htim4, TIM_CHANNEL_1); //使能更新中断


  
    __HAL_TIM_SET_COUNTER(&htim1,0);      //定时器1清零
    __HAL_TIM_SET_COUNTER(&htim2,0);      //定时器2清零 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    if(TIM4CH1_CAPTURE_STA&0X800)
    {            
        TIM_FREQ = (float)(TIM2_COUNTER_TEMP*20.0f)/TIM1_COUNTER_TEMP;
        
        printf("%.2fKHZ",TIM_FREQ);
        
        TIM1_COUNTER_TEMP = 0;
        TIM2_COUNTER_TEMP = 0;
        TIM4CH1_CAPTURE_STA = 0X80;
        HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
    }  
    
    else if(TIM4CH1_CAPTURE_STA&0X400)
    {
        TIM_FREQ = (float) 20000 / TIM1_COUNTER_TEMP;
        printf("%.2fHZ",TIM_FREQ);
        TIM1_COUNTER_TEMP = 0;
        TIM4CH1_CAPTURE_STA = 0X80;
    }
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//定时器TIM4输入捕获中断处理回调函数，该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{	 
                    //粗测第一次捕获上升沿
                    if(TIM4CH1_CAPTURE_STA&0X80)
                    {
                        TIM4CH1_CAPTURE_STA = 0X40;		//等待第二次上升沿
                        HAL_TIM_Base_Start(&htim1);     //使能定时器1，对标准信号计数
                    }	
                    //完成粗测
                    else if(TIM4CH1_CAPTURE_STA&0X40)
                    {
                        if(__HAL_TIM_GET_COUNTER(&htim1) < 200)//获得定时器1计数值,小于200，则频率大于100
                            TIM4CH1_CAPTURE_STA = 0x20;                    //开启等精度测频
                        else 
                            TIM4CH1_CAPTURE_STA = 0x04;                    //开启低频测频
                        
                       HAL_TIM_Base_Stop(&htim1);                        //关闭定时器1
                       __HAL_TIM_SET_COUNTER(&htim1,0);       //定时器1清零
                    }  
                    
                    else if(TIM4CH1_CAPTURE_STA&0X20)             //开启等精度测评
                    {
                        TIM4CH1_CAPTURE_STA = 0X10;                      //等待闸门时间
                        HAL_TIM_Base_Start_IT(&htim5);                           //使能TIM5定时器中断
                        __HAL_TIM_SET_COUNTER(&htim5,0);      //定时器5清零
                    }
                    
                    else if(TIM4CH1_CAPTURE_STA&0X10)              //开启计数
                    {
                          TIM4CH1_CAPTURE_STA = 0X01;   //等待闸门时间
                          HAL_TIM_Base_Start(&htim1);     //对标准信号计数
                          HAL_TIM_Base_Start(&htim2);     //对被测信号计数
                    }
                    
                   else if(TIM4CH1_CAPTURE_STA&0X08)	//完成一次等精度测频
                  {	                      
                        TIM1_COUNTER_TEMP = __HAL_TIM_GET_COUNTER(&htim1); //获得定时器1计数值
                        TIM2_COUNTER_TEMP = __HAL_TIM_GET_COUNTER(&htim2); //获得定时器2计数值
                                            
                        HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
                        HAL_TIM_Base_Stop(&htim1);            //关闭定时器1
                        HAL_TIM_Base_Stop(&htim2);            //关闭定时器2 
                        __HAL_TIM_SET_COUNTER(&htim1,0);      //定时器1清零
                        __HAL_TIM_SET_COUNTER(&htim2,0);      //定时器2清零     
                        TIM4CH1_CAPTURE_STA = 0X800;		//完成一次等精度测频                      
                   }
                  
                   else if(TIM4CH1_CAPTURE_STA&0X04)        //开启低频检测
                   {
                       TIM4CH1_CAPTURE_STA = 0X02;    //等待第二次检测上升沿
                       HAL_TIM_Base_Start(&htim1);     //使能定时器1，对标准信号计数
                   }
                   
                   else if(TIM4CH1_CAPTURE_STA&0X02)      //完成低频检测
                   {
                       TIM1_COUNTER_TEMP = __HAL_TIM_GET_COUNTER(&htim1); //获得定时器1计数值
                       HAL_TIM_Base_Stop(&htim1);            //关闭定时器1
                       __HAL_TIM_SET_COUNTER(&htim1,0);      //定时器1清零
                       TIM4CH1_CAPTURE_STA = 0X400;  //输出结果
                   }
              
}
		

 //定时器TIM5溢出中断处理回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(TIM5 == htim->Instance)
    {
       if(TIM4CH1_CAPTURE_STA&0x01)
            TIM4CH1_CAPTURE_STA = 0x08;
    }
}    

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch ,1, 0xffff);
    
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
