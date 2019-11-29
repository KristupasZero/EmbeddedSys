/**

******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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



#define  PERIOD_VALUE       (uint32_t)(666 - 1) //Period Value
#define  PULSELOW_VALUE        (uint32_t)(PERIOD_VALUE/3)        /* Capture Compare 1 Value  */
#define  PULSEMEDIUM_VALUE       (uint32_t)(PERIOD_VALUE*2/3) /* Capture Compare 2 Value  */
#define  PULSEHIGH_VALUE       (uint32_t)(PERIOD_VALUE)        /* Capture Compare 3 Value  */
#define  PULSEOFF_VALUE       (uint32_t)(PERIOD_VALUE*0)  


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

ADC_HandleTypeDef    Adc_Handle;

TIM_HandleTypeDef    Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure, Tim4_OCInitStructure;

uint16_t Tim3_PrescalerValue,Tim4_PrescalerValue;
__IO uint16_t ADC1ConvertedValue;   //if declare it as 16t, it will not work.


volatile double  setPoint = 23.5;
uint32_t ADC_BUF[8];
double measuredTemp; 
ADC_HandleTypeDef             Adc_Handle;
int8_t state =1;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef        sConfig;

/* Converted value declaration */
uint32_t                      aResultDMA;


char lcd_buffer[6];    // LCD display buffer


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void TIM4_Config (void);



//static void EXTILine14_Config(void); // configure the exti line4, for exterrnal button, WHICH BUTTON?


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	HAL_Init();

	SystemClock_Config();   
	
	HAL_InitTick(0x0000); // set systick's priority to the highest.

	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);


	BSP_LCD_GLASS_Init();
	//Config our timer 4
	TIM4_Config();
	BSP_JOY_Init(JOY_MODE_EXTI);  
	
	 /* ### - 1 - Initialize ADC peripheral #################################### */
  Adc_Handle.Instance          = ADC1;
  if (HAL_ADC_DeInit(&Adc_Handle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }
	//ADC Set up
  Adc_Handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
  Adc_Handle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  Adc_Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  Adc_Handle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  Adc_Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  Adc_Handle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  Adc_Handle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  Adc_Handle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  Adc_Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  Adc_Handle.Init.DMAContinuousRequests = ENABLE;                        /* DMA circular mode selected */
  Adc_Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  Adc_Handle.Init.OversamplingMode      = DISABLE;                       /* No oversampling */

  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&Adc_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&Adc_Handle, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    Error_Handler();
  }
	else
	{
		measuredTemp = HAL_ADC_GetValue(&Adc_Handle);
	}
  
  /* ### - 3 - Channel configuration ######################################## */
  sConfig.Channel      = 6;                /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&Adc_Handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 4 - Start conversion in DMA mode ################################# */
  if (HAL_ADC_Start_DMA(&Adc_Handle, &aResultDMA, 1) != HAL_OK)
  {
    Error_Handler();
  }

	
	
	//HAL_ADC_Start(&Adc_Handle);
	//HAL_ADC_Start_DMA(&Adc_Handle,(uint32_t*)ADC_BUF,8);
	//measuredTemp = (int32_t)HAL_ADC_GetValue(&Adc_Handle);
	

	BSP_LCD_GLASS_Clear();
	//Start ADC
	HAL_ADC_Start_IT(&Adc_Handle);
  while (1)
  {
		if(state==1)
		{
			//Get our calc value
			measuredTemp = (int32_t)HAL_ADC_GetValue(&Adc_Handle);
			measuredTemp = 0.0244140625 * measuredTemp;
      
			//Print the value
			sprintf(lcd_buffer,"%0.1f",measuredTemp);
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
			BSP_LCD_GLASS_Clear();
			} 
		 
		if(state==-1)
		{
			sprintf(lcd_buffer,"%0.1f",setPoint);
			BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
			HAL_Delay(250);
			BSP_LCD_GLASS_Clear();
		}
		//Comparing our value to the Set point
		//Changes the fan setting according to the difference
	if(measuredTemp-10 > setPoint)
      {
        Tim4_OCInitStructure.Pulse=PULSEHIGH_VALUE;
      }else if(measuredTemp-5 >setPoint){
        Tim4_OCInitStructure.Pulse=PULSEMEDIUM_VALUE;

      }else if (measuredTemp>setPoint){
        Tim4_OCInitStructure.Pulse=PULSELOW_VALUE;
      }else{
        Tim4_OCInitStructure.Pulse=PULSEOFF_VALUE;
      }
			//Error Handling
				if (HAL_TIM_PWM_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1) != HAL_OK){
    Error_Handler();
				}
//Error Handling
  if (HAL_TIM_PWM_Start(&Tim4_Handle, TIM_CHANNEL_1) != HAL_OK){
    Error_Handler();
  }
		}
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */

//Clock Set up
void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error
		BSP_LED_On(LED5);
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
		BSP_LED_On(LED5);
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}




/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						state = state * -1;
						break;	
			case GPIO_PIN_1:     //left button						
							
							break;
			case GPIO_PIN_2:    //right button						  to play again.
						
							break;
			case GPIO_PIN_3:    //up button]
              if(state==-1)							
							setPoint += 0.5;
							break;
			
			case GPIO_PIN_5:    //down button		
              if(state==-1)					
							setPoint -= 0.5;
							break;
			
			default://
						//default
						break;
	  } 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 

}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM4_pwm
	
	__HAL_TIM_SET_COUNTER(htim, 0x0000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* Adc_Handle)
{
	/*
	sprintf(lcd_buffer,"%s","testing");
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
	HAL_Delay(500);
	BSP_LCD_GLASS_Clear();
	
	uint32_t val[8];
	if(Adc_Handle ->Instance ==ADC1)
		{
			val[0] = ADC_BUF[0];
			val[1] = ADC_BUF[1];
			val[2] = ADC_BUF[2];
			val[3] = ADC_BUF[3];
			val[4] = ADC_BUF[4];
			val[5] = ADC_BUF[5];
			val[6] = ADC_BUF[6];
			val[7] = ADC_BUF[7];
			
			for(int i=0;i<8;i++)
			{
				measuredTemp = measuredTemp + val[i];
			}
			measuredTemp = measuredTemp/8;
			HAL_ADC_Stop_DMA(Adc_Handle);
			HAL_ADC_Stop(Adc_Handle);
    }
	*/
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz
void  TIM4_Config(void)
{
  
  /* Compute the prescaler value to have TIM4 counter clock equal to 1 Hz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 160000) - 1;
  
  /* Set TIM3 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = PERIOD_VALUE; // Arr register (ovrflow) for timer 4, want timer 3
	
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  Tim4_Handle.Init.RepetitionCounter = 0;

  if(HAL_TIM_PWM_Init(&Tim4_Handle) != HAL_OK){
    Error_Handler();
  }
	//Set up the PWM parts
  Tim4_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  Tim4_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  Tim4_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
  Tim4_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  Tim4_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  Tim4_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
	//Set default value for the TIM4
  Tim4_OCInitStructure.Pulse = PULSEMEDIUM_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1) != HAL_OK){
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&Tim4_Handle, TIM_CHANNEL_1) != HAL_OK){
    Error_Handler();
  }
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}


static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
