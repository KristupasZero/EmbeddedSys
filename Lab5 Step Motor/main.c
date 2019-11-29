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




/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

int  t=1,which=1;
int Tmode=3,TDirection;
int8_t mode = 1;
uint16_t speed = 4000;
uint8_t direction = 1;
uint8_t count = 0;
uint8_t half_step_state = 0;
static GPIO_InitTypeDef  GPIOArray[4];
TIM_HandleTypeDef   Tim3_Handle;
uint16_t Tim3_PrescalerValue;


char lcd_buffer[6];    // LCD display buffer


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM3_Config(void);
void clear(int set );
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
	
	
	SystemClock_Config();   //sysclock is 80Hz. HClkm apb1 an apb2 are all 80Mhz.
  
	HAL_InitTick(0x0000); // set systick's priority to the highest.
	//Sets up an array of pins
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIOArray[0].Pin=GPIO_PIN_12;
  GPIOArray[1].Pin=GPIO_PIN_14;
  GPIOArray[2].Pin=GPIO_PIN_13;
  GPIOArray[3].Pin=GPIO_PIN_15;
  for(int i=0; i<4; i++){
    GPIOArray[i].Mode  = GPIO_MODE_OUTPUT_PP;
    GPIOArray[i].Pull  = GPIO_PULLUP;
    GPIOArray[i].Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIOArray[i]);
  }
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);

  
	BSP_LCD_GLASS_Init();
//	GPIO_CLK_ENABLE();
	clear(0);
	BSP_JOY_Init(JOY_MODE_EXTI);  
	TIM3_Config();


 	
  while (1)
  {
		
	} //end of while 1

}


void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    Tim3 is of 16 bits. Timer 2..7 is on APB1.
	
		Since the clock source is MSI, and the clock range is RCC_MSIRANGE_6, SystemCoreClock=4Mhz.
	
		Since RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1, HCLK=4Mhz.
	
		Since RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1, PCLK1=4MHz, and TIM3CLK=4Mhz.
		(if the APB1 prescaler is not 1, the timer clock frequency will be 2 times of APB1 frequency)
	
		 that is, for current RCC config, the the Timer3 frequency=SystemCoreClock.
	
		To get TIM3's counter clock at 10 KHz, for example, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	
		i.e: Prescaler = (SystemCoreClock /10 KHz) - 1
       
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/ 4000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
 
  Tim3_Handle.Init.Period = speed - 1;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
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

void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
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
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz??


void clear(int set){
	if(set==0){
	HAL_GPIO_WritePin(GPIOE,GPIOArray[0].Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOE,GPIOArray[0].Pin,GPIO_PIN_RESET);
	}
	for(int i=1; i<4; i++){
		HAL_GPIO_WritePin(GPIOE,GPIOArray[i].Pin,GPIO_PIN_RESET);
	}
	
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 	
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");
						//SELECT button			
// mode = 0 means half stepping, mode = 1 means full stepping			
							if(mode==0){
								mode=1;
								speed=speed/2;
							}else{
								mode=0;
								speed=speed*2;
							}
			
						break;	
			case GPIO_PIN_1:     //left button
							direction = 0;  // clockwise
							break;
			case GPIO_PIN_2:    //right button						 
							direction = 1; // counter clockwise
							break;
			case GPIO_PIN_3:    //up button
								if(speed -5 <=0)  // minimum speed is 0 which is the stop speed
							{
								speed -= 1;
							}else{
								speed-=250;
							}
              TIM3_Config();
							break;
			
			case GPIO_PIN_5:    //down button						
               speed += 250;
               TIM3_Config();
							break;
			default://
						//default
						break;
	  } 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32fxx_hal_tim.c for different callback function names. 
																															//for timer 3  , Timer 3  use update event initerrupt
{
	//Resets the Mode, if the temp variable is not the current one
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");
	if(Tmode!=mode || TDirection!=direction){
		//Sets the temp variables
		Tmode=mode;
		count=0;
		TDirection=direction;
		//Different resets depending on mode or direction
		if(direction==0){
			t=1;
			which=1;
		}else if(mode==0 && direction==1){
			t=0;
			which=0;
			count=3;
		}
		//Clears all the pins
		clear(0);
		return;
	}
	//For Full Steps 
	if(mode==1){
		//Clockwise
		if(direction==0){
			//Toggles last pin, and toggles the current pin to get a cycle that loops when count = 3
			if(count!=3){
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				count++;
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				return;
			}else{
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				count=0;
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				return;
			}
			//Clockwise Direction
		}else if (direction==1){
			if(count!=0){
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				count--;
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				return;
			}else{
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				count=3;
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				return;
			}
		}
		//Half Step
	}else if(mode==0){
		//Clockwise
		if(direction==0){
			if(which==1){
				//T is second counter, easiest way to implement this cycle having two alternating indexes, that toggling on accending order
				if(t!=3){
					HAL_GPIO_TogglePin(GPIOE,GPIOArray[t].Pin);
					t++;
				}else{
					HAL_GPIO_TogglePin(GPIOE,GPIOArray[t].Pin);
					t=0;
				}
				which=0;
				return;
			}else{
				//Switches to count index which the second index, that toggles every other cycle
				if(count!=3){
					HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
					count++;
				}else{
					HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
					count=0;
				}
				which=1;
				return;
			}
	// Counter Clock Wise direction
	}else if(direction==1){
		if(which==1){
			//First Index but in opposite order
			if(t!=0){
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[t].Pin);
				t--;
			}else{
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[t].Pin);
				t=3;
			}
				which=0;
				return;
		}else{
			//Second Index but in opposite order
			if(count!=0){
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				count--;
			}else{
				HAL_GPIO_TogglePin(GPIOE,GPIOArray[count].Pin);
				count=3;
			}
			which=1;
			return;
			}
	}
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 
		
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
