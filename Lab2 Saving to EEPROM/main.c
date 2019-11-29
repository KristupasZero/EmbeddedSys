/******************************************************************************
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

char lcd_buffer[6];    // LCD display buffer

TIM_HandleTypeDef    Tim3_Handle,Tim4_Handle;
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t Tim3_PrescalerValue,Tim4_PrescalerValue;

__IO uint16_t Tim4_CCR; // the pulse of the TIM4
__O uint8_t factor = 0;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; // the emulated EEPROM can save 3 varibles, at these three addresses.
uint16_t EEREAD=10000;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type
Point_Typedef singlePoint;
DoublePoint_Typedef doublePoint;
DigitPosition_Typedef charPosition;

RNG_HandleTypeDef Rng_Handle;



/*Variable we actually used
	On_off is a simple int that determines if the led is on or off and then multiplied by -1 to change the state to get a flashing state
	Count is the variable used for pretty much every counting function, this is in ms
			Such as:
					Counting the number of flashes
					Counting the time to get to the randomly generated number
					Counting the time it takes the user to react
	Mode is a varialbe that is used to determine the state of the function and where it should go next, all the button pressed are synced on the mode
	Some buttons will only work if within a certain mode
					For the different modes:
						0. A restart mode, both the LEDs are turned off, and count is set to zero from within this state, only the select button will take it to it's next statte (2), 
						1. The beginning state, the lights will flash for a certain number of times, after enough counts the mode goes to 1
						2. A random Number is generated, using the RNG function (HAL_RNG_GetRandomNumber(&Rng_Handle)%(5000)+17000) (Roughly 1.7s +- 0.5s) (Reason for this is the clock speed being 0.1ms) after this is done goes to mode 3
						3. Count is again used to count up to the randomly generated number, after this is achieved the LED is turned on and count is reset and mode is 4
						4. Count is used to time the reaction time of the user, it's waiting for the selection button to be pressed to stop it, then the count is used to compare to the best time from the EEPROM and stored if lower, LED5 is PRESSED to be able to visual the difference
	randTime is the variable used to store the randomly generated number


*/
int on_off = 1;
int count = 0;
int mode = 1;
int randTime;





/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM3_Config(void);



//static boolean pressed=false;

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

  /* Configure the system clock to 4 MHz */
  SystemClock_Config();
 
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest
	//Setting up LEDs/Joy Stick
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Off(LED4);
	BSP_LED_Off(LED5);
	EE_WriteVariable(VirtAddVarTab[0], (uint16_t)100000);

	BSP_JOY_Init(JOY_MODE_EXTI);

	BSP_LCD_GLASS_Init();
	TIM3_Config();


	//Clearing the glass
	BSP_LCD_GLASS_Clear();
	
	







	
//******************* use emulated EEPROM ====================================
	//First, Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
		
// EEPROM Init 
	EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }

	
	
	
	
	
	
	
	
	
	
//*********************use RNG ================================  
Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	Hal_status=HAL_RNG_Init(&Rng_Handle);   //go to msp.c to see further low level initiation.
	
	if( Hal_status != HAL_OK)
  {
    Error_Handler();
  }
//then can use RNG

//	BSP_LCD_GLASS_DisplayString();
	//Bit masking
	//we want 0-2000, &=0x0000007D0;
	

  /* Infinite loop */
  while (1)
  {
		

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

  // The following clock configuration sets the Clock configuration sets after System reset                
  // It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig 
  // and to be eventually adapted to new clock configuration                                               

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48
	
	
	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz
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
  Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/ 100000) - 1;
  /*We changed this part, with the two values
			100000 /10 is 0.0001s for the clock speed
			However all the other measurements are multiplied by 10
			The reason for this is if the clock speed were to be 0.001s or 1ms then the time it takes to call the interupts and do other processing work causes the visibile timer to longer to be 1ms
			
	
	
	
	
	
	
	
	*/
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
	Tim3_Handle.Init.ccr =3;
  Tim3_Handle.Init.Period = 10 - 1;
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
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						//If waiting for the game to start, and the button is pressed the game begins
						if(mode==0){
							mode=2;
						}
						//When waiting for the input of the button to stop the timer (mode 4)
						else if(mode==4){
							//Gets the time, can't use count for the rest, because count keeps on getting updated by the system interupts
							int score=count;
							//Turns on the other LED so the user can see when they have pressed the button
							BSP_LED_On(LED4);
							//Clearing the LCD so the score can be displayed
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%d",score);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							//Reads the value from the EEPROM to be able to compare later
							EE_ReadVariable(VirtAddVarTab[0],&EEREAD);
							//If the score is really low, then cheating is involed
							if(score<90){
								//Resets the Mode
								mode=1;
								BSP_LCD_GLASS_Clear();
								BSP_LCD_GLASS_DisplayString((uint8_t*)"CHEAT");	
								//If the score lower than the current record, the score is written into the EEPROM
							}else if(score<=EEREAD){
								EE_WriteVariable(VirtAddVarTab[0], (uint16_t)score);
							}

						}
			
						break;	
			case GPIO_PIN_1:     //left button	
							//The intial Mode / Reset of all major components 
							mode = 1;
							BSP_LCD_GLASS_Clear();
							BSP_LED_Off(LED4);
							BSP_LED_Off(LED5);
							count = 0;
							break;
			case GPIO_PIN_2:    //right button
							//Simply displays the random time it'll take the light to turn on (Used for testing mostly)
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%d",randTime);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							break;
			case GPIO_PIN_3:    //up button
							//Displays the current Mode (Used for testing mostly)
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%d",mode);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							break;
			
			case GPIO_PIN_5:    //down button		
							//Reads the best record from EEPROM and then displays it on the LCD
							EE_ReadVariable(VirtAddVarTab[0],&EEREAD);
							BSP_LCD_GLASS_Clear();
							sprintf(lcd_buffer,"%d",EEREAD);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							break;
			default://
						//default
						break;
	  } 
}

//Interupt function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32lxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
	//To specify the interrupt was triggered by TIM3
	if ((*htim).Instance==TIM3){
				//If in waiting state Turns off LEDs and Count
				if(mode==0){
					BSP_LED_Off(LED4);
					BSP_LED_Off(LED5);
					count=0;
				}
				//Intial State flashes the LED 4 on and off
        if(mode == 1){
						//Count used to count how many times the interrupt has been trigger
            count += 1;
						//To ensure the light is flashing at a reasonable pace and not at the count speed
						if(count%1000==0 ){
							//Alternates the state of on/off
							if(on_off == 1){
								BSP_LED_On(LED4);
								on_off *= -1;
							}
							else if( on_off == -1){
								BSP_LED_Off(LED4);
								on_off *= -1;
							}
					//After enough repetitions the LED turns off and the mode is set to 0, waiting for the selection button to be pressed
					}else if(count>20001){
							BSP_LED_Off(LED4);
							mode = 0;
						}
					//Once the selection button has been pressed a random number is generated and the mode is changed
					}else if(mode==2){
							randTime = HAL_RNG_GetRandomNumber(&Rng_Handle)%(5000)+17000;
							mode=3;
					//Count is used to wait for the random time, and then the LED is turned on and the Mode is changed
					}else if(mode==3){
						count++;
						if(count==randTime){
							count=0;
							BSP_LED_On(LED5);
							mode=4;
						}
					//The timer is started and being displayed in this mode, and waiting for the selection button
					}else if(mode==4){
							count++;
							/*The reason for this check is because if LCD is getting refreshed for every single count increase
								the LCD will be constantly updating and the buttons will not be registered as pressed because
								LCD will be halfway being refreshed til the next interrupt is triggered cause the LCD to once again be triggered
								Leading to an endless loop where none of the button presses are read
								Furthermore 15 was chosen because it was found to be the optimal refresh rate
								Any lower and the LCD will not fully update all it's digits causing the LCD to be read to the 0.01s rather than the 0.001s 
								As per the requirements, and the LCD will not being update at exactly 1 second but rather slower, decent amount of testing went into this
								Any higher than the number will no longer be accurate
								This was timed alongside a stopwatch and it was nearly on time
								Furthermore if the user would like a more accurate representation it's reccomended that this constant update be removed
								And only the final result is displayed
						*/	
						if(count%15==0){
								BSP_LCD_GLASS_Clear();
								sprintf(lcd_buffer,"%d",count);
								BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							}
					}
				}
}
	 
void reset (){
	
	
}
	

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																															
	

		//clear the timer counter at the end of call back to avoid interrupt interval variation!  in stm32l4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this macro is defined in stm32l4xx_hal_tim.h

}


static void Error_Handler(void)
{
 
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
