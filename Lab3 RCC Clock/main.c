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


this program: 

1. This project needs the libraray file i2c_at2464c.c and its header file. 
2. in the i2c_at2464c.c, the I2C SCL and SDA pins are configured as PULLUP. so do not need to pull up resistors (even do not need the 100 ohm resisters).
NOTE: students can also configure the TimeStamp pin 	

*/




/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef  pI2c_Handle;
int mode=0;
RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range

   
  uint8_t s;
	uint8_t m;
	uint8_t h;

char lcd_buffer[6];    // LCD display buffer
char timestring[10]={0};  //   
char datestring[6]={0};


int8_t wd=4, dd=31, mo=10, yy=17, ss = 0x00, mm = 0x01, hh = 0x08; // for weekday, day, month, year, second, minute, hour
int8_t ss2 = 0x00, mm2 = 0x00, hh2 = 0x00;
__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed

__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed;  // button pressed 
__IO uint8_t  sel_held;   // if the selection button is held for a while (>800ms)

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void saveTime(void);

void RTC_Config(void);
void RTC_AlarmAConfig(void);
uint8_t showtime[50];
//Used for string editing
char temp[8];
static void RTC_TimeShow(uint8_t* showtime)
{
 
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  sprintf((char*)showtime,"%02d%02d%02d",RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes, RTC_TimeStructure.Seconds);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)showtime);
} 

uint8_t feild = 3;  // Variable used to indicate which feild (hour-minute-seconds) to increment. 1 = hours, 2 = minutes, 3 = seconds
int editstate = -1; // Variable used to change the state of the program into the edit mode. editstate = -1 indicate we are not editing the feilds, editstate = 1 indicate we are editing the feilds.
uint8_t edit = 0;   // Variable used to to indicate when we want the to input the new feilds into the RTC clock that is being displayed

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


void EditTime(feild){ // This function increments the feilds of the clock based on what value is passed through feild, 1= hours , 2= minutes, 3= seconds
	//edit = 1;           
	if(feild==1){
		if(hh2 + 0x01 >0x18){ // if incrementing will make the field greater of equal to 24
			// set the time back to 00:00:00 because time can't go past 24
			hh2 = 0x00;					

		}
		else if(hh2 + 0x01 <=0x18){  // if the increment will make the feild less than or equal to 24 
			// increment the hour by 1 
			hh2 += 0x01;


		}
	 // Here we are adding a lable with the current vaule of the feild we are editing
	 BSP_LCD_GLASS_Clear();
	 sprintf(temp, "Hr %d",hh2);
	 sprintf(lcd_buffer,"%s",temp);
	 BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
	}
	
	// feild 2 is minutes
	if(feild==2){
		if(mm2 + 1 >= 0x3c){ // if incrementing will make the field greater of equal to 60
			// Since incretmenting the minutes results in 60 we reset minutes to 00
			mm2 = 0x00;
		}
		else if(mm2 + 1 < 0x3c){ // if the increment will make the feild less than 60
			// Since incrementing the will not result in minutes == 60, simpliy increment the minutes by 1 
			mm2 += 0x01; 
		}
		
	// Here we are adding a lable with the current vaule of the feild we are editing
	BSP_LCD_GLASS_Clear();
	sprintf(temp, "Mn %d",mm2);
	sprintf(lcd_buffer,"%s",temp);
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);

	}
	// feild 3 is the seconds
	if(feild ==3){
		if(ss2 + 1 >= 0x3c){ // if incrementing will make the field greater of equal to 60
			// Since incretmenting the seconds results in 60 we reset seconds to 00 
			ss2 = 0x00;
		}
		else if(ss2 + 1 < 0x3c) // if the increment will make the feild less than 60
			// Since incrementing the will not result in seconds == 60, simpliy increment the seconds by 1 
			ss2 += 0x01;
		
	// Here we are adding a lable with the current vaule of the feild we are editing
	BSP_LCD_GLASS_Clear();
	sprintf(temp, "Sc %d",ss2);
	sprintf(lcd_buffer,"%s",temp);
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
	}
	
}
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

	leftpressed=0;
	rightpressed=0;
	uppressed=0;
	downpressed=0;
	selpressed=0;
	sel_held=0;
	

	HAL_Init();
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
											
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest, !!!This line need to be after systemClock_config()

	
	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);

	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	
	HAL_Delay(1000);


//configure real-time clock
	RTC_Config();
	
	RTC_AlarmAConfig();
	
	I2C_Init(&pI2c_Handle);


//*********************Testing I2C EEPROM------------------
/*
	//the following variables are for testging I2C_EEPROM
	uint8_t data1 =0x67,  data2=0x68;
	uint8_t readData=0x00;
	uint16_t EE_status;


	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, data1);

  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);
	

*/
//******************************testing I2C EEPROM*****************************	
		

  /* Infinite loop */
  while (1)
  {
		
		if(mode==0){
			RTC_TimeShow(showtime);
		}
			//the joystick is pulled down. so the default status of the joystick is 0, when pressed, get status of 1. 
			//while the interrupt is configured at the falling edge---the moment the pressing is released, the interrupt is triggered.
			//therefore, the variable "selpressed==1" can not be used to make choice here.
			if (BSP_JOY_GetState() == JOY_SEL && mode!=1) {
					SEL_Pressed_StartTick=HAL_GetTick(); 
					while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
						if ((HAL_GetTick()-SEL_Pressed_StartTick)>800 && leftpressed==0) {
							mode=1;	
							// Display the alternate time here
							
							BSP_LCD_GLASS_Clear();
							sprintf(temp, "WD %d",wd);
							sprintf(lcd_buffer,"%s",temp);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							HAL_Delay(500);
							BSP_LCD_GLASS_Clear();
							sprintf(temp, "DD %d",dd);
							sprintf(lcd_buffer,"%s",temp);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							HAL_Delay(500);
							BSP_LCD_GLASS_Clear();
							sprintf(temp, "M %d",mo);
							sprintf(lcd_buffer,"%s",temp);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							HAL_Delay(500);							
							BSP_LCD_GLASS_Clear();
							sprintf(temp, "YY %d",yy);
							sprintf(lcd_buffer,"%s",temp);
							BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
							mode=0;
							HAL_Delay(500);
						} 
					}
			}				
			
/*==============================================================			

//==============================================================					
			if (selpressed==1)  {
	
					selpressed=0;
			} 
//==============================================================			

//==============================================================		 
			if (leftpressed==1) {

							
					leftpressed=0;
			}			
//==============================================================			

//==============================================================							
			if (rightpressed==1) {

			
					rightpressed=0;
			}
//==============================================================			

//==============================================================						
			//switch (myState) { 
				
				
				
			//} //end of switch					
		
*/

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

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
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz


void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//****2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
		
	
					
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
				
				RTCHandle.Init.AsynchPrediv = 127; 
				RTCHandle.Init.SynchPrediv = 249; 
				
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
			
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
					BSP_LCD_GLASS_Clear(); 
					BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 	
				}
	
	
	
	
	//****3.***** init the time and date
				
				
 	//	*****************Students: please complete the following lnes*****************************
	//	****************************************************************************************		
				RTC_DateStructure.Year = 0x14;
				RTC_DateStructure.Month = RTC_MONTH_APRIL;
				RTC_DateStructure.Date = 0x14;
				RTC_DateStructure.WeekDay = RTC_WEEKDAY_MONDAY;
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
				} 
  
				//Sets our RTC structure using our variables
  
				RTC_TimeStructure.Hours = hh;  
				RTC_TimeStructure.Minutes = mm; 
				RTC_TimeStructure.Seconds = ss;
				RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
				}	
	  
 //********************************************************************************/



				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to ‘1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}

void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;


	
	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask =  RTC_WEEKDAY_MONDAY;
	
	
	
  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
	//		BSP_LCD_GLASS_Clear(); 
	//		BSP_LCD_GLASS_DisplayString((uint8_t *));
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

//You may need to disable and enable the RTC Alarm at some moment in your application
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */

void saveTime(){

	/* Get the RTC current Time */
  HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
	uint8_t readData1;
	//Reads the first slot of memory for Seconds, Minutes, Hours
	s=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);
	m=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);
	h=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+2);
	//Writes the memory onto the second memory slot
	I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+3, s);
	I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+4, m);
	I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+5, h);
	//Writes the current time 
	I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, RTC_TimeStructure.Seconds);
	I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1, RTC_TimeStructure.Minutes);
	I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+2, RTC_TimeStructure.Hours);
	
}



//Mode 3 Is edit
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						selpressed=1;	
						//Makes the saveTime function start
						if(mode==0){
							saveTime();
							
						}
						break;	
			case GPIO_PIN_1:     //left button						
							leftpressed=1;
							if(mode==1){
								mode=0;
								RTC_TimeShow(showtime);
							}else if(mode==2){
								mode=0;
								RTC_TimeShow(showtime);
							}else if(mode==3){   // If the left button is pressed when we are in the edit state then we increment feild which tells us which feild we want to edit
								if(feild <=3){
										feild += 1;
									}
								else if(feild >3){
									feild = 1;
								}
								// Lables to indicate which feild we are editing
								if(feild ==1){
									BSP_LCD_GLASS_Clear();
									BSP_LCD_GLASS_DisplayString((uint8_t*)"Hour");
									HAL_Delay(500);
									BSP_LCD_GLASS_Clear();
									
								}
								if(feild == 2){
									BSP_LCD_GLASS_Clear();
									BSP_LCD_GLASS_DisplayString((uint8_t*)"Minutes");
									HAL_Delay(500);
									BSP_LCD_GLASS_Clear();
								}
								if(feild ==3){
									BSP_LCD_GLASS_Clear();
									BSP_LCD_GLASS_DisplayString((uint8_t*)"Second");
									HAL_Delay(500);
									BSP_LCD_GLASS_Clear();
								}
								
								
							}else{
								//Displaying The last two times the program saved to the EEPROM
								mode=2;
								//Reading the first memory Slot
								s =I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);
								m =I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);
								h=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+2);
								BSP_LCD_GLASS_Clear();
								HAL_Delay(50);
								//Displays the Message
								sprintf(lcd_buffer,"%s","LastTm");
								BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
								HAL_Delay(500);
								if(mode==2){
								BSP_LCD_GLASS_Clear();
								sprintf(lcd_buffer,"%02d%02d%02d",h,m,s);
								BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
								
								HAL_Delay(500);
								}
								if(mode==2){
								BSP_LCD_GLASS_Clear();
								sprintf(lcd_buffer,"%s","2ndTm");
								BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
								HAL_Delay(500);
								}
									if(mode==2){
								//Displays the Second Memory Slot from the memory
								s =I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+3);
								m =I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+4);
								h=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+5);
								sprintf(lcd_buffer,"%02d%02d%02d",h,m,s);
								BSP_LCD_GLASS_Clear();
								BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
								HAL_Delay(500);
								}
							}
							



							break;
			case GPIO_PIN_2:    //right button
							//rightpressed=1;	
			
							if(mode==0){ // Mode == 0 indicates that we have pressed the right button for the first time 
								mode=3; // we set mode to 3 so that the next time mode is pressed we update the clock to the editted values 
								//Here we are saving the time of the clock when the right button is pressed for the first time.
								// We need to this specific time becuase the RTC is running in the background continously
								// ss2,mm2,hh2 the temp variables where well will save and edit the time
								ss2 = ss;
								mm2 = mm;
								hh2 = hh;
								
							}else if(mode==3){ //If mode is set to 3 then we want to upload the edited values of our temp variables ss2,mm2 and hh2 to the diplayed clock
								// We simply make the clock variables equal to our edited time variables
								hh = hh2;
								mm = mm2;
								ss= ss2;
								RTC_Config(); // we run RTC_Config to reset the RTC with our new values
								//edit = 0; 
								mode=0; // we set mode back to 0 to go back to the inital state of the DFSM
							}
			
							break;
			case GPIO_PIN_3:    //up button							
							uppressed=1;
							if(mode==3){ // If mode is == 3 we are are in the time edit state
								EditTime(feild); // in the edit state we run EditTime which allows us to edit the RTC clock values, feild indicates if we are editing hours, minutes or seconds
							}
							break;
			
			case GPIO_PIN_5:    //down button						
							//BSP_LCD_GLASS_Clear();
							//BSP_LCD_GLASS_DisplayString((uint8_t*)"down");
							break;
			case GPIO_PIN_14:    //down button						
							//BSP_LCD_GLASS_Clear();
						  //	BSP_LCD_GLASS_DisplayString((uint8_t*)"PE14");
							break;			
			default://
						//default
			if(mode==0){
			RTC_TimeShow(showtime);
			}
						break;
	  } 
}



void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  BSP_LED_Toggle(LED5);
	//RTC_TimeShow();
		 // Refresh the screen
	
	
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
