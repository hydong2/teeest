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
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"//??
#include "string.h"//dong
#include "stdlib.h"//dong
#include "stm32f4xx_hal.h"//dong
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CLCD.h"
#include <stdio.h>
#include "7SEG.h"
#include <math.h>// DAC sin wave
#include "rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	ZERO,
	YEAR,
	MONTH,
	DATE,
	AMPM,
	HOUR,
	MINUTE,
	SECOND, //6
}_SetTime;
typedef enum{ // use only alarm setting
aMONTH,
aDATE,
aAMPM,
aHOUR,
aMINUTE,
}_SetAlarm ;
typedef enum{
	NO_SET,
	TIME_SET,
	ALARM_SET,
}_MODE;

typedef enum{
	VIEW_SET,
	VIEW_NORMAL,
	VIEW_ALARM,
	//BELL//3
}_VIEW;
typedef enum{
	ALARM_ONE=1,
	ALARM_TWO,
}_ALARM;
typedef enum{
	NONE,
	SW_ONE,
	SW_TWO,
	SW_THREE,
	SW_FOUR,
}_SW_BUTT0N;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define LCD_ADDRESS (0x27 << 1 //dong
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t rx3_data; // UART

//-----------------------------RTC Input code-------------------------------------------------//
RTC_TimeTypeDef sTime;  //set
RTC_TimeTypeDef aTime; //alarm

RTC_DateTypeDef sDate; //set
RTC_DateTypeDef aDate; //alarm
//-------------------------------------------------------------------------------------------//



char ampm[2][3] = {"AM", "PM"}; //2차원 문자 배열// AM = 0, PM = 1 ?

uint8_t SW1_PIN=0;// Butten press, pull check //it was "user_pressed_flag"
//uint8_t SW1_pulled=0;//it was "user_pulled_flag" //not used??
uint8_t SW2_PIN=0;
uint8_t SW3_PIN=0;
uint8_t SW4_PIN=0;

_SetTime set_time = ZERO; //setmode 0
_MODE mode=NO_SET;  //mode set
_VIEW view=VIEW_NORMAL;
_ALARM alarm=ALARM_ONE;
_SetAlarm setalarm =aMONTH;
_SW_BUTT0N sw_button=NONE;
_SW_BUTT0N direction;
_SW_BUTT0N sw_button_before=NONE;
//-------------------------------------ctrl c+v-----https://eteo.tistory.com/91----------------//
uint32_t old_tick=0;
uint32_t current_tick=0;

uint32_t old_alarm_tick=0;
uint32_t current_alarm_tick=0;

uint32_t btn_tick=0;

uint8_t alarm_on=0;

//------------------------------------------------------------------------------------------//
typedef struct{
	uint8_t ch;
	uint8_t state;
	uint32_t repeat_time;
	uint32_t pre_time;

}button_obj_t;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++//

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
void Error_Handler(void);//dong
/* USER CODE BEGIN PFP */
_SW_BUTT0N getdirection();
void move_cur_time(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, _SW_BUTT0N  sw_button);

void buttonObjCreate(button_obj_t *p_obj, uint32_t repeat_time);
bool buttonObjGetClicked(button_obj_t *p_obj, uint32_t pressed_time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//------------------------------------------------------------uart??------------------------//
/*int _write(int file, char* p, int len)
{
	HAL_UART_Transmit(&huart3,  p,  len, 10);
	return len;
}
*/
//-------------------------------------------------------------------------------------------//

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//volatile uint16_t adcval[4];  // adc control ?
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
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  //---------------------------------------------------------right led --//use PMW -> TIM3_CH1,2,3-----//
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  //---------------------------------------------------------left led---//use PMW -> TIM4_CH1,2,3-----//
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  //----------------------------------------------------------------------------------------------//
	 //HAL_UART_Receive_IT(&huart3, &rx3_data, 1);  //lesson 2-3 make uart Intterupt
	 HAL_TIM_Base_Start_IT(&htim7);// system clock?--------------------------TIM7----1.0s------------//

	 //---------------------CLCD Setting--------------------------------------------------------//
	 CLCD_GPIO_Init();
	 CLCD_Init();
	 //------------------------------------------------------------------------------------------//
	 //CLCD_Puts(0,0, "Welcome to");
	 //CLCD_Puts(0,1, "M-HIVE");
	 CLCD_Clear();

	 _7SEG_GPIO_Init();// _7SEG_GPIO Setting-----------------------------------------------//

 //-------------------------------------------led PWM Control---------if use this, go down "TIM3,4 Interrupt"------//
	 /*
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  //R led
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  //L led
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	 */
 //-----------------------------------------------------------------------------------------------------------//


	 //HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); //sub moter

	 //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //buzzer

	  //HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);  // dc moter
	  //HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4); // dc moter

	 //HAL_ADC_Start_DMA(&hadc1, adcval[0], 4);
	 //HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

	 //TIM10->CCR1 = 1500;  //sub moter

	 //TIM5->CCR1 = 0;     // dc moter
	 //TIM5->CCR4 = 5000;  // dc moter

 //----------------------------------new time code start-------------------------------------------------//
button_obj_t push_btn; //+++++++
uint32_t btn_cnt = 0;
buttonObjCreate(&push_btn, 1000); // (button_obj_t *p_obj, uint32_t repeat_time);
 // --------------------------------------------------------Read RTC and save in sTime?------------//
	 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

	 HAL_RTC_GetTime(&hrtc, &aTime, RTC_FORMAT_BCD);
	 HAL_RTC_GetDate(&hrtc, &aDate, RTC_FORMAT_BCD);
//------------------------------------------------------------------------------------------------//
	 uint8_t toggle=0; //ctrl c+v
	 //uint8_t blink_time=0; //not toggle use this!! go go!
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 //uint8_t a = 0;
	 //float f = 1.234;
	 uint8_t str[20]; // used CLCD
	 uint8_t tmpDate[20]; // CLCD
	 uint8_t tmpTime[20]; // CLCD
	 uint8_t tmpMode[20];
	 //uint16_t ccr = 0;	//led pwm control
	 //uint16_t psc = 1000; //??
	 //uint8_t ud_flag = 0; //??
	 //uint16_t dacval = 0;//dac control
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$44//

  while (1)
  {
	  //++++++ button test++++++//
	  current_tick=HAL_GetTick();
	  getBtn();
	  if(buttonObjGetClicked(&push_btn, 100) == true) //pressed_time
	  {
		  btn_cnt++;
		  sprintf(str, "btnClicked:%d", btn_cnt);
		  CLCD_Puts(0, 0, str);
		  sprintf(str, "true");
		  CLCD_Puts(0, 1, str);

	  }else{
		  sprintf(str, "false");
		  CLCD_Puts(0, 1, str);
	  }
//-----------------------------------------------------------------------------------CLCD main view--------------//

  //-------------------------------------------------------ALARM mode END----------------------------//
//****************************************END WHILE****************************--------------move code CV end---------------//

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }//WHILE END

  //==========================================================================버튼?���?? ?���?? ?��?��?���??================================//

  //========================================================================================================//
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */
//++++++++++++++++++++I con't button control++++++++++++++//
void buttonObjCreate(button_obj_t *p_obj, uint32_t repeat_time)
{
	//p_obj -> ch = ch;
	p_obj -> pre_time = btn_tick;
	p_obj -> state = 0;
	p_obj -> repeat_time = repeat_time;
}
bool buttonObjGetClicked(button_obj_t *p_obj, uint32_t pressed_time)//push_btn, 100
{
	bool ret = false;

	switch(p_obj->state)
	{
	case 0:
		//if(buttonGetPreseed(p_obj->ch)==ture)
		if(sw_button==SW_ONE)
		{
			p_obj->state = 1;
			p_obj->pre_time = btn_tick;// = Gettick;
		}
		break;

	case 1:
		//if(buttonGetPressed(p_obj->ch)==true)
		if(sw_button==SW_ONE)
		{
			if(current_tick-p_obj->pre_time >=pressed_time){//pressed_time 이상 누르면 참값 보냄
				ret = true; //------------------------------------clicked

				p_obj->state = 2;
				p_obj->pre_time = current_tick; // 참 보냈을 때의 시간
			}


			//btn_tick = current_tick;

		}
		else
		{
			p_obj->state = 0;
		}
		break;

	case 2:
		//if(buttonGetPressed(p_obj->ch)==true)
		if(sw_button==SW_ONE)
		{
			if(current_tick-p_obj->pre_time >= p_obj->repeat_time) // 다시 참 내보내기
			{
				p_obj->state = 1;
				p_obj->pre_time = current_tick;
			}

		}
		else
		{
			p_obj->state = 0;
		}
		break;
	}

	return ret;
}
//=----------------------------my---------------------------------------code butten------//
void getBtn(){
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)){
		sw_button=SW_ONE;
	}
	else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)){
		sw_button=SW_TWO;
	}
	else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)){
		sw_button=SW_THREE;
	}else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)){
		sw_button=SW_FOUR;
	}else{
		sw_button=NONE;
	}
}
	  void move_cur_time(RTC_TimeTypeDef *Time, RTC_DateTypeDef *Date, _SW_BUTT0N  sw_button){// EX)move_cur_time(&sTime, button);
		switch(sw_button){
		case SW_ONE:
			if(mode==TIME_SET){
				set_time++;
				if(set_time > SECOND) set_time = YEAR;
			}else if(mode==ALARM_SET){
				setalarm++;
				if(setalarm > aMINUTE) setalarm = aMONTH;
			}else if(mode==NO_SET){
				view++;
				if(view > VIEW_ALARM) view = VIEW_NORMAL;
			}
				break;

		case SW_TWO:
			if(mode==TIME_SET){
				if(set_time==AMPM){
					Time->TimeFormat ^= 1;
				}else if(set_time==HOUR){
					Time->Hours++;
					if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 1;// I don't understand ;; why not 0??
					if(!IS_RTC_HOUR24(Time->Hours)) Time->Hours = 1;
				}else if(set_time==MINUTE){
					Time->Minutes++;
					if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 0;
				}else if(set_time==SECOND){
					Time->Seconds++;
					if(!IS_RTC_SECONDS(Time->Seconds)) Time->Seconds = 0;
				}else if(set_time==YEAR){
					Date->Year++;
					if(!IS_RTC_YEAR(Date->Year)) Date->Year = 0;
				}else if(set_time==MONTH){
					Date->Month++;
					if(!IS_RTC_MONTH(Date->Month)) Date->Month = 1;// maybe it is not problem
				}else if(set_time==DATE){
					Date->Date++;
					if(!IS_RTC_DATE(Date->Date)) Date->Date = 1;
				}
			}else if(mode==ALARM_SET){
				if(setalarm==aAMPM){
					Time->TimeFormat ^= 1;
				}else if(setalarm==aHOUR){
					Time->Hours++;
					if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 1;// I don't understand ;; why not 0??
					if(!IS_RTC_HOUR24(Time->Hours)) Time->Hours = 1;
				}else if(setalarm==aMINUTE){
					Time->Minutes++;
					if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 0;
				}else if(setalarm==aMONTH){
					Date->Month++;
					if(!IS_RTC_MONTH(Date->Month)) Date->Month = 1;// maybe it is not problem
				}else if(setalarm==aDATE){
					Date->Date++;
					if(!IS_RTC_DATE(Date->Date)) Date->Date = 1;
				}
			}


			break;
	//---------------------I did Year,Month, Date code add!!------------------MY------//

	//--------------------------------------------//


		case SW_THREE:
			if(mode==TIME_SET){
				if(set_time==AMPM){
					Time->TimeFormat ^= 1;
				}else if(set_time==HOUR){
					Time->Hours--;
					if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 12;
					if(!IS_RTC_HOUR24(Time->Hours)) Time->Hours = 23;
				}else if(set_time==MINUTE){
					Time->Minutes--;
					if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 59;
				}else if(set_time==SECOND){
					Time->Seconds--;
					if(!IS_RTC_SECONDS(Time->Seconds)) Time->Seconds = 59;
				}else if(set_time==YEAR){
					Date->Year--;
					if(!IS_RTC_YEAR(Date->Year)) Date->Year = 99;
				}else if(set_time==MONTH){
					Date->Month--;
					if(!IS_RTC_MONTH(Date->Month)) Date->Month = 12;// maybe it is not problem
				}else if(set_time==DATE){
					Date->Date--;
					if(!IS_RTC_DATE(Date->Date)) Date->Date = 31;
				}
			}else if(mode==ALARM_SET){
				if(setalarm==aAMPM){
					Time->TimeFormat ^= 1;
				}else if(setalarm==aHOUR){
					Time->Hours--;
					if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 12;// I don't understand ;; why not 0??
					if(!IS_RTC_HOUR24(Time->Hours)) Time->Hours = 23;
				}else if(setalarm==aMINUTE){
					Time->Minutes--;
					if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 59;
				}else if(setalarm==aMONTH){
					Date->Month--;
					if(!IS_RTC_MONTH(Date->Month)) Date->Month = 12;// maybe it is not problem
				}else if(setalarm==aDATE){
					Date->Date--;
					if(!IS_RTC_DATE(Date->Date)) Date->Date = 31;
				}
			}


			break;
		case NONE:
			break;
		//case UNKNOWN:
		//	break;
		}
		sw_button=NONE;
		 HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		 HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
	}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		//HAL_UART_Receive_IT(&huart3,  &rx3_data, 1);
		//HAL_UART_Transmit(&huart3, &rx3_data, 1, 10);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static unsigned char cnt = 0;
	uint8_t blink_time=0;
	//static unsigned char blink_cnt = 0;
	if(htim->Instance == TIM7)  //1.0s
	{
		blink_time^=1;
		cnt++;

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // if switch in action
{
	if(GPIO_Pin == GPIO_PIN_3)  // SW1 mode change
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // switch push -> led light

		//sw_button=SW_ONE;

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)){ // CTRL CV...// if my SW1 pressed
					//user_pulled_flag=0;
					//SW1_pulled=0;
					//user_pressed_flag=1;
			sw_button=SW_ONE;
					SW1_PIN = 1;

					old_tick=HAL_GetTick(); // just pressed time
					current_tick=HAL_GetTick();// continue check now time = Current time qudtls
				}else {
					//user_pulled_flag=1;
					//SW1_pulled=0;
					//user_pressed_flag=0;
					SW1_PIN=0;
					sw_button=NONE;
				}
//--------------------------------------------------------------------------------------------------------------------//sw2
	}
	if(GPIO_Pin == GPIO_PIN_15) //SW2 time up // AMPM Change
		{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);// switch push -> led light
		//--------------------------------------------------if my SW2 pressed
		//sw_button=SW_TWO;//Just 1 push check
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)){
							SW2_PIN=1;//long push check
							sw_button=SW_TWO;

							old_tick=HAL_GetTick(); // just pressed time
							current_tick=HAL_GetTick();
						}else {
							SW2_PIN=0;
							sw_button=NONE;
						}

//--------------------------------------------------------------------------------------------------------------------//sw3
		}
	if(GPIO_Pin == GPIO_PIN_4) //SW3 time down
		{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);// switch push -> led light
		//----------------------------------------------- if my SW3 pressed
		//sw_button=SW_THREE;//Just 1 push check
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)){
							SW3_PIN=1;//long push check
							sw_button=SW_THREE;
							old_tick=HAL_GetTick(); // just pressed time
							current_tick=HAL_GetTick();
						}else {
							SW3_PIN=0;
							sw_button=NONE;
						}
		}
//--------------------------------------------------------------------------------------------------------------------//sw4
	if(GPIO_Pin == GPIO_PIN_10) //SW4
		{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);// switch push -> led light
		//----------------------------------------------- if my SW3 pressed
		//sw_button=SW_FOUR;//Just 1 push check
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)){
							SW4_PIN=1;//long push check
							sw_button=SW_FOUR;

							old_tick=HAL_GetTick(); // just pressed time
							current_tick=HAL_GetTick();
						}else {
							SW4_PIN=0;
							sw_button=NONE;
						}
		}
}
uint32_t HAL_GetTick(void)
{
  return uwTick;
}
//-======================================================//
void LCD_SendCommand(uint8_t command) {
    HAL_I2C_Mem_Write(&hi2c1, LCD_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, &command, 1, HAL_MAX_DELAY);
}

void LCD_SendData(uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, LCD_ADDRESS, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_SendCommand(0x03);
    HAL_Delay(5);
    LCD_SendCommand(0x03);
    HAL_Delay(1);
    LCD_SendCommand(0x03);
    HAL_Delay(5);
    LCD_SendCommand(0x02);
    HAL_Delay(1);

    LCD_SendCommand(0x20 | 0x08 | 0x00); // 4-bit mode, 2-line, 5x8 font
    HAL_Delay(1);
    LCD_SendCommand(0x08 | 0x04); // display off
    HAL_Delay(1);
    LCD_SendCommand(0x01); // clear display
    HAL_Delay(5);
    LCD_SendCommand(0x06 | 0x02); // cursor increment
    HAL_Delay(1);
    LCD_SendCommand(0x0C | 0x00 | 0x00); // display on
    HAL_Delay(1);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01); // clear display
    HAL_Delay(5);
}

void LCD_SetCursor(uint8_t col, uint8_t row) {
    uint8_t address = 0x80 | (col + (row * 0x40));
    LCD_SendCommand(address);
}

void LCD_Print(const char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    LCD_Init();

    float volt;
    int sensorPin = A0;
    char str[16];

    while (1) {
        volt = 0;
        for (int i = 0; i < 800; i++) {
            volt += ((float)analogRead(sensorPin) / 1023) * 5;
        }
        volt = volt / 800;
        volt = round_to_dp(volt, 1);

        LCD_Clear();
        LCD_SetCursor(0, 0);
        sprintf(str, "%.1f V", volt);
        LCD_Print(str);

        HAL_Delay(1000); // 1초마다 LCD를 업데이트합니다.
    }
}

float round_to_dp(float in_value, int decimal_place) {
    float multiplier = powf(10.0f, decimal_place);
    in_value = roundf(in_value * multiplier) / multiplier;
    return in_value;
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
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
