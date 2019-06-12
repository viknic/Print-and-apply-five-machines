
/*
 * stepper motors pins
 * switched positions are
 * 1 off
 * 2 on
 * 3 on
 * 4 off
 * 5 on
 * 6 on
 * 7 off
 * 8 on
 */




/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */
//////mode 1 IS HORIZONTAL AND MODE 0 IS VERTICAL
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
/* Private includes ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);


void sensor_check(void);
/* USER CODE BEGIN PFP */

uint8_t rollback_home;
uint8_t carton_sen;
uint8_t uartchar[1];
#define servo_mtr 1
#define roll_back_mtr 2

#define print_pos 4
#define _apply_pos 5
#define _vertical_print_pos 6

#define ___home 9

#define Emergency_Light HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
#define 	Emergency_Light_toggle()  HAL_GPIO_TogglePin(GPIOA,PAD_Relay_Pin);\
																			__delay_ms(200);
#define Emergency_Light_Reset() HAL_GPIO_WritePin(GPIOA,PAD_Relay_Pin,SET)


uint8_t SPS;
//uint8_t SUTION_on = 1400;
uint32_t PRINT_on;
uint32_t PIPE_off;
uint32_t PIPE_on;
uint32_t Suction_On_time;
uint32_t CARTON_PRESS_DELAY;
uint32_t App_Stroke_mm;
uint32_t App_Stroke_mm_Operation;
uint32_t uartchar_rec_buffer[50],count_buff,_buff;
//FLAG FOR UPDATE MODE

uint32_t _suction_on_ = 0;
uint8_t _print_signal[5];

uint16_t Rollback_Read;

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}


uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}
void __delay_ms(unsigned int _msec)
{
	for(unsigned int ____i=0;____i<_msec;____i++){
		DWT_Delay_us(1000);
	}
}

/* USER CODE END 0 */

void pad_position(unsigned char _position)
{
	//uint8_t read_sen;
}

void motor_move_mtrid_step_dir_frequency(unsigned char __motor_id, unsigned int __steps, unsigned char __dir,unsigned int __frequency,unsigned int _mode)
{
////////mode 1 IS HORIZONTAL AND MODE 0 IS VERTICAL///////////////
	uint32_t step___c = 0;
	uint32_t STEP__C1 = 0;
	if(__dir == ___home)
	{
    uint32_t X1 =	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    while(X1 != 0)
  	{
     HAL_GPIO_WritePin(GPIOB,M21_Pin,GPIO_PIN_RESET);
     X1 =	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_SET);
  	 DWT_Delay_us(110);
  	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_RESET);
  	 DWT_Delay_us(110);
  	}
	}
	uint32_t profilling_count_1[11];
	if(__dir == 1)
	{
		HAL_GPIO_WritePin(GPIOB,M21_Pin,GPIO_PIN_SET);
	}
	else if(__dir == 0)
	{
		HAL_GPIO_WritePin(GPIOB,M21_Pin,GPIO_PIN_RESET);
	}
	if(__steps > 25000)
	{
		profilling_count_1[0]	= 110  ;//150; 60;
		profilling_count_1[1]	= 100  ;//150; 55;
		profilling_count_1[2]	= 90  ;//150; 50;
		profilling_count_1[3]	= 80  ;//150; 45;
		profilling_count_1[4]	= 70  ;//150; 40;
		profilling_count_1[5]	= 60  ;//150; 35;
		profilling_count_1[6]	= 50  ;//150; 30;
		profilling_count_1[7]	= 40  ;//150; 25;
		profilling_count_1[8]	= 30  ;//150; 20;
		profilling_count_1[9]	= 20  ;//150; 15;ME
		profilling_count_1[10]	= 20  ;//150; 10;
		profilling_count_1[11]	= 20  ;//150;  5;
	}
		else if(__steps <= 25000 && __steps >3000)
		{
			profilling_count_1[0]	= 110  ;//150; 60;
			profilling_count_1[1]	= 100  ;//150; 55;
			profilling_count_1[2]	= 90  ;//150; 50;
			profilling_count_1[3]	= 80  ;//150; 45;
			profilling_count_1[4]	= 70  ;//150; 40;
			profilling_count_1[5]	= 60  ;//150; 35;
			profilling_count_1[6]	= 50  ;//150; 30;
			profilling_count_1[7]	= 40  ;//150; 25;
			profilling_count_1[8]	= 30  ;//150; 20;
			profilling_count_1[9]	= 20  ;//150; 15;ME
			profilling_count_1[10]	= 20  ;//150; 10;
			profilling_count_1[11]	= 20  ;//150;  5;

		}
	else
		{
		profilling_count_1[0]	= 110  ;//150; 60;
		profilling_count_1[1]	= 100  ;//150; 55;
		profilling_count_1[2]	= 90  ;//150; 50;
		profilling_count_1[3]	= 80  ;//150; 45;
		profilling_count_1[4]	= 70  ;//150; 40;
		profilling_count_1[5]	= 60  ;//150; 35;
		profilling_count_1[6]	= 50  ;//150; 30;
		profilling_count_1[7]	= 30  ;//150; 25;
		profilling_count_1[8]	= 20  ;//150; 20;
		profilling_count_1[9]	= 20  ;//150; 15;ME
		profilling_count_1[10]	= 20  ;//150; 10;
		profilling_count_1[11]	= 20  ;//150;  5;
			}
			uint32_t acceleration_count_1;
			uint32_t acceleration_count_2;
			uint32_t acceleration_count_3;
			uint32_t acceleration_count_4;
			uint32_t acceleration_count_5;
			uint32_t acceleration_count_6;
			uint32_t acceleration_count_7;
			uint32_t acceleration_count_8;
			uint32_t acceleration_count_9;
			uint32_t acceleration_count_10;

		    uint32_t fullspeed_count;

	    	uint32_t decelaration_count_1;
	    	uint32_t decelaration_count_2;
	    	uint32_t decelaration_count_3;
	    	uint32_t decelaration_count_4;
	    	uint32_t decelaration_count_5;
	    	uint32_t decelaration_count_6;
	    	uint32_t decelaration_count_7;
	    	uint32_t decelaration_count_8;
	    	uint32_t decelaration_count_9;
	    	uint32_t decelaration_count_10;
	    	uint32_t profile_persentage_diff;

		if(__motor_id == servo_mtr)
		{
//							for(unsigned int _step =0;_step<__steps;_step++)
////							{
//								HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_SET);
//								DWT_Delay_us(__frequency);
//								HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_RESET);
//								DWT_Delay_us(__frequency);
//								//DWT_Delay_us(profilling_count_1[_stepa] - (profilling_count_1[_stepa]/2));
//							}


			acceleration_count_1 = (__steps / 100);//200
			decelaration_count_1 = (__steps / 100);//200
			fullspeed_count = __steps -(acceleration_count_1*10+decelaration_count_1*10);
			profile_persentage_diff = __steps -(acceleration_count_1+decelaration_count_1+fullspeed_count);

			for(unsigned int _stepa = 0;_stepa<10;_stepa++){
				for(unsigned int _step =0;_step<acceleration_count_1;_step++)
				{
					HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_RESET);
					DWT_Delay_us(profilling_count_1[_stepa]/2);
				    HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_SET);
				    DWT_Delay_us(profilling_count_1[_stepa]);
				}
			}
			for(unsigned int _step = 0;_step<fullspeed_count;_step++)
			{

					HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_RESET);
					DWT_Delay_us(profilling_count_1[10]/2);
					HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_SET);
					DWT_Delay_us(profilling_count_1[10]);
					step___c ++;
					if(__dir == 1 && step___c >=2000 && _mode == 1)
					{
					  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);  //APPLY POS
					}
					else if(__dir == 0 && step___c >=2000 && _mode == 1)
					{
					 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET); //PRINT POS
					}
			}

			for(unsigned int _stepa = 10;_stepa>0;_stepa--){
				for(unsigned int _step =0;_step<decelaration_count_1;_step++)
				{
					HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_RESET);
					DWT_Delay_us(profilling_count_1[_stepa]/2);
					HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_SET);
					DWT_Delay_us(profilling_count_1[_stepa]);
//					STEP__C1 ++;
				}

			}

			for(unsigned int _step = 0;_step<profile_persentage_diff;_step++)
			{
				    HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_RESET);
					DWT_Delay_us(profilling_count_1[0]/2);
					HAL_GPIO_WritePin(GPIOB,M22_Pin ,GPIO_PIN_SET);
					DWT_Delay_us(profilling_count_1[0]);
			}
		}
}

void Servo_arm(uint32_t Steps,uint8_t _Dir,uint32_t _Freq)
{
	//  HAL_GPIO_WritePin(GPIOB, Ungrip_Relay_Pin|Suction_Relay_Pin|GPIO_PIN_15|Srvo1_Pin
  //  |M32_Pin|M31_Pin|M22_Pin|M21_Pin
  //  |M12_Pin|M11_Pin, GPIO_PIN_RESET);
	uint32_t i,Read_Sen;
	Read_Sen =	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
     if(_Dir == 1)
     {
 		HAL_GPIO_WritePin(GPIOB,M21_Pin,GPIO_PIN_SET);
     	for(i = 0;i<Steps;i++)
     	{
     	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_SET);
     	 DWT_Delay_us(_Freq);
     	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_RESET);
     	 DWT_Delay_us(_Freq);
     	}
     }
     if(_Dir == 0)
     {
    	 HAL_GPIO_WritePin(GPIOB,M21_Pin,GPIO_PIN_RESET);
     	for(i = 0;i<Steps;i++)
     	{
     	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_SET);
     	 DWT_Delay_us(_Freq);
     	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_RESET);
     	 DWT_Delay_us(_Freq);
     	}
     }
     if(_Dir == 2)//home
     {
     HAL_GPIO_WritePin(GPIOB,M21_Pin,GPIO_PIN_RESET);
     Read_Sen =	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
     while(Read_Sen == 0)
   	{
     Read_Sen =	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
   	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_SET);
   	 DWT_Delay_us(100);
   	 HAL_GPIO_WritePin(GPIOB,M22_Pin,GPIO_PIN_RESET);
   	 DWT_Delay_us(100);
   	}
     }
}
void Roll_back_ARm(uint32_t _Steps,uint8_t Dir,uint32_t Freq_)
{
	uint32_t i,Read_Sen;
	Read_Sen =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
     if(Dir == 1)
     {
     	HAL_GPIO_WritePin(GPIOB,M11_Pin,GPIO_PIN_SET);
     	for(i = 0;i<_Steps;i++)
     	{
     	 HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_SET);
     	 DWT_Delay_us(Freq_);
     	 HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_RESET);
     	 DWT_Delay_us(Freq_);
     	}
     }
     if(Dir == 0)
     {
     	HAL_GPIO_WritePin(GPIOB,M11_Pin,GPIO_PIN_RESET);
     	for(i = 0;i<_Steps;i++)
     	{
     	 HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_SET);
     	 DWT_Delay_us(Freq_);
     	 HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_RESET);
     	 DWT_Delay_us(Freq_);
     	}
     }
     if(Dir == 2)//home
     {
     	HAL_GPIO_WritePin(GPIOB,M11_Pin,GPIO_PIN_RESET);
    	Read_Sen =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
    	while(Read_Sen != 0)
         	{
    			Read_Sen =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
				 HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_SET);
				 DWT_Delay_us(45);
				 HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_RESET);
				 DWT_Delay_us(45);
         	}
     }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void ___wait(void)
{
	uint32_t Count__=0;
	for(unsigned int __delay_is=0;__delay_is<PRINT_on;__delay_is++)
	{

//		uint8_t Read_Sen =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
//		if(Read_Sen != 0)
//		{
//			HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_SET);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_RESET);
//		}
		/*
		 * make pipe blow On
		 */
		if(__delay_is == PIPE_on){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,SET); //pipe blow
		}
		/*
		 * make pipe Blow Off
		 */
		if(__delay_is == PIPE_off){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,RESET); //pipe blow
		}
		/*
		 * make suction On here
		 */
		if(__delay_is == Suction_On_time){
			HAL_GPIO_WritePin(GPIOB,Suction_Relay_Pin,SET);//suction ON
		}
		/*
		 *delay is here SPLIT DELAY MAKE ROLLBACK HERE
		 */
		HAL_GPIO_WritePin(GPIOB,M12_Pin,GPIO_PIN_RESET);

//		__delay_ms(1);
		HAL_GPIO_WritePin(GPIOB,M11_Pin,GPIO_PIN_RESET);//dir home
		for(unsigned int loop=0;loop<1000;loop++)
		{
			Rollback_Read = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
			Count__++;
			if(Count__ == 50)
			{
				Count__ = 0;
				if(Rollback_Read == 0)
				{
					//HAL_GPIO_TogglePin(GPIOB,M12_Pin);
				}
//				else
//				{
//					//toggle step
//
//				}
			}
			DWT_Delay_us(1);
		}
		/*
		 * roll back motor is at home position
		 */
		//ToDo
	}
}
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

  /* Initialize all configured peripherals */

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  DWT_Delay_Init();

  /* USER CODE BEGIN 2 */
  /*
   * emergency stop user acknowledgement
   */
  while(Emergency_Light == 0){Emergency_Light_toggle();}/*Set Emergency toggling*/Emergency_Light_Reset();/*Emergency reset*/
  /*
   * end here
   *
   */

  /* USER CODE END 2 */
  /*
   *
   * Send Application that system is reseted
   *
   */


  uint8_t uartchars[15] = {'P','r','i','n','t','_','A','n','d','_','A','p','p','l','y'};
  HAL_UART_Transmit(&huart3,uartchars,15,100);
  motor_move_mtrid_step_dir_frequency(0,500,___home,1000,1);//SERVO HOME
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  	uartchar[0] = 0;
	    	while(uartchar[0] == 0)
	    	{
	     	   HAL_UART_Receive (&huart3,uartchar ,1, 100);
	    	}
	    	//
	    	/*
	    	 *
	    	 * Horizontal Printing Process
	    	 *
	    	 *
	    	 */
	    	if(uartchar[0] == 'R')
	    	{
				uartchar[0] = 0;
				while(uartchar[0] == 0)
				{
				   HAL_UART_Receive (&huart3,uartchar ,1, 100);
				}
				//Horizontal mode start here
				if(uartchar[0] == 'A')
				{
					uartchar[0] = 0;
					while(uartchar[0] != 'S')
					{//Horizontal mode
						/*
						 * if parameters are zero then do not run
						 * send K to raspi ask parameters
						 */
						if(PRINT_on == 0 || PIPE_on == 0 || Suction_On_time == 0 || PIPE_off == 0)
						{
							_print_signal[0] = 'K';
							HAL_UART_Transmit(&huart3,_print_signal,1,100);
							break;
						}
						/*
						 * end here
						 */

						/*
						 *make Servo Home
						 */
						motor_move_mtrid_step_dir_frequency(0,500,___home,1000,1);//SERVO HOME


						/*
						 * make a pad at print position
						 *
						 */
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);//roTATION 90*

						/*
						 * send the print signal to raspberry pi
						 * wait 100ms - sending time
						 */
						_print_signal[0] = 'D';
						HAL_UART_Transmit(&huart3,_print_signal,1,100);

						/*Wait print time
						 *
						 * make Suction here
						 * make pipe blow here ON AND OFF
						 * make roll back motor here and calibrate rollback motor
						 *
						 */

						___wait();
						Roll_back_ARm(0,2,0);
						Roll_back_ARm(6000,1,45);
						/*
						 * make sure is rollback motor is at home
						 * recheck rollback motor
						 */
						//todO

						/*
						 * make a pad at apply position
						 */

//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);//roTATION 90*

						uint8_t XX5 = HAL_GPIO_ReadPin(GPIOA, Carton_Detect_Sen_Pin); //CARTON SENSOR DETETCTION LOOP
						while(XX5 == 0)
						{
								XX5 = HAL_GPIO_ReadPin(GPIOA, Carton_Detect_Sen_Pin);
						}
						/*
						 * cartoon delay apply time is here
						 */
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);
						__delay_ms(CARTON_PRESS_DELAY);

						/*
						 *make servo down full stroke
						 */
						App_Stroke_mm_Operation = App_Stroke_mm*66;
						motor_move_mtrid_step_dir_frequency(servo_mtr,App_Stroke_mm_Operation,1,50,1);//SERVO dw
						uint8_t XX2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
						if(XX2 == 0)
						{
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);
						}
						else
						{
							_print_signal[0] = 'N';
							HAL_UART_Transmit(&huart3,_print_signal,1,100);
						}
						/*
						 * Suction Off here
						 */

						HAL_GPIO_WritePin(GPIOB,Suction_Relay_Pin,RESET);//suction Off

						/*
						 * make pipe blow On here
						 */

						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET); //pad blow on

						/*
						 * pad blow time is here
						 * time pad blow time is setable example - 100ms assumed
						 */

						__delay_ms(300);

						/*
						 * make a pad Blow Off Here
						 */
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET); //pad blow off

						/*
						 * make the servo gantry up position
						 */
						App_Stroke_mm_Operation = App_Stroke_mm*66;
						App_Stroke_mm_Operation -= 1000;
						motor_move_mtrid_step_dir_frequency(servo_mtr,App_Stroke_mm_Operation,0,50,1);//SERVO UP

						/*
						 * make servo gantry is at home position
						 *
						 */
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
						//__delay_ms(100);

						motor_move_mtrid_step_dir_frequency(0,500,___home,1000,1);//SERVO HOME
						uint8_t XX3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
						if(XX3 == 0)
						{
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
						}
						else
						{
							_print_signal[0] = 'N';
							HAL_UART_Transmit(&huart3,_print_signal,1,100);
						}
						/*
						 * READ CHAR TO CHECK WHETHER ABORT OR NOT ABORT
						 *
						 */
						HAL_UART_Receive (&huart3,uartchar ,1, 100);//READ CHAR TO CHECK WHETHER ABORT OR NOT ABORT
					}
				}
				/*
				 * Vertical mode here
				 */
				else if(uartchar[0] == 'B')
				{
					uartchar[0] = 0;
					while(uartchar[0] != 'S')
					{
						//Vertical mode

						/*
						 * if parameters are zero then do not run
						 * send K to raspi ask parameters
						 */
						if(PRINT_on == 0 || PIPE_on == 0 || Suction_On_time == 0 || PIPE_off == 0)
						{
							_print_signal[0] = 'K';
							HAL_UART_Transmit(&huart3,_print_signal,1,100);
							break;
						}
						/*
						 * end here
						 */
						/*
						 *make Servo Home
						 */
						motor_move_mtrid_step_dir_frequency(0,500,___home,1000,0);//SERVO HOME
						/*
						 * send the print signal to raspberry pi
						 * wait 100ms - sending time
						 */
						_print_signal[0] = 'D';
						HAL_UART_Transmit(&huart3,_print_signal,1,100);

					 	/*Wait print time
						 *
						 * make Suction here
						 * make pipe blow here ON AND OFF
						 * make roll back motor here and calibrate rollback motor
						 *
						 */
						___wait();
						/*
						 * make sure is rollback motor is at home
						 * recheck rollback motor
						 */
						Roll_back_ARm(0,2,0);
						Roll_back_ARm(6000,1,45);

						/*
						 * cartoon delay apply time is here
						 */

						uint8_t XX5 = HAL_GPIO_ReadPin(GPIOA, Carton_Detect_Sen_Pin); //CARTON SENSOR DETETCTION LOOP
						while(XX5 == 0)
						{
							XX5 = HAL_GPIO_ReadPin(GPIOA, Carton_Detect_Sen_Pin);
						}
						__delay_ms(CARTON_PRESS_DELAY);


						/*
						 *make servo down full stroke
						 */
						App_Stroke_mm_Operation = App_Stroke_mm*66;
//						App_Stroke_mm_Operation -= 1000;
						motor_move_mtrid_step_dir_frequency(servo_mtr,App_Stroke_mm_Operation,1,50,0);//SERVO dw

						/*
						 * Suction Off here
						 */

						HAL_GPIO_WritePin(GPIOB,Suction_Relay_Pin,RESET);//suction Off

						/*
						 * make pipe blow On here
						 */

						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET); //pad blow on

						/*
						 * pad blow time is here
						 * time pad blow time is setable example - 100ms assumed
						 */

						__delay_ms(300);

						/*
						 * make a pad Blow Off Here
						 */
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET); //pad blow off

						/*
						 * make the servo gantry up position
						 */
						App_Stroke_mm_Operation = App_Stroke_mm*66;
						App_Stroke_mm_Operation -= 1000;
						motor_move_mtrid_step_dir_frequency(servo_mtr,App_Stroke_mm_Operation,0,50,0);//SERVO UP

						/*
						 * make servo gantry is at home position
						 *
						 */
						motor_move_mtrid_step_dir_frequency(0,500,___home,1000,0);//SERVO HOME
						/*
						 * READ CHAR TO CHECK WHETHER ABORT OR NOT ABORT

						 */

						HAL_UART_Receive (&huart3,uartchar ,1, 100);//READ CHAR TO CHECK WHETHER ABORT OR NOT ABORT
					}
				}
	    	}
	    	//MAINTAINANCE MODE IS HERE
	 if(uartchar[0] == 'M')
	 {
	    	uartchar[0] = 0;
	    	while(uartchar[0] == 0)
	    	{
	     	   HAL_UART_Receive (&huart3,uartchar ,1, 100);
	    	}
			if(uartchar[0] == 'Q') //Servo home
			{
				motor_move_mtrid_step_dir_frequency(0,500,___home,1000,0);
			}
			//TODO

			//TODO
			else if(uartchar[0] == 'A')
			{
				 uint32_t X1 =	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
				 if(X1 != 0)
				 {
				 motor_move_mtrid_step_dir_frequency(servo_mtr,2000,0,100,0);//
				 }
			}

			else if(uartchar[0] == 'B')
			{
				motor_move_mtrid_step_dir_frequency(servo_mtr,2000,1,100,0);//dw conferm
			}
			else if(uartchar[0] == 'C')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100);
				HAL_GPIO_WritePin(GPIOB,Suction_Relay_Pin,SET);//suction ON
			}
			else if(uartchar[0] == 'D')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100);
				HAL_GPIO_WritePin(GPIOB,Suction_Relay_Pin,RESET);//suction OFF
			}

			else if(uartchar[0] == 'E')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,SET); //pad blow on
			}
			else if(uartchar[0] == 'F')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,RESET);//pad blow off
			}
			else if(uartchar[0] == 'G')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,SET); //pipe blow
			}
			else if(uartchar[0] == 'H')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,RESET); //pipe blow
			}
			else if(uartchar[0] == 'I')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100);
			    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);//roTATION 90* APPLY POS
			}
			else if(uartchar[0] == 'J')
			{
//				HAL_UART_Transmit(&huart3,uartchar,1,100);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);//roTATION 90* //PRINT POS
			}

			else if(uartchar[0] == 'K')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100); //rollback ccw

				Roll_back_ARm(0,2,0);
			}
			else if(uartchar[0] == 'P')
			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100); // rollback clk
				Roll_back_ARm(1000,1,150);
			}
//			else if(uartchar[0] == 'K')
//			{
//				HAL_UART_Transmit(&huart3, uartchar , 1, 100); // rollback Home
//				Roll_back_ARm(0,2,0);
//			}
			else if(uartchar[0] == 'R')
			{
				motor_move_mtrid_step_dir_frequency(servo_mtr,5000,1,1000,0);//dw
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);
				__delay_ms(100);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
				motor_move_mtrid_step_dir_frequency(0,500,___home,1000,0); //HOME
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
			}
			else if(uartchar[0] == 'S')
			{
				motor_move_mtrid_step_dir_frequency(servo_mtr,5000,1,1000,0);//dw
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);
				__delay_ms(1);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
				motor_move_mtrid_step_dir_frequency(0,500,___home,1000,0); //HOME
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
			}
			else if(uartchar[0] == 'T')
			{
				sensor_check();
				NVIC_SystemReset();
			}
			else if(uartchar[0] == 'U')
			{
				_print_signal[0] = 'Q';
				HAL_UART_Transmit(&huart3,_print_signal,1,100);
				__delay_ms(Suction_On_time);
				_print_signal[0] = 'W';
				HAL_UART_Transmit(&huart3,_print_signal,1,100);
				uartchar[0] = 0;
			}
			else if(uartchar[0] == 'V')
			{
				Roll_back_ARm(6400,1,150);
				uartchar[0] = 0;
			}
			else if(uartchar[0] == 'W')
			{
				Roll_back_ARm(6400,0,150);
				uartchar[0] = 0;
			}
//			else if(uartchar[0] == 'X')
//			{
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,SET);
//				__delay_ms(100);
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,SET);
//				__delay_ms(100);
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,SET);
//				uartchar[0] = 0;
//			}
//			else if(uartchar[0] == 'Y')
//			{
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,RESET);
//				__delay_ms(100);
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,RESET);
//				__delay_ms(100);
//				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,RESET);
//				uartchar[0] = 0;
//			}
//


	    }
	 /*
	  * Read parameters here Check
	  */

     if(uartchar[0] == 'U')
     {
       	uartchar[0] = 0;
         while(uartchar[0] == 0)
     	{
     	   HAL_UART_Receive (&huart3, uartchar, 1, 100);
     	}
         if(uartchar[0] == 'H' || uartchar[0] == 'V')
         {
        	 uartchar[0] = 0;
            while(uartchar[0] == 0)
            {
               HAL_UART_Receive (&huart3, uartchar, 1, 100);
            }
            	   	   if(uartchar[0] == 'A') // _Suction on time
            	   	   {
            	   		   uint8_t _Suction_ON[5] ={};
            	  		   count_buff = 0;
            	   		   HAL_UART_Receive(&huart3, _Suction_ON,5, 100);
                      	   for(unsigned int ___i=0;___i<5;___i++){
                      		 _Suction_ON[___i] -= '0';
                      	   }
                      	 Suction_On_time =((_Suction_ON[0]*10000)+(_Suction_ON[1]*1000)+(_Suction_ON[2]*100)+(_Suction_ON[3]*10)+(_Suction_ON[4]));
            	   	   }
            	   	uartchar[0] = 0;
            while(uartchar[0] == 0)
            {
             HAL_UART_Receive (&huart3, uartchar, 1, 100);
            }
            	   	   if(uartchar[0] == 'B')// Pipe_on_Time
            	   	   {
            	   		   uint8_t _Pipe_ON[5] ={};
            	   		   HAL_UART_Receive(&huart3, _Pipe_ON,5, 100);
                      	   for(unsigned int ___i=0;___i<5;___i++){
                      		 _Pipe_ON[___i] -= '0';
                      	   }
            	   		   PIPE_on = ((_Pipe_ON[0]*10000)+(_Pipe_ON[1]*1000)+(_Pipe_ON[2]*100)+(_Pipe_ON[3]*10)+(_Pipe_ON[4]));
            	   	   }
            	   	uartchar[0] = 0;
            while(uartchar[0] == 0)
            {
             HAL_UART_Receive (&huart3, uartchar, 1, 100);
            }
             if(uartchar[0] == 'C')// Pipe_of_Time
             {
          	   uint8_t _Pipe_OFF[5] ={};
          	   HAL_UART_Receive(&huart3, _Pipe_OFF,5, 100);
          	   for(unsigned int ___i=0;___i<5;___i++){
          		 _Pipe_OFF[___i] -= '0';
          	   }

          	   PIPE_off = ((_Pipe_OFF[0]*10000)+(_Pipe_OFF[1]*1000)+(_Pipe_OFF[2]*100)+(_Pipe_OFF[3]*10)+(_Pipe_OFF[4]));
             }
             uartchar[0] = 0;
            while(uartchar[0] == 0)
            {
                HAL_UART_Receive (&huart3, uartchar, 1, 100);
            }
            if(uartchar[0] == 'D')// Pipe_of_Time
            {
         	   uint8_t Print_ON[5] ={};
         	   HAL_UART_Receive(&huart3, Print_ON,5, 100);
          	   for(unsigned int ___i=0;___i<5;___i++){
          		 Print_ON[___i] -= '0';
          	   }
             PRINT_on = ((Print_ON[0]*10000)+(Print_ON[1]*1000)+(Print_ON[2]*100)+(Print_ON[3]*10)+(Print_ON[4]));
            }

            uartchar[0] = 0;
            while(uartchar[0] == 0)
            {
                HAL_UART_Receive (&huart3, uartchar, 1, 100);
            }


              if(uartchar[0] == 'E')// Crton press delay
              {
               uint8_t Carton_press[5] ={};
               HAL_UART_Receive(&huart3, Carton_press,5, 100);
          	   for(unsigned int ___i=0;___i<5;___i++){
          		 Carton_press[___i] -= '0';
          	   }
               CARTON_PRESS_DELAY = ((Carton_press[0]*10000)+(Carton_press[1]*1000)+(Carton_press[2]*100)+(Carton_press[3]*10)+(Carton_press[4]));
              }

              uartchar[0] = 0;
              while(uartchar[0] == 0)
              {
                  HAL_UART_Receive (&huart3, uartchar, 1, 100);
              }
              	/*
              	 *Distance in mm
              	 */
                if(uartchar[0] == 'F')// stroke in mm
                {
                 uint8_t App_Stroke_mm__[5] ={};
                 HAL_UART_Receive(&huart3, App_Stroke_mm__,5, 100);
            	   for(unsigned int ___i=0;___i<5;___i++){
            		   App_Stroke_mm__[___i] -= '0';
            	   }
            	   App_Stroke_mm = ((App_Stroke_mm__[0]*10000)+(App_Stroke_mm__[1]*1000)+\
            			   (App_Stroke_mm__[2]*100)+(App_Stroke_mm__[3]*10)+(App_Stroke_mm__[4]));

            	   //(App_Stroke_mm > 300)?(App_Stroke_mm = = 300):(App_Stroke_mm = App_Stroke_mm - 1);
            	   if(App_Stroke_mm > 300)
            	   {
            		   App_Stroke_mm = 300;
            	   }
                }

				_print_signal[0] = 'C';
				HAL_UART_Transmit(&huart3,_print_signal,1,100);
         }
     }
  	  } //while endX
}// INT Main end

void sensor_check(void)
{

	uint32_t XX1,XX2,XX3,XX4,XX5,XX6,XX7,XX8,XX9,XX10;

	XX8 = HAL_GPIO_ReadPin(GPIOB, Servo_Home_Sen_Pin);     //Servo Home sensor      //position no 8
	XX9 = HAL_GPIO_ReadPin(GPIOB, Grip_sen_Pin);           //grip sensor            //position no 9
	XX3 = HAL_GPIO_ReadPin(GPIOA, RollBack_Sen_Pin);       //Rollback home sensor   // position no 3
	XX5 = HAL_GPIO_ReadPin(GPIOB, Carton_Detect_Sen_Pin);  //Carton_Detect_Sen_    // position no 5

	XX1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); //position 1
	XX2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); //position 2
	XX4= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4); //position 4
	XX6 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6); //position 6
	XX7 =  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7); //position 7
	XX10 =  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);// position 10

	if(XX1 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK
	}
	else
	{
		uartchar[0] = '1';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK
	}
	if(XX2 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK
	}
	else
	{
		uartchar[0] = '2';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK
	}
	if(XX3 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK//Rollback home sensor
	}
	else
	{
		uartchar[0] = '3';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK//Rollback home sensor
	}
	if(XX4 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);
	}
	else
	{
		uartchar[0] = '4';
		HAL_UART_Transmit(&huart3,uartchar,1,100);
	}
	if(XX5 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK   //Carton_Detect_Sen_
	}
	else
	{
		uartchar[0] = '5';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK //Carton_Detect_Sen_
	}
	if(XX6 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);
	}
	else
	{
		uartchar[0] = '6';
		HAL_UART_Transmit(&huart3,uartchar,1,100);
	}
	if(XX7 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);
	}
	else
	{
		uartchar[0] = '7';
		HAL_UART_Transmit(&huart3,uartchar,1,100);
	}
	if(XX8 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK//Servo Home sensor
	}
	else
	{
		uartchar[0] = '8';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK//Servo Home sensor
	}
	if(XX9 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK //grip sensor
	}
	else
	{
		uartchar[0] = '9';
		HAL_UART_Transmit(&huart3,uartchar,1,100);//send OK //grip sensor
	}

	if(XX10 == 0)
	{
		uartchar[0] = '0';
		HAL_UART_Transmit(&huart3,uartchar,1,100);
	}
	else
	{
		uint8_t uart[3] = {'_','1','0'};
		HAL_UART_Transmit(&huart3,uart,3,100);
	}

	uartchar[0] = 1;

//	if(__dir == 1 && __steps >=20000)
//	{
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,SET);
//	}
//	if(__dir == 0 && __steps >=20000)
//	{
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,RESET);
//	}

}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();



  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|PAD_Relay_Pin|Suction_Built_Relay_Pin 
                          |Emergency_Stop_Light_Pin|Relay8_Pin|Srvo2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|Ungrip_Relay_Pin|Suction_Relay_Pin|GPIO_PIN_15 
                          |Srvo1_Pin|M32_Pin|M31_Pin|M22_Pin 
                          |M21_Pin|M12_Pin|M11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SEN1_Pin */
  GPIO_InitStruct.Pin = SEN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SEN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UnGrip_Sen_Pin RollBack_Sen_Pin Conveyer_SEN_Pin Carton_Detect_Sen_Pin 
                           Carton_Detect2_Sen_Pin Sen10_Pin */
  GPIO_InitStruct.Pin = UnGrip_Sen_Pin|RollBack_Sen_Pin|Conveyer_SEN_Pin|Carton_Detect_Sen_Pin 
                          |Carton_Detect2_Sen_Pin|Sen10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PAD_Relay_Pin Suction_Built_Relay_Pin 
                           Emergency_Stop_Light_Pin Relay8_Pin Srvo2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|PAD_Relay_Pin|Suction_Built_Relay_Pin 
                          |Emergency_Stop_Light_Pin|Relay8_Pin|Srvo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Servo_Home_Sen_Pin Grip_sen_Pin */
  GPIO_InitStruct.Pin = Servo_Home_Sen_Pin|Grip_sen_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 Ungrip_Relay_Pin Suction_Relay_Pin PB15 
                           Srvo1_Pin M32_Pin M31_Pin M22_Pin 
                           M21_Pin M12_Pin M11_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|Ungrip_Relay_Pin|Suction_Relay_Pin|GPIO_PIN_15 
                          |Srvo1_Pin|M32_Pin|M31_Pin|M22_Pin 
                          |M21_Pin|M12_Pin|M11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*
   * Enabling External Interrupts here
   */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn,0,0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /*
   * End
   */
}


/* USER CODE BEGIN 4 */
/*
 * external interrupt event callback loop is here
 *
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	NVIC_SystemReset();
	/*
	 * emergency Stop event will occure here
	 */
	/*
	 * Desable the interrupt
	 */
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	if(GPIO_Pin == GPIO_PIN_6){
		/*
		 * send R character to monitor
		 */
//
		_print_signal[0] = 'R';
		HAL_UART_Transmit(&huart3,_print_signal,1,100);

		/*
		 * MAKE SYSTEM RESET HERE
		 */
		NVIC_SystemReset();
	}

	/*
	 * Enabling the inter
	 */
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


