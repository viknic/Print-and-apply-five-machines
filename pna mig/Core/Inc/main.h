/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEN1_Pin GPIO_PIN_13
#define SEN1_GPIO_Port GPIOC
#define UnGrip_Sen_Pin GPIO_PIN_0
#define UnGrip_Sen_GPIO_Port GPIOA
#define RollBack_Sen_Pin GPIO_PIN_1
#define RollBack_Sen_GPIO_Port GPIOA
#define Conveyer_SEN_Pin GPIO_PIN_4
#define Conveyer_SEN_GPIO_Port GPIOA
#define Carton_Detect_Sen_Pin GPIO_PIN_5
#define Carton_Detect_Sen_GPIO_Port GPIOA
#define Carton_Detect2_Sen_Pin GPIO_PIN_7
#define Carton_Detect2_Sen_GPIO_Port GPIOA
#define Servo_Home_Sen_Pin GPIO_PIN_0
#define Servo_Home_Sen_GPIO_Port GPIOB
#define Grip_sen_Pin GPIO_PIN_1
#define Grip_sen_GPIO_Port GPIOB
#define Ungrip_Relay_Pin GPIO_PIN_13
#define Ungrip_Relay_GPIO_Port GPIOB
#define Suction_Relay_Pin GPIO_PIN_14
#define Suction_Relay_GPIO_Port GPIOB
#define PAD_Relay_Pin GPIO_PIN_8
#define PAD_Relay_GPIO_Port GPIOA
#define Suction_Built_Relay_Pin GPIO_PIN_9
#define Suction_Built_Relay_GPIO_Port GPIOA
#define Emergency_Stop_Light_Pin GPIO_PIN_10
#define Emergency_Stop_Light_GPIO_Port GPIOA
#define Relay8_Pin GPIO_PIN_11
#define Relay8_GPIO_Port GPIOA
#define Sen10_Pin GPIO_PIN_12
#define Sen10_GPIO_Port GPIOA
#define Srvo2_Pin GPIO_PIN_15
#define Srvo2_GPIO_Port GPIOA
#define Srvo1_Pin GPIO_PIN_3
#define Srvo1_GPIO_Port GPIOB
#define M32_Pin GPIO_PIN_4
#define M32_GPIO_Port GPIOB
#define M31_Pin GPIO_PIN_5
#define M31_GPIO_Port GPIOB
#define M22_Pin GPIO_PIN_6
#define M22_GPIO_Port GPIOB
#define M21_Pin GPIO_PIN_7
#define M21_GPIO_Port GPIOB
#define M12_Pin GPIO_PIN_8
#define M12_GPIO_Port GPIOB
#define M11_Pin GPIO_PIN_9
#define M11_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
