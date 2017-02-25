/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DRDY_Pin GPIO_PIN_2
#define DRDY_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define MEMS_INT3_Pin GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_4
#define PWM1_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_7
#define SPI1_MISO_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_0
#define PWM2_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_1
#define PWM3_GPIO_Port GPIOB
#define DIR_UP1_Pin GPIO_PIN_8
#define DIR_UP1_GPIO_Port GPIOE
#define DIR_UP2_Pin GPIO_PIN_9
#define DIR_UP2_GPIO_Port GPIOE
#define DIR_UP3_Pin GPIO_PIN_10
#define DIR_UP3_GPIO_Port GPIOE
#define DIR_DWN0_Pin GPIO_PIN_11
#define DIR_DWN0_GPIO_Port GPIOE
#define DIR_DWN3_Pin GPIO_PIN_12
#define DIR_DWN3_GPIO_Port GPIOE
#define DIR_DWN2_Pin GPIO_PIN_13
#define DIR_DWN2_GPIO_Port GPIOE
#define DIR_DWN1_Pin GPIO_PIN_14
#define DIR_DWN1_GPIO_Port GPIOE
#define DIR_UP0_Pin GPIO_PIN_15
#define DIR_UP0_GPIO_Port GPIOE
#define DIR_DWN4_Pin GPIO_PIN_8
#define DIR_DWN4_GPIO_Port GPIOD
#define DIR_UP4_Pin GPIO_PIN_9
#define DIR_UP4_GPIO_Port GPIOD
#define SW_DWN4_Pin GPIO_PIN_10
#define SW_DWN4_GPIO_Port GPIOD
#define SW_UP4_Pin GPIO_PIN_11
#define SW_UP4_GPIO_Port GPIOD
#define SW_DWN3_Pin GPIO_PIN_12
#define SW_DWN3_GPIO_Port GPIOD
#define SW_UP3_Pin GPIO_PIN_13
#define SW_UP3_GPIO_Port GPIOD
#define SW_DWN2_Pin GPIO_PIN_14
#define SW_DWN2_GPIO_Port GPIOD
#define SW_UP2_Pin GPIO_PIN_15
#define SW_UP2_GPIO_Port GPIOD
#define PWM0_Pin GPIO_PIN_6
#define PWM0_GPIO_Port GPIOC
#define SW_DWN1_Pin GPIO_PIN_9
#define SW_DWN1_GPIO_Port GPIOC
#define SW_UP1_Pin GPIO_PIN_8
#define SW_UP1_GPIO_Port GPIOA
#define SW_DWN0_Pin GPIO_PIN_9
#define SW_DWN0_GPIO_Port GPIOA
#define SW_UP0_Pin GPIO_PIN_10
#define SW_UP0_GPIO_Port GPIOA
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_15
#define PWM4_GPIO_Port GPIOA
#define SIG_TRIG0_Pin GPIO_PIN_1
#define SIG_TRIG0_GPIO_Port GPIOD
#define SIG_TRIG1_Pin GPIO_PIN_2
#define SIG_TRIG1_GPIO_Port GPIOD
#define SIG_TRIG2_Pin GPIO_PIN_3
#define SIG_TRIG2_GPIO_Port GPIOD
#define SIG_TRIG3_Pin GPIO_PIN_4
#define SIG_TRIG3_GPIO_Port GPIOD
#define SIG_TRIG4_Pin GPIO_PIN_5
#define SIG_TRIG4_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MEMS_INT1_Pin GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
