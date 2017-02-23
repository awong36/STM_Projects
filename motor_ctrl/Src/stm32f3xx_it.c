/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "stdbool.h"

/* USER CODE BEGIN 0 */
#define total_retry 3           //Total amount of retries before motor stops after timeout
#define timeout 30000           //Time limit for motor to reach position, configured as seconds/1ms
#define rest_time 5000          //Rest time after movement completes, configured as seconds/1ms
#define debounce 100            //Debounce time for micro-switches, configured as seconds/1ms
#define signal_time 2000        //On time for external trigger signal, configured as seconds/1ms

extern bool motor_dir[5];
extern bool motion[5];
extern uint8_t retry[5];
extern uint8_t cnt_trigger[5];
extern uint32_t cnt_timer[5];
extern uint32_t motor_timer[5];
extern uint32_t motor_rest[5];
extern uint32_t count[5];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
* each iteration increment by 1ms config by systick config
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
    
  //Motor Switches condition
  if (HAL_GPIO_ReadPin(SW_UP1_GPIO_Port, SW_UP1_Pin)){
       count[0] ++;
       if (count[0] >= debounce){ // delay for debounce
           count[0] = 0;
           if (motor_dir[0] != 1){
                motor_dir[0] = 1;      //next direction
                motor_rest[0] = rest_time; //set rest time
                motion[0] = 0;
                cnt_trigger[0] ++;
           }     
       }    
  }
  
  if (HAL_GPIO_ReadPin(SW_DWN1_GPIO_Port, SW_DWN1_Pin)){
       count[0] ++;
       if (count[0] >= debounce){ // delay debounce
           count[0] = 0;
           if (motor_dir[0] != 0){
                motor_dir[0] = 0;      //next direction 
                motor_rest[0] = rest_time; //set rest time 
                motion[0] = 0;
                cnt_trigger[0] ++;
           }    
        }
  }  

  //Motor reset and timeout condition
  if (HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin) || HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin)){
       motor_timer[0] ++;     //increment run timer
       if (motor_timer[0] > timeout){         //timeout 
           motion[0] = 0;
           motor_rest[0] = rest_time;
           retry[0] ++;
       }    
       if (motion[0] == 0){
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
            motor_timer[0] = 0;          
       } 
  }
  //Motion ready condition
  if (retry[0] <= total_retry){
      if (motor_rest[0] > 0){
            motor_rest[0] --;
            motion[0] = 0;
      }
      if (motor_rest[0] == 0){
            motion[0] = 1;
      }       
     
     if (motion[0] == 1){
         if (motor_dir[0] == 1 && HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin) != SET){
            if (cnt_trigger[0] == 2){
                HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET); 
                cnt_timer[0] = signal_time;        
            }
            cnt_trigger[0] = 0;
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
            motor_timer[0] = 0;

         }
         if (motor_dir[0] == 0 && HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin) != SET){ 
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            motor_timer[0] = 0; 
         }       
     } 
     
           
     
            
   }  
   if (cnt_timer != 0){
        cnt_timer[0] --;
        HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin); 
     }
     else{
         //(cnt_timer[0] == 0){
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
     }  
  /* USER CODE END SysTick_IRQn 0 */
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
     
     
         
         
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
