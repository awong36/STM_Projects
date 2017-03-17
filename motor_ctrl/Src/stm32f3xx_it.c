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

/* USER CODE BEGIN 0 */
#include "stdbool.h"

#define total_retry 3           //Total amount of retries before motor stops after timeout
#define timeout 30000           //Time limit for motor to reach position, configured as seconds/1ms
#define rest_time 15000          //Rest time after movement completed, configured as seconds/1ms
#define debounce 100            //Debounce time for micro-switches, configured as seconds/1ms
#define signal_time 2000        //On time for external trigger signal, configured as seconds/1ms
#define restart_time 3600000     //reset retry count after this set time expires, configured as seconds/1ms
#define up 0                    //boolean for up direction 
#define down 1                  //boolean for down direction

extern bool motor_dir[5];
extern bool motion[5];
extern bool SW_UP[5];
extern bool SW_DWN[5];
extern bool DIR_UP[5];                                                                                                                             
extern bool DIR_DWN[5];
extern uint8_t retry[5];
extern uint8_t cnt_trigger[5];
extern uint32_t cnt_timer[5];
extern uint32_t motor_timer[5];
extern uint32_t motor_rest[5];
extern uint32_t count[5];
extern uint32_t count_dir[5];
extern uint32_t retry_timer[5];
uint8_t i;
     

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

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



/* USER CODE BEGIN GPIO_update */

void UpdateGPIO(void)
{
  if (HAL_GPIO_ReadPin(SW_UP0_GPIO_Port, SW_UP0_Pin)){
      count[0] ++;
      if (count[0] >= debounce){ // delay for debounce
        count[0] = 0;
        SW_UP[0] = 1;
      }    
  }
  else if (!HAL_GPIO_ReadPin(SW_UP0_GPIO_Port, SW_UP0_Pin)){
        SW_UP[0] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_UP1_GPIO_Port, SW_UP1_Pin)){
      count[1] ++;
      if (count[1] >= debounce){ // delay for debounce
        count[1] = 0;
        SW_UP[1] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(SW_UP1_GPIO_Port, SW_UP1_Pin)){
        SW_UP[1] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_UP2_GPIO_Port, SW_UP2_Pin)){
      count[2] ++;
      if (count[2] >= debounce){ // delay for debounce
        count[2] = 0;
        SW_UP[2] = 1;
      }
  }
  else if (!HAL_GPIO_ReadPin(SW_UP2_GPIO_Port, SW_UP2_Pin)){
        SW_UP[2] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_UP3_GPIO_Port, SW_UP3_Pin)){
      count[3] ++;
      if (count[3] >= debounce){ // delay for debounce
        count[3] = 0;
        SW_UP[3] = 1;
      }  
  }
  else if (!HAL_GPIO_ReadPin(SW_UP3_GPIO_Port, SW_UP3_Pin)){
        SW_UP[3] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_UP4_GPIO_Port, SW_UP4_Pin)){
      count[4] ++;
      if (count[4] >= debounce){ // delay for debounce
        count[4] = 0;
        SW_UP[4] = 1;
      }
  }
  else if (!HAL_GPIO_ReadPin(SW_UP4_GPIO_Port, SW_UP4_Pin)){
        SW_UP[4] = 0;
  }
  
  //Scan DOWN switch GPIO status      
  if (HAL_GPIO_ReadPin(SW_DWN0_GPIO_Port, SW_DWN0_Pin)){
      count[0] ++;
      if (count[0] >= debounce){ // delay for debounce
        count[0] = 0;
        SW_DWN[0] = 1;
      }
  }      
  else if (!HAL_GPIO_ReadPin(SW_DWN0_GPIO_Port, SW_DWN0_Pin)){
        SW_DWN[0] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_DWN1_GPIO_Port, SW_DWN1_Pin)){
      count[1] ++;
      if (count[1] >= debounce){ // delay for debounce
        count[1] = 0;
        SW_DWN[1] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(SW_DWN1_GPIO_Port, SW_DWN1_Pin)){
        SW_DWN[1] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_DWN2_GPIO_Port, SW_DWN2_Pin)){
      count[2] ++;
      if (count[2] >= debounce){ // delay for debounce
        count[2] = 0;
        SW_DWN[2] = 1;
      }       
  }
  else if (!HAL_GPIO_ReadPin(SW_DWN2_GPIO_Port, SW_DWN2_Pin)){
        SW_DWN[2] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_DWN3_GPIO_Port, SW_DWN3_Pin)){
      count[3] ++;
      if (count[3] >= debounce){ // delay for debounce
        count[3] = 0;
        SW_DWN[3] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(SW_DWN3_GPIO_Port, SW_DWN3_Pin)){
        SW_DWN[3] = 0;
  }
  
  if (HAL_GPIO_ReadPin(SW_DWN4_GPIO_Port, SW_DWN4_Pin)){
      count[4] ++;
      if (count[4] >= debounce){ // delay for debounce
        count[4] = 0;
        SW_DWN[4] = 1;
      }
  }
  else if (!HAL_GPIO_ReadPin(SW_DWN4_GPIO_Port, SW_DWN4_Pin)){
        SW_DWN[4] = 0;
  }
  
}    

void UpdateDirection(void)
{
  //Scan UP direction GPIO status
  if (HAL_GPIO_ReadPin(DIR_UP0_GPIO_Port, DIR_UP0_Pin)){
      count_dir[0] ++;
      if (count_dir[0] >= debounce){
           count_dir[0] = 0;
           DIR_UP[0] = 1;
      }
  }    
  else if (!HAL_GPIO_ReadPin(DIR_UP0_GPIO_Port, DIR_UP0_Pin)){
      DIR_UP[0] = 0;
  }   
  
  
  if (HAL_GPIO_ReadPin(DIR_UP1_GPIO_Port, DIR_UP1_Pin)){
      count_dir[1] ++;
      if (count_dir[1] >= debounce){
           count_dir[1] = 0;
           DIR_UP[1] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_UP1_GPIO_Port, DIR_UP1_Pin)){
      DIR_UP[1] = 0;
  } 
  if (HAL_GPIO_ReadPin(DIR_UP2_GPIO_Port, DIR_UP2_Pin)){
      count_dir[2] ++;
      if (count_dir[2] >= debounce){
           count_dir[2] = 0;
           DIR_UP[2] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_UP2_GPIO_Port, DIR_UP2_Pin)){
      DIR_UP[2] = 0;
  } 
  
  if (HAL_GPIO_ReadPin(DIR_UP3_GPIO_Port, DIR_UP3_Pin)){
      count_dir[3] ++;
      if (count_dir[3] >= debounce){
           count_dir[3] = 0;
           DIR_UP[3] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_UP3_GPIO_Port, DIR_UP3_Pin)){
      DIR_UP[3] = 0;
  } 
  if (HAL_GPIO_ReadPin(DIR_UP4_GPIO_Port, DIR_UP4_Pin)){
      count_dir[4] ++;
      if (count_dir[4] >= debounce){
           count_dir[4] = 0;
           DIR_UP[4] = 1;
      }  
  }
  else if (!HAL_GPIO_ReadPin(DIR_UP4_GPIO_Port, DIR_UP4_Pin)){
      DIR_UP[4] = 0;
  } 
  //Scan DOWN direction GPIO status
  if (HAL_GPIO_ReadPin(DIR_DWN0_GPIO_Port, DIR_DWN0_Pin)){
      count_dir[0] ++;
      if (count_dir[0] >= debounce){
           count_dir[0] = 0;
           DIR_DWN[0] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_DWN0_GPIO_Port, DIR_DWN0_Pin)){
      DIR_DWN[0] = 0;   
  }
  if (HAL_GPIO_ReadPin(DIR_DWN1_GPIO_Port, DIR_DWN1_Pin)){
      count_dir[1] ++;
      if (count_dir[1] >= debounce){
           count_dir[1] = 0;
           DIR_DWN[1] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_DWN1_GPIO_Port, DIR_DWN1_Pin)){
      DIR_DWN[1] = 0;   
  }
  if (HAL_GPIO_ReadPin(DIR_DWN2_GPIO_Port, DIR_DWN2_Pin)){
      count_dir[2] ++;
      if (count_dir[2] >= debounce){
           count_dir[2] = 0;
           DIR_DWN[2] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_DWN2_GPIO_Port, DIR_DWN2_Pin)){
      DIR_DWN[2] = 0;   
  }
  if (HAL_GPIO_ReadPin(DIR_DWN3_GPIO_Port, DIR_DWN3_Pin)){
      count_dir[3] ++;
      if (count_dir[3] >= debounce){
           count_dir[3] = 0;
           DIR_DWN[3] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_DWN3_GPIO_Port, DIR_DWN3_Pin)){
      DIR_DWN[3] = 0;   
  }
  if (HAL_GPIO_ReadPin(DIR_DWN4_GPIO_Port, DIR_DWN4_Pin)){
      count_dir[4] ++;
      if (count_dir[4] >= debounce){
           count_dir[4] = 0;
           DIR_DWN[4] = 1;
      } 
  }
  else if (!HAL_GPIO_ReadPin(DIR_DWN4_GPIO_Port, DIR_DWN4_Pin)){
      DIR_DWN[4] = 0;   
  }
}    

/* USER CODE END GPIO_update */

/**
* @brief This function handles System tick timer.
*/

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  UpdateGPIO();
  UpdateDirection();  
 
  //Motor Switches condition 
  for(i = 0; i <= 4; i++){
     if (motion[i] == 1){ 
         if (DIR_UP[i] && !DIR_DWN[i]){ 
            if (SW_UP[i]){
                    motor_rest[i] = rest_time; //set rest time
                    motor_timer[i] = 0;
                    motion[i] = 0;
                    cnt_trigger[i] ++;     
            } 
                  
        } 
        if (DIR_DWN[i] && !DIR_UP[i]){
            if (SW_DWN[i]){
                    motor_rest[i] = rest_time; //set rest time 
                    motor_timer[i] = 0;
                    motion[i] = 0;
                    cnt_trigger[i] ++;  
            }
            
        }      
    }
  }
  
  //Motor timeout condition
  for(i = 0; i <= 4; i++){
    if (motion[i] == 1){  
        if (DIR_UP[i] == 1 || DIR_DWN[i] == 1){
           motor_timer[i] ++;     //increment run timer
           if (motor_timer[i] > timeout){         //timeout 
               motion[i] = 0;
               motor_rest[i] = rest_time;
               motor_timer[i] = 0;
               cnt_trigger[0] = 0;
               retry[i] ++;
               //switch direction when error occurs
               if (DIR_UP[i] && !DIR_DWN[i]){
                    motor_dir[i] = down;
               }
               if (DIR_DWN[i] && !DIR_UP[i]){
                    motor_dir[i] = up;
               }    
           }
   }
   }
  }    
  
  //Motor reset condition

  if (motion[0] == 0){
    HAL_GPIO_WritePin(DIR_UP0_GPIO_Port, DIR_UP0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_DWN0_GPIO_Port, DIR_DWN0_Pin, GPIO_PIN_RESET);
    motor_timer[0] = 0;    
  }
  if (motion[1] == 0){
    HAL_GPIO_WritePin(DIR_UP1_GPIO_Port, DIR_UP1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_DWN1_GPIO_Port, DIR_DWN1_Pin, GPIO_PIN_RESET);
    motor_timer[1] = 0;    
  }
  if (motion[2] == 0){
    HAL_GPIO_WritePin(DIR_UP2_GPIO_Port, DIR_UP2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_DWN2_GPIO_Port, DIR_DWN2_Pin, GPIO_PIN_RESET);
    motor_timer[2] = 0;      
  }
  if (motion[3] == 0){
    HAL_GPIO_WritePin(DIR_UP3_GPIO_Port, DIR_UP3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_DWN3_GPIO_Port, DIR_DWN3_Pin, GPIO_PIN_RESET);
    motor_timer[3] = 0;    
  }
  if (motion[4] == 0){
    HAL_GPIO_WritePin(DIR_UP4_GPIO_Port, DIR_UP4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_DWN4_GPIO_Port, DIR_DWN4_Pin, GPIO_PIN_RESET);
    motor_timer[4] = 0;  
  }
  
  //Motion ready condition
  for(i = 0; i <= 4; i++){
    if (motion[i] == 0){  
        if (retry[i] <= total_retry){
          if (motor_rest[i] > 0){
                motor_rest[i] --;
                if (SW_DWN[i]){
                    motor_dir[i] = up;
                }
                else if (SW_UP[i]){
                    motor_dir[i] = down;
                }        
          }
          if (motor_rest[i] <= 1){
                motion[i] = 1;
          }       
        }
        if (retry[i] > total_retry){
               retry_timer[i] ++;
               if (retry_timer[i] >= restart_time){
                   retry[i] = 0;
                   retry_timer[i] = 0;
               }    
        }
   }
   }    
   
   //Motor motion enable condition
   if (motion[0] == 1){
         
         if (motor_dir[0] == down && DIR_DWN[0] != SET){          
            if (cnt_trigger[0] == 2){
                HAL_GPIO_WritePin(SIG_TRIG0_GPIO_Port, SIG_TRIG0_Pin, GPIO_PIN_SET); //sig trig set for counter
                //cnt_timer[0] = signal_time;
                retry[0] = 0;   //reset retry count if cycle counter increments
                
            }
            HAL_GPIO_WritePin(DIR_DWN0_GPIO_Port, DIR_DWN0_Pin, GPIO_PIN_SET);
            motor_timer[0] = 0;
            cnt_trigger[0] = 0;
         }
         else if (motor_dir[0] == up && DIR_UP[0] != SET){ 
            HAL_GPIO_WritePin(SIG_TRIG0_GPIO_Port, SIG_TRIG0_Pin, GPIO_PIN_RESET);  //sig trig reset for counter
            HAL_GPIO_WritePin(DIR_UP0_GPIO_Port, DIR_UP0_Pin, GPIO_PIN_SET);
            motor_timer[0] = 0; 
         }       
     } 
   if (motion[1] == 1){
         if (motor_dir[1] == down && DIR_DWN[1] != SET){          
            if (cnt_trigger[1] == 2){
                HAL_GPIO_WritePin(SIG_TRIG1_GPIO_Port, SIG_TRIG1_Pin, GPIO_PIN_SET); //sig trig set for counter
                cnt_timer[1] = signal_time; 
                retry[1] = 0;   //reset retry count if cycle counter increments        
            }
            HAL_GPIO_WritePin(DIR_DWN1_GPIO_Port, DIR_DWN1_Pin, GPIO_PIN_SET);
            motor_timer[1] = 0;
            cnt_trigger[1] = 0;
         }
         else if (motor_dir[1] == up && DIR_UP[1] != SET){ 
            HAL_GPIO_WritePin(SIG_TRIG1_GPIO_Port, SIG_TRIG1_Pin, GPIO_PIN_RESET);  //sig trig reset for counter
            HAL_GPIO_WritePin(DIR_UP1_GPIO_Port, DIR_UP1_Pin, GPIO_PIN_SET);
            motor_timer[1] = 0; 
         }       
     } 
   if (motion[2] == 1){
         if (motor_dir[2] == down && DIR_DWN[2] != SET){          
            if (cnt_trigger[2] == 2){
                HAL_GPIO_WritePin(SIG_TRIG2_GPIO_Port, SIG_TRIG2_Pin, GPIO_PIN_SET); //sig trig set for counter
                cnt_timer[2] = signal_time;
                retry[2] = 0;   //reset retry count if cycle counter increments
                
            }
            HAL_GPIO_WritePin(DIR_DWN2_GPIO_Port, DIR_DWN2_Pin, GPIO_PIN_SET);
            motor_timer[2] = 0;
            cnt_trigger[2] = 0;
         }
         else if (motor_dir[2] == up && DIR_UP[2] != SET){ 
            HAL_GPIO_WritePin(SIG_TRIG2_GPIO_Port, SIG_TRIG2_Pin, GPIO_PIN_RESET);  //sig trig reset for counter
            HAL_GPIO_WritePin(DIR_UP2_GPIO_Port, DIR_UP2_Pin, GPIO_PIN_SET);
            motor_timer[2] = 0; 
         }       
     }
   if (motion[3] == 1){
         if (motor_dir[3] == down && DIR_DWN[3] != SET){          
            if (cnt_trigger[3] == 2){
                HAL_GPIO_WritePin(SIG_TRIG3_GPIO_Port, SIG_TRIG3_Pin, GPIO_PIN_SET); //sig trig set for counter
                cnt_timer[3] = signal_time;
                retry[3] = 0;   //reset retry count if cycle counter increments
                
            }
            HAL_GPIO_WritePin(DIR_DWN3_GPIO_Port, DIR_DWN3_Pin, GPIO_PIN_SET);
            motor_timer[3] = 0;
            cnt_trigger[3] = 0;
         }
         else if (motor_dir[3] == up && DIR_UP[3] != SET){ 
            HAL_GPIO_WritePin(SIG_TRIG3_GPIO_Port, SIG_TRIG3_Pin, GPIO_PIN_RESET);  //sig trig reset for counter
            HAL_GPIO_WritePin(DIR_UP3_GPIO_Port, DIR_UP3_Pin, GPIO_PIN_SET);
            motor_timer[3] = 0; 
         }       
     }
   if (motion[4] == 1){
         if (motor_dir[4] == down && DIR_DWN[4] != SET){          
            if (cnt_trigger[4] == 2){
                HAL_GPIO_WritePin(SIG_TRIG4_GPIO_Port, SIG_TRIG4_Pin, GPIO_PIN_SET); //sig trig set for counter
                cnt_timer[4] = signal_time;
                retry[4] = 0;   //reset retry count if cycle counter increments
                
            }
            HAL_GPIO_WritePin(DIR_DWN4_GPIO_Port, DIR_DWN4_Pin, GPIO_PIN_SET);
            motor_timer[4] = 0;
            cnt_trigger[4] = 0;
         }
         else if (motor_dir[4] == up && DIR_UP[4] != SET){ 
            HAL_GPIO_WritePin(SIG_TRIG4_GPIO_Port, SIG_TRIG4_Pin, GPIO_PIN_RESET);  //sig trig reset for counter
            HAL_GPIO_WritePin(DIR_UP4_GPIO_Port, DIR_UP4_Pin, GPIO_PIN_SET);
            motor_timer[4] = 0; 
         }       
     }
   

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
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

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
