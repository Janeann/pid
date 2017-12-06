/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "pid.h"
#include "tim.h"
#define APP_LOG_MODULE_NAME "[freertos]"
#include "app_log.h"
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
  arm_pid_instance_t pid_eeprom [1] =
  {{6, 98304, 53084, 1835, 222822, 0, 0}};
 
  arm_pid_t cat;
 
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
 HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 pid_init (&cat,&pid_eeprom[0], 255, (uint32_t*)&htim1.Instance->CCR1, (uint32_t*)&htim1.Instance->CCR1, (uint32_t*)&htim2.Instance->CNT);
  
  
  for(;;)
  {
    osDelay(4);
    pid_update(&cat,1);
    int c=SEGGER_RTT_GetKey();
    if(c == 'a')
    {
    APP_LOG_INFO("VL：%d\r\n"  ,cat.pid.VL);
    APP_LOG_INFO("Kf：%d\r\n"  ,cat.pid.Kf);
    APP_LOG_INFO("Kp：%d\r\n"  ,cat.pid.Kp);
    APP_LOG_INFO("Ki：%d\r\n"  ,cat.pid.Ki);
    APP_LOG_INFO("Kd：%d\r\n"  ,cat.pid.Kd);
    APP_LOG_INFO("模式：%d\r\n"  ,cat.servo_mode);
    APP_LOG_INFO("目标点：%d\r\n"  ,cat.setpoint);
    APP_LOG_INFO("实际位置：%d\r\n"  ,cat.feedback);
    APP_LOG_INFO("速度：%d\r\n"  ,cat.velocity);
    APP_LOG_INFO("积分：%d\r\n"  ,cat.integral);
    APP_LOG_INFO("积分限制：%d\r\n"  ,cat.integral_limit);
       
    APP_LOG_INFO("上次误差：%d\r\n"  ,cat.last_error);
    APP_LOG_INFO("上次位置：%d\r\n"  ,cat.last_position);
    APP_LOG_INFO("最大PWM值：%d\r\n"  ,cat.pwm_max_val);
    APP_LOG_INFO("当前PWM值：%d\r\n"  ,(uint32_t) *cat.pwm_p);
    APP_LOG_INFO("当前编码器值：%d\r\n"  ,(uint32_t)*cat.encoder);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
