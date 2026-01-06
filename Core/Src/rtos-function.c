/*
 * rtos-function.cpp
 *
 *  Created on: Mar 1, 2025
 *      Author: stanly
 */

/*stm32 include*/
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/*microROS include*/
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "stm32f4xx_it.h"

#include <std_msgs/msg/int32.h>
/*user include*/
#include "timers.h"
#include "uros_init.h"
#include "motor_monitor.hpp"

/**************** stm32 variable ****************/
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim2;
/**************** stm32 variable ****************/

TimerHandle_t xTimer;
int test = 0;
int16_t count = 0;

void vTimerCallback( TimerHandle_t xTimer )
{
  // servo_refresh_angle();
  // magnet_valve_refresh();
  // air_pump_refresh();
  
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 /* USER CODE BEGIN Callback 0 */

 /* USER CODE END Callback 0 */
 if (htim->Instance == TIM10) {
  motor_monitor();
  
  count = __HAL_TIM_GetCounter(&htim5);
 }
 /* USER CODE BEGIN Callback 1 */

 /* USER CODE END Callback 1 */
}


/**************** freertos callback ****************/
void StartDefaultTask(void *argument)
{
  /*init*/
  // mission_init(); // servo init
  // uros_init();
  motor_init();
  
  // rtos timer for refreshing servo angle
  xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, vTimerCallback);
  xTimerStart(xTimer, 0);
  HAL_TIM_Base_Start_IT(&htim11);


  for(;;)
  {
    // uros_agent_status_check();
    
  }
}
/**************** freertos callback ****************/
