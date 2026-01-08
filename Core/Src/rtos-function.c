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
#include "chassis.hpp"

/**************** stm32 variable ****************/
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
/**************** stm32 variable ****************/


TimerHandle_t xTimer;
int test = 0;
float V_Linear = 0.0;
float W_angular = 0.0;


// int16_t count = 0;

// void vTimerCallback( TimerHandle_t xTimer )
// {
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11) {
    HAL_IncTick();
    // chassis_set_speed(V_Linear, W_angular);
    // chassis_get_speed(&V_Linear, &W_angular);
  }
}


/**************** freertos callback ****************/
void StartDefaultTask(void *argument)
{
  /*init*/
  HAL_TIM_Base_Start_IT(&htim11);   // Start the timer interrupt for chassis control
  uros_init();
  
  // rtos timer for refreshing servo angle
  // xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, vTimerCallback);
  // xTimerStart(xTimer, 0);
  for(;;)
  {
    uros_agent_status_check();
  }
}
/**************** freertos callback ****************/
