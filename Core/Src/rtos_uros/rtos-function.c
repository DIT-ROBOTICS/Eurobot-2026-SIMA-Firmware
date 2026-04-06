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
#include "chassis_monitor.hpp"
#include "vl53l0x_api.h"
#include "vl53l0x_ctrl.hpp"

/**************** stm32 variable ****************/
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
/**************** stm32 variable ****************/
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_Dev_t  vl53l0x_l; // left module
VL53L0X_Dev_t  vl53l0x_r; // right module


TimerHandle_t xRangingTimer;
float vl53l0x_ranges[3] = {0.0, 0.0, 0.0};
float V_Linear_goal = 0.0;
float W_angular_goal = 0.0;
float V_Linear_now = 0.0;
float W_angular_now = 0.0;

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_RangingMeasurementData_t RangingData2;
VL53L0X_RangingMeasurementData_t RangingData3;

// int16_t count = 0;

void RangingTimerCallback( TimerHandle_t xTimer )
{
  vl53l0x_ranges[0] = vl53l0x_read_distance(&vl53l0x_r, &RangingData);
  vl53l0x_ranges[1] = vl53l0x_read_distance(&vl53l0x_c, &RangingData2);
  vl53l0x_ranges[2] = vl53l0x_read_distance(&vl53l0x_l, &RangingData3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11) {
    HAL_IncTick();
    chassis_set_speed(V_Linear_goal, W_angular_goal);
    chassis_get_speed(&V_Linear_now, &W_angular_now);
  }
}


/**************** freertos callback ****************/
void StartDefaultTask(void *argument)
{
  /*init*/
  HAL_TIM_Base_Start_IT(&htim11);   // Start the timer interrupt for chassis control
  uros_init();
  chassis_init();
  vl53l0x_init_single(&hi2c1, &vl53l0x_r, GPIOB, GPIO_PIN_13);
  vl53l0x_init_single(&hi2c2, &vl53l0x_l, GPIOC, GPIO_PIN_4);
  vl53l0x_init_single(&hi2c3, &vl53l0x_c, GPIOC, GPIO_PIN_5);
  // rtos timer for refreshing servo angle
  xRangingTimer = xTimerCreate("RangingTimer", pdMS_TO_TICKS(50), pdTRUE, (void *)0, RangingTimerCallback);
  xTimerStart(xRangingTimer, 0);
  for(;;)
  {
    uros_agent_status_check();
//     osDelay(5);
  }
}
/**************** freertos callback ****************/
