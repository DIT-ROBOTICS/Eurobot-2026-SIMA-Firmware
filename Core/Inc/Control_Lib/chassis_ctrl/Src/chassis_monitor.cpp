#include "chassis.hpp"
#include "chassis_monitor.hpp"
#include "motor_ctrl.hpp"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

// EN_A: TIM5 PWM_A: TIM2_CH1 DIR_A: PB_12
// EN_B: TIM3 PWM_B: TIM2_CH3 DIR_B: PB_4

MotorController Motor_R(&htim5, &htim2, TIM_CHANNEL_1, GPIOB, GPIO_PIN_12, 3, 80, 0);
MotorController Motor_L(&htim3, &htim2, TIM_CHANNEL_3, GPIOB, GPIO_PIN_4 , 3, 80, 0);

Chassis chassis(&Motor_R, &Motor_L);

void chassis_init(){
    Motor_R.init( -1,-1);
    Motor_L.init( 1, 1);
}

void chassis_set_speed(float V_Linear_goal, float W_goal) {
    chassis.setSpeed(V_Linear_goal, W_goal);
}

void chassis_get_speed(float* V_Linear_now, float* W_now) {
    chassis.Differential_ForwardKinematics();
    *V_Linear_now = chassis._V_Linear_now;
    *W_now = chassis._W_now;
}