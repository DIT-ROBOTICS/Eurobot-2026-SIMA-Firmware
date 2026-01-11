#include "motor_monitor.hpp"
#include "motor_ctrl.hpp"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

// EN_A: TIM5 PWM_A: TIM2_CH1 DIR_A: PB_12
// EN_B: TIM3 PWM_B: TIM2_CH3 DIR_B: PB_4

// MotorController Motor_R(&htim5, &htim2, TIM_CHANNEL_1, GPIOB, GPIO_PIN_12, 3, 80, 0);
// MotorController Motor_L(&htim3, &htim2, TIM_CHANNEL_3, GPIOB, GPIO_PIN_4 , 3, 80, 0);

// float VgoalR = 0.5;
// float VgoalL = 0.5;


void motor_init(){
	// Motor_R.init( -1,-1);
	// Motor_L.init( 1, 1);
}

void motor_monitor(void) {
    // Motor_R.setSpeed(VgoalR);
    // Motor_L.setSpeed(VgoalL);
}
