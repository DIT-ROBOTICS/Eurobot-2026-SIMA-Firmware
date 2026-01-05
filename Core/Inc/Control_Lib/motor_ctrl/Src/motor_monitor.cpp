#include "motor_monitor.hpp"
#include "motor_ctrl.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

//trace :
//1    PA0   ∧右
//2    PC0    I
//3    PC1    I
//4    PC2    I
//5    PC3   ∨左
//左  PC4
//右  PC5
//switch： PA4(程式啟動)
//motor：
//           dir    pwm    enA/enB
//    FR :B12   PC6      PA8/PA9
//    FL :A12   PC7      PA1/PA5
//    BR :B14   PC8      PA6/PA7
//    BL :B15   PC9      PB6/PB7
//
MotorController Motor_FR(&htim1, &htim8, TIM_CHANNEL_1, GPIOB, GPIO_PIN_12, 1.1, 40, 0);
MotorController Motor_FL(&htim2, &htim8, TIM_CHANNEL_2, GPIOA, GPIO_PIN_12, 1.1, 40, 0);
MotorController Motor_BR(&htim3, &htim8, TIM_CHANNEL_3, GPIOB, GPIO_PIN_14, 1.1, 40, 0);
MotorController Motor_BL(&htim4, &htim8, TIM_CHANNEL_4, GPIOB, GPIO_PIN_15, 1.1, 40, 0);

float VgoalFR = 0.0;
float VgoalFL = 0.0;
float VgoalBR = 0.0;
float VgoalBL = 0.0;


void motor_init(){
	Motor_FR.init( 1, 1);
	Motor_FL.init( 1,-1);
	Motor_BR.init(-1, 1);
	Motor_BL.init( 1,-1);
}

void motor_monitor(void) {
    Motor_FR.setSpeed(VgoalFR);
    Motor_BR.setSpeed(VgoalBR);
    Motor_FL.setSpeed(VgoalFL);
    Motor_BL.setSpeed(VgoalBL);
//    test = Motor_BR.getSpeed();
//    Motor_BL.getSpeed();
//    Motor_FR.getSpeed();
//    Motor_FL.getSpeed();
}
