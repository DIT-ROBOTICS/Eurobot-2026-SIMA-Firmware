#include "chassis.hpp"
#include "motor_config.h"
float Vx_global,Vy_global,dt;
void Chassis::setSpeed(float V_Linear_goal, float W_goal){
    _V_Linear_goal = V_Linear_goal;
    _W_goal  = W_goal;
    Differencial_InverseKinematics();
    _motorR->setSpeed(_V_R_goal / _wheel_perimeter);   // RPS
    _motorL->setSpeed(_V_L_goal / _wheel_perimeter);   // RPS
}

void Chassis::Differencial_ForwardKinematics(){
    _V_R_now = _motorR->getSpeed() * WHEEL_DIA * PI;   // m/s
    _V_L_now = _motorL->getSpeed() * WHEEL_DIA * PI;   // m/s

    _Vx_now = (_V_R_now + _V_L_now) / 2.0f;
    _W_now = (_V_R_now - _V_L_now) / CHASSIS_WIDTH;
}

void Chassis::Differencial_InverseKinematics(){
    _V_R_goal = _V_Linear_goal + _W_goal*(CHASSIS_WIDTH/2);
    _V_L_goal = _V_Linear_goal - _W_goal*(CHASSIS_WIDTH/2);
}

// void Chassis::getLocation(){
// 	Mecan_ForwardKinematics();
// 	dt = DT;
//     theta += _W_now * DT/1000;                                              // rad
//     Vx_global = _Vx_now * cos(theta) - _Vy_now * sin(theta);   // cm/s
//     Vy_global = _Vx_now * sin(theta) + _Vy_now * cos(theta);   // cm/s
//     x += (Vx_global * (dt/1000));                                        // cm
//     y += (Vy_global * (dt/1000));										// cm
// }

// void Chassis::Mecan_ForwardKinematics(){
//     _V_FR_now = _motorFR->getSpeed() * WHEEL_DIA * PI;                   // cm/s
//     _V_FL_now = _motorFL->getSpeed() * WHEEL_DIA * PI;                   // cm/s
//     _V_BR_now = _motorBR->getSpeed() * WHEEL_DIA * PI;                   // cm/s
//     _V_BL_now = _motorBL->getSpeed() * WHEEL_DIA * PI;                   // cm/s

//     _Vx_now = (-_V_FR_now + _V_FL_now + _V_BR_now - _V_BL_now) / 4.0f;
//     _Vy_now = (_V_FR_now + _V_FL_now + _V_BR_now + _V_BL_now) / 4.0f;
//     _W_now = (-_V_FR_now +   _V_FL_now - _V_BR_now + _V_BL_now) / (CHASSIS_WIDTH + CHASSIS_LENGTH);
// }

// void Chassis::Mecan_InverseKinematics(){
//     _V_FR_goal = -_Vx_goal + _Vy_goal + _W_goal * _chassis_factor;
//     _V_FL_goal = _Vx_goal + _Vy_goal - _W_goal * _chassis_factor;
//     _V_BR_goal = _Vx_goal + _Vy_goal + _W_goal * _chassis_factor;
//     _V_BL_goal = -_Vx_goal + _Vy_goal - _W_goal * _chassis_factor;
// }
