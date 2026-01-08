#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "math.h"
#include "motor_ctrl.hpp"
#include "chassis_config.h"
#include "motor_config.h"

/**********************************************************************/
/*******All the parameter about chassis is in "chassis_config.h"*******/
/**********************************************************************/
extern float Vx_global,Vy_global,dt;

class Chassis {
    public:
        Chassis(MotorController* motorR, MotorController* motorL):
                _motorR(motorR), _motorL(motorL){}
        ~Chassis() = default;

        void setSpeed(float V_Linear_goal, float W_goal);       // Set the speed of chassis
        // void getLocation();                                  // Get the location of the chassis
        void Differential_ForwardKinematics();                  // Compute the forward kinematics of differential chassis
        void Differential_InverseKinematics();                  // Compute the inverse kinematics of differential chassis
        // void Mecan_ForwardKinematics();                                     // Compute the forward kinematics of mecanum chassis
        // void Mecan_InverseKinematics();                                     // Compute the inverse kinematics of mecanum chassis
        // float x = INIT_X, y = INIT_Y, theta = INIT_THETA;
        MotorController* _motorR;
        MotorController* _motorL;
        float _V_Linear_now = 0, _W_now = 0;

    private:        
        float _V_R_goal = 0, _V_L_goal = 0;
        float _V_Linear_goal = 0, _W_goal = 0;
        // float _wheel_perimeter = (PI * WHEEL_DIA);
        float _V_R_now = 0, _V_L_now = 0;
};


#ifdef __cplusplus
}
#endif

#endif // CHASSIS_HPP
