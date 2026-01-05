
#ifndef INC_CONTROL_LIB_CHASSIS_CTRL_INC_CHASSIS_MONITOR_HPP_
#define INC_CONTROL_LIB_CHASSIS_CTRL_INC_CHASSIS_MONITOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
void chassis_monitor(void);
void chassis_set_speed(float vx,float vy,float vz);
void chassis_give_speed();
extern float vel_x, vel_y, vel_z;

#ifdef __cplusplus
}

#endif




#endif /* INC_CONTROL_LIB_CHASSIS_CTRL_INC_CHASSIS_MONITOR_HPP_ */
