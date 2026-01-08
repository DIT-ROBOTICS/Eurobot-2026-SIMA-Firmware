
#ifndef INC_CONTROL_LIB_CHASSIS_CTRL_INC_CHASSIS_MONITOR_HPP_
#define INC_CONTROL_LIB_CHASSIS_CTRL_INC_CHASSIS_MONITOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
void chassis_init();
void chassis_set_speed(float V_Linear_goal, float W_goal);
void chassis_get_speed(float* V_Linear_now, float* W_now);

#ifdef __cplusplus
}

#endif




#endif /* INC_CONTROL_LIB_CHASSIS_CTRL_INC_CHASSIS_MONITOR_HPP_ */
