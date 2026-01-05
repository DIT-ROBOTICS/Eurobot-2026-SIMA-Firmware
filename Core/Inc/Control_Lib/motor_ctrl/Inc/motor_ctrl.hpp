#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f446xx.h"

#include "math.h"
#include "motor_config.h"

class MotorController{
public:
    MotorController(TIM_HandleTypeDef* enc, TIM_HandleTypeDef* pwm, uint32_t channel, GPIO_TypeDef* dirGPIO, uint16_t dirPin, double kp, double ki, double kd):
        _enc(enc), _pwm(pwm), _channel(channel), _dirGPIO(dirGPIO), _dirPin(dirPin), _kp(kp), _ki(ki), _kd(kd){}
    ~MotorController() = default;
    
    void init(int en_ctrl,int dir_ctrl);                        // Initialize the basic parameter
    void setSpeed(float speed);        // Set motor speed in RPS
    float getSpeed();                  // Return the current speed (Need to enable setSpeed())


private:
    TIM_HandleTypeDef* _enc;
    TIM_HandleTypeDef* _pwm;
    uint32_t _channel;
    GPIO_TypeDef* _dirGPIO;
    uint16_t _dirPin;

    float _kp;
    float _ki;
    float _kd;

    bool _isCountingDown = false;
    bool _isRotating = false;
    uint16_t _last_cnt = 0;
    int16_t cnt = 0;
    uint16_t _pwmValue = 0;
    float _pidOutput = 0.0;
    float _currentSpeed = 0;
    float _targetSpeed = 0;
    float _error = 0;
    float _lastError = 0;
    float _integral = 0;
    float _dt = 100;
    uint16_t _current_cnt = 0;
    int _en_ctrl = 0;
    int _dir_ctrl = 0;
    //int _doupdatespeed = 0;

    float ComputePID();                // PID controller
    float updateSpeed();               // Update the motor current speed
};

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CTRL_H */
