#include "motor_ctrl.hpp"


void MotorController::init(int en_ctrl,int dir_ctrl) {
    HAL_TIM_Encoder_Start(_enc, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(_pwm, _channel);
    _dir_ctrl = dir_ctrl;
    _en_ctrl = en_ctrl;
}

void MotorController::setSpeed(float speed) {

    _targetSpeed = speed;
    ComputePID();

    if (_dir_ctrl == 1){
    	HAL_GPIO_WritePin(_dirGPIO, _dirPin, _pidOutput >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }else{
    	HAL_GPIO_WritePin(_dirGPIO, _dirPin, _pidOutput >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    _pwmValue = (uint16_t)(fabs(_pidOutput) * PWM_ARR );
    if (_pwmValue < 10) _pwmValue = 0;
    __HAL_TIM_SET_COMPARE(_pwm, _channel, _pwmValue);
}

float MotorController::getSpeed() {
    return _currentSpeed;
}

float MotorController::ComputePID() {
    updateSpeed();
    _error = _targetSpeed - _currentSpeed;

    _integral += _error * (DT / 1000.0);
    if(_integral >= INTEGRAL_LIMIT) _integral = INTEGRAL_LIMIT;
    else if(_integral <= -INTEGRAL_LIMIT) _integral = -INTEGRAL_LIMIT;


    float derivative = (_error - _lastError) / (DT / 1000.0);

    _pidOutput = (_kp * _error) + (_ki * _integral) + (_kd * derivative);

    // Update last error
    _lastError = _error;
    if(_pidOutput > 1) _pidOutput = 1;
    else if (_pidOutput < -1) _pidOutput = -1;
    return _pidOutput;
}

float MotorController::updateSpeed() {
	_cnt = __HAL_TIM_GetCounter(_enc);
	_currentSpeed = 10 * (_cnt / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0);
    __HAL_TIM_SET_COUNTER(_enc, 0);
    _currentSpeed *= _en_ctrl;
    return _currentSpeed;
}
