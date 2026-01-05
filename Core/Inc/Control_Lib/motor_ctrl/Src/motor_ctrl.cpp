#include "motor_ctrl.hpp"
int times = 0;
uint32_t current_cnt;
void MotorController::init(int en_ctrl,int dir_ctrl) {
    HAL_TIM_Encoder_Start(_enc, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(_pwm, _channel);
    _dir_ctrl = dir_ctrl;
    _en_ctrl = en_ctrl;
}

void MotorController::setSpeed(float speed) {

    _targetSpeed = speed;
//    times++;
    ComputePID();

//    if (_pidOutput > 80.0) _pidOutput = 80.0;
//    if (_pidOutput < -80.0) _pidOutput = -80.0;
    if (_dir_ctrl == 1){
    	HAL_GPIO_WritePin(_dirGPIO, _dirPin, _pidOutput >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }else{
    	HAL_GPIO_WritePin(_dirGPIO, _dirPin, _pidOutput >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    _pwmValue = (uint16_t)(fabs(_pidOutput) * PWM_ARR );///  10.0);
    if (_pwmValue < 10) _pwmValue = 0;
    __HAL_TIM_SET_COMPARE(_pwm, _channel, _pwmValue);
//    _targetSpeed = speed;
//    ComputePID();
//    _pwmValue = fabs(_pidOutput) * PWM_ARR;
//
//	if(_pidOutput > 0) HAL_GPIO_WritePin(_dirGPIO, _dirPin, GPIO_PIN_SET);
//	else HAL_GPIO_WritePin(_dirGPIO, _dirPin, GPIO_PIN_RESET);
//
//	__HAL_TIM_SET_COMPARE(_pwm, _channel, (uint16_t)_pwmValue);

}

float MotorController::getSpeed() {
//	updateSpeed();
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
//	float bound = 1.0f / _ki;
//	_error = _targetSpeed - _currentSpeed;
//
//	_integral += _error * (DT / 1000.0);
////    differential = error - pre_error;
//	if(_integral > bound) _integral = bound;
//	else if(_integral < -bound) _integral = -bound;
////    else if (fabs(_speed) < 0.0005) _integral =0;
////    if (ki * integral >= 1) ki * integral =0.01;
//
//	_pidOutput = _kp * _error + _ki * _integral ;//+ kd * differential
////    pre_error = error;
//
//	if (_pidOutput > 1) _pidOutput = 1;
//	else if (_pidOutput < -1) _pidOutput = -1;
//    return _pidOutput;
}

float MotorController::updateSpeed() {
//    if(_last_cnt == __HAL_TIM_GET_COUNTER(_enc))
//        _isCountingDown = false;
//    else
//        _isCountingDown = __HAL_TIM_IS_TIM_COUNTING_DOWN(_enc);
//
//    if(_isCountingDown)
//        _currentSpeed = -((_enc->Instance->ARR - __HAL_TIM_GET_COUNTER(_enc)) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS
//    else
//        _currentSpeed = (__HAL_TIM_GET_COUNTER(_enc) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS
//
//    _last_cnt = __HAL_TIM_GET_COUNTER(_enc);

	cnt = __HAL_TIM_GetCounter(_enc);
	_currentSpeed = (cnt/ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0);
    __HAL_TIM_SET_COUNTER(_enc, 0);
    _currentSpeed *= _en_ctrl;
    return _currentSpeed;
}
//double MotorController::updateSpeed() {
//    if(_last_cnt == __HAL_TIM_GET_COUNTER(_enc)){
//        _isRotating = false;
//        _currentSpeed = 0;
//    }else{
//        _isRotating = true;
//    }
//
//    _isCountingDown = __HAL_TIM_IS_TIM_COUNTING_DOWN(_enc);
//
//    if(_isRotating && _isCountingDown){
//        _currentSpeed = -((_enc->Instance->ARR - __HAL_TIM_GET_COUNTER(_enc)) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS
//    }else if(_isRotating && (!_isCountingDown)){
//        _currentSpeed = (__HAL_TIM_GET_COUNTER(_enc) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS
//    }
//    _currentSpeed *= _en_ctrl;
//    _last_cnt = __HAL_TIM_GET_COUNTER(_enc);
//    __HAL_TIM_SET_COUNTER(_enc, 0);
//    return _currentSpeed;
//}
//double MotorController::updateSpeed() {
//	_current_cnt = __HAL_TIM_GET_COUNTER(_enc);
//// Calculate the difference considering counter overflow/underflow
//    int32_t count_diff;
//    // Simple difference calculation
//    count_diff = (int32_t)_current_cnt - (int32_t)_last_cnt;
//    // Handle 16-bit counter wraparound
//    if (count_diff > 32767) {
//        // Counter wrapped from high to low (underflow)
//        count_diff -= 65536;
//    } else if (count_diff < -32767) {
//        // Counter wrapped from low to high (overflow)
//        count_diff += 65536;
//    }
//    // Calculate speed: count_diff / (encoder_resolution * reduction_ratio * 4) / time_in_seconds
//    // Divide by 4 for quadrature encoding (TI12 mode gives 4x resolution)
//    _currentSpeed = (double)count_diff / (ENCODER_RESOLUTION * REDUCTION_RATIO * 4.0) / (DT / 1000.0); // RPS
//}
//double MotorController::updateSpeed() {
//    ///_current_cnt = __HAL_TIM_GET_COUNTER(_enc);
//    if(_last_cnt == current_cnt){
//    	_isRotating = false;
//        _isCountingDown = false;
//    }else{
//    	_isRotating = true;
//    	_isCountingDown = __HAL_TIM_IS_TIM_COUNTING_DOWN(_enc);
//    }
//
//
//    if(_isCountingDown){
//        _currentSpeed = -((_enc->Instance->ARR - __HAL_TIM_GET_COUNTER(_enc)) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS
//    }else{
//        _currentSpeed = (__HAL_TIM_GET_COUNTER(_enc) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS
//    }
//    _last_cnt = _current_cnt;
//    __HAL_TIM_SET_COUNTER(_enc, 0);
//    _doupdatespeed++;
//    return _currentSpeed;
//}
//double MotorController::updateSpeed2(int sign) {
//    _cnt = __HAL_TIM_GET_COUNTER(_enc);
//    __HAL_TIM_SET_COUNTER(_enc, 0);
//    _currentSpeed = sign * (float)_enc / (4.0f * _resolution * span * _reduction_ratio);// 100.0 *   //每秒圈數
//    return _currentSpeed;
//}
