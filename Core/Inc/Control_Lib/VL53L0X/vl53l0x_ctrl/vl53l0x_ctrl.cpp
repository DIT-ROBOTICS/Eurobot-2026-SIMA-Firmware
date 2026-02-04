#include "vl53l0x_ctrl.hpp"


void vl53l0x_init_single(I2C_HandleTypeDef* hi2c, VL53L0X_DEV Dev, GPIO_TypeDef* Xshut, uint16_t XshutPin)
{
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    Dev->I2cHandle = hi2c;
    Dev->I2cDevAddr = 0x52;

    HAL_GPIO_WritePin(Xshut, XshutPin, GPIO_PIN_RESET); // Disable XSHUT
    HAL_Delay(5);
    HAL_GPIO_WritePin(Xshut, XshutPin, GPIO_PIN_SET); // Enable XSHUT
    HAL_Delay(5);

    VL53L0X_WaitDeviceBooted( Dev );
    VL53L0X_DataInit( Dev );
    VL53L0X_StaticInit( Dev );
    VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    // Enable/Disable Sigma and Signal check
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
    VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

float vl53l0x_read_distance(VL53L0X_DEV Dev)
{
    VL53L0X_RangingMeasurementData_t RangingData;
    VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
    return static_cast<float>(RangingData.RangeMilliMeter);
}

