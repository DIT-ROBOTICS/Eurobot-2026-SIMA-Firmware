#ifndef VL53L0X_CTRL_HPP
#define VL53L0X_CTRL_HPP


#ifdef __cplusplus
extern "C" {
#endif
// Include statements
#include "vl53l0x_api.h"
#include "main.h"

// Function declarations
void vl53l0x_init_single(I2C_HandleTypeDef* hi2c, VL53L0X_DEV Dev, GPIO_TypeDef* Xshut, uint16_t XshutPin);
float vl53l0x_read_distance(VL53L0X_DEV Dev);

#ifdef __cplusplus
}

#endif


#endif // VL53L0X_CTRL_HPP
