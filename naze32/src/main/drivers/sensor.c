#include <stdbool.h>

#include <platform.h>

#include "sensors/sensors.h"

#include "drivers/sensor.h"

#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_mpu3050.h"
#include "drivers/accgyro_mpu6050.h"
#include "drivers/accgyro_mpu6500.h"

#include "drivers/barometer_bmp280.h"

#include "drivers/compass_hmc5883l.h"
#include "drivers/compass_qmc5883l.h"


const sensorDetect_t gyro_detect[] = {
#ifdef USE_GYRO_MPU6050
    #ifdef GYRO_MPU6050_ALIGN
    { GYRO_MPU6050, GYRO_MPU6050_ALIGN, (sensorDetectFuncPtr)mpu6050GyroDetect, NULL },
    #else        
    { GYRO_MPU6050, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6050GyroDetect, NULL },
    #endif
#endif

#ifdef USE_GYRO_MPU3050
    #ifdef GYRO_MPU3050_ALIGN
    { GYRO_MPU3050, GYRO_MPU3050_ALIGN, (sensorDetectFuncPtr)mpu3050Detect, NULL },
    #else        
    { GYRO_MPU3050, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu3050Detect, NULL },
    #endif
#endif

#ifdef USE_GYRO_MPU6500
    #ifdef GYRO_MPU6500_ALIGN
    { GYRO_MPU6500, GYRO_MPU6500_ALIGN, (sensorDetectFuncPtr)mpu6500GyroDetect, NULL },
    #else        
    { GYRO_MPU6500, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6500GyroDetect, NULL },
    #endif
#endif

    { GYRO_NONE, IMUS_ALIGN_DEFAULT, NULL, NULL }
};


const sensorDetect_t acc_detect[] = {
#ifdef USE_ACC_MPU6050
    #ifdef ACC_MPU6050_ALIGN
    { ACC_MPU6050, ACC_MPU6050_ALIGN, (sensorDetectFuncPtr)mpu6050AccDetect, NULL },
    #else
    { ACC_MPU6050, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6050AccDetect, NULL },
    #endif
#endif

#ifdef USE_ACC_MPU6500
    #ifdef ACC_MPU6500_ALIGN
    { ACC_MPU6500, ACC_MPU6500_ALIGN, (sensorDetectFuncPtr)mpu6500AccDetect, NULL },
    #else
    { ACC_MPU6500, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6500AccDetect, NULL },
    #endif
#endif

    { ACC_NONE, IMUS_ALIGN_DEFAULT, NULL, NULL }
};


const sensorDisable_t baro_disable[] = {
    { NULL, NULL }
};


const sensorDetect_t baro_detect[] = {
#ifdef USE_BARO_BMP280
    { BARO_BMP280, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)bmp280Detect, NULL },
#endif

    { BARO_NONE, IMUS_ALIGN_DEFAULT, NULL, NULL }
};


const sensorDetect_t mag_detect[] = {
#ifdef USE_MAG_HMC5883
    #ifdef MAG_HMC5883_ALIGN
    { MAG_HMC5883, MAG_HMC5883_ALIGN, (sensorDetectFuncPtr)hmc5883lDetect, NULL },
    #else
    { MAG_HMC5883, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)hmc5883lDetect, NULL },
    #endif
#endif

#ifdef USE_MAG_QMC5883
    #ifdef MAG_QMC5883_ALIGN
    { MAG_QMC5883, MAG_QMC5883_ALIGN, (sensorDetectFuncPtr)qmc5883lDetect, NULL },
    #else
    { MAG_QMC5883, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)qmc5883lDetect, NULL },
    #endif
#endif

    { MAG_NONE, IMUS_ALIGN_DEFAULT, NULL, NULL }
};
