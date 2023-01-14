#include <stdbool.h>

#include <platform.h>

#include "sensors/sensors.h"

#include "drivers/sensor.h"

#include "drivers/accgyro_adxl345.h"
#include "drivers/accgyro_bma280.h"
#include "drivers/accgyro_mma845x.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_mpu3050.h"
#include "drivers/accgyro_mpu6050.h"
#include "drivers/accgyro_mpu6500.h"
#include "drivers/accgyro_spi_mpu6500.h"

#include "drivers/barometer_bmp085.h"
#include "drivers/barometer_bmp280.h"
#include "drivers/barometer_ms5611.h"

#include "drivers/compass_hmc5883l.h"


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

#ifdef USE_GYRO_SPI_MPU6500
    #ifdef GYRO_MPU6500_ALIGN
    { GYRO_MPU6500, GYRO_MPU6500_ALIGN, (sensorDetectFuncPtr)mpu6500SpiGyroDetect, NULL },
    #else        
    { GYRO_MPU6500, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6500SpiGyroDetect, NULL },
    #endif
#endif

    { GYRO_NONE, IMUS_ALIGN_DEFAULT, NULL, NULL }
};


#ifdef USE_ACC_ADXL345
static const drv_adxl345_config_t acc_params = {
    .useFifo = false,
    .dataRate = 800     // unused currently
};
#endif

const sensorDetect_t acc_detect[] = {
#ifdef USE_ACC_ADXL345
    #ifdef ACC_ADXL345_ALIGN
    { ACC_ADXL345, ACC_ADXL345_ALIGN, (sensorDetectFuncPtr)adxl345Detect, (void*)&acc_params },
    #else
    { ACC_ADXL345, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)adxl345Detect, (void*)&acc_params },
    #endif
#endif

#ifdef USE_ACC_MPU6050
    #ifdef ACC_MPU6050_ALIGN
    { ACC_MPU6050, ACC_MPU6050_ALIGN, (sensorDetectFuncPtr)mpu6050AccDetect, NULL },
    #else
    { ACC_MPU6050, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6050AccDetect, NULL },
    #endif
#endif

#ifdef USE_ACC_MMA8452
    #ifdef ACC_MMA8452_ALIGN
    { ACC_MMA8452, ACC_MMA8452_ALIGN, (sensorDetectFuncPtr)mma8452Detect, NULL },
    #else
    { ACC_MMA8452, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mma8452Detect, NULL },
    #endif
#endif

#ifdef USE_ACC_BMA280
    #ifdef ACC_BMA280_ALIGN
    { ACC_BMA280, ACC_BMA280_ALIGN, (sensorDetectFuncPtr)bma280Detect, NULL },
    #else
    { ACC_BMA280, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)bma280Detect, NULL },
    #endif
#endif

#ifdef USE_ACC_MPU6500
    #ifdef ACC_MPU6500_ALIGN
    { ACC_MPU6500, ACC_MPU6500_ALIGN, (sensorDetectFuncPtr)mpu6500AccDetect, NULL },
    #else
    { ACC_MPU6500, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6500AccDetect, NULL },
    #endif
#endif

#ifdef USE_ACC_SPI_MPU6500
    #ifdef ACC_MPU6500_ALIGN
    { ACC_MPU6500, ACC_MPU6500_ALIGN, (sensorDetectFuncPtr)mpu6500SpiAccDetect, NULL },
    #else
    { ACC_MPU6500, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)mpu6500SpiAccDetect, NULL },
    #endif
#endif

    { ACC_NONE, IMUS_ALIGN_DEFAULT, NULL, NULL }
};


const sensorDisable_t baro_disable[] = {
    { (sensorDisableFuncPtr)bmp085Disable, NULL },
    { NULL, NULL }
};


const sensorDetect_t baro_detect[] = {
#ifdef USE_BARO_MS5611
    { BARO_MS5611, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)ms5611Detect, NULL },
#endif

#ifdef USE_BARO_BMP085
    { BARO_BMP085, IMUS_ALIGN_DEFAULT, (sensorDetectFuncPtr)bmp085Detect, NULL },
#endif

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

    { MAG_NONE, IMUS_ALIGN_DEFAULT, NULL, NULL }
};
