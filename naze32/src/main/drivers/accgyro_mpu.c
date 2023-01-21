/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <platform.h>
#include "build_config.h"
#include "debug.h"

#include "core/pif_i2c.h"

#include "common/maths.h"

#include "nvic.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "bus_i2c.h"
#include "gyro_sync.h"

#include "accgyro_mpu3050.h"
#include "accgyro_mpu6050.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6500.h"
#include "accgyro_mpu.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

//#define DEBUG_MPU_DATA_READY_INTERRUPT

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);

static void mpu6050FindRevision(void);

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void);
#endif

mpuDetectionResult_t mpuDetectionResult;

mpuConfiguration_t mpuConfiguration;
static const extiConfig_t *mpuIntExtiConfig = NULL;

#define MPU_ADDRESS             0x68

const extiConfig_t *selectMPUIntExtiConfig(void)
{
#ifdef NAZE
    // MPU_INT output on rev4 PB13
    static const extiConfig_t nazeRev4MPUIntExtiConfig = {
            .gpioAPB2Peripherals = RCC_APB2Periph_GPIOB,
            .gpioPin = Pin_13,
            .gpioPort = GPIOB,
            .exti_port_source = GPIO_PortSourceGPIOB,
            .exti_line = EXTI_Line13,
            .exti_pin_source = GPIO_PinSource13,
            .exti_irqn = EXTI15_10_IRQn
    };
    // MPU_INT output on rev5 hardware PC13
    static const extiConfig_t nazeRev5MPUIntExtiConfig = {
            .gpioAPB2Peripherals = RCC_APB2Periph_GPIOC,
            .gpioPin = Pin_13,
            .gpioPort = GPIOC,
            .exti_port_source = GPIO_PortSourceGPIOC,
            .exti_line = EXTI_Line13,
            .exti_pin_source = GPIO_PinSource13,
            .exti_irqn = EXTI15_10_IRQn
    };

    if (hardwareRevision < NAZE32_REV5) {
        return &nazeRev4MPUIntExtiConfig;
    } else {
        return &nazeRev5MPUIntExtiConfig;
    }
#endif

    return NULL;
}

mpuDetectionResult_t *detectMpu()
{
    memset(&mpuDetectionResult, 0, sizeof(mpuDetectionResult));
    memset(&mpuConfiguration, 0, sizeof(mpuConfiguration));

    mpuIntExtiConfig = selectMPUIntExtiConfig();

    bool ack;
    uint8_t sig;
    uint8_t inquiryResult;

    // MPU datasheet specifies 30ms.
    delay(35);

    ack = actI2cRead(MPU_ADDRESS, MPU_RA_WHO_AM_I, 1, &sig, 1);
    if (ack) {
        mpuConfiguration.read = mpuReadRegisterI2C;
        mpuConfiguration.write = mpuWriteRegisterI2C;
    } else {
#ifdef USE_SPI
        bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult();
        UNUSED(detectedSpiSensor);
#endif

        return &mpuDetectionResult;
    }

    mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;

    // If an MPU3050 is connected sig will contain 0.
    ack = actI2cRead(MPU_ADDRESS, MPU_RA_WHO_AM_I_LEGACY, 1, &inquiryResult, 1);
    inquiryResult &= MPU_INQUIRY_MASK;
    if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_3050;
        mpuConfiguration.gyroReadXRegister = MPU3050_GYRO_OUT;
        return &mpuDetectionResult;
    }

    sig &= MPU_INQUIRY_MASK;

    if (sig == MPUx0x0_WHO_AM_I_CONST) {

        mpuDetectionResult.sensor = MPU_60x0;

        mpu6050FindRevision();
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_65xx_I2C;
    }

    return &mpuDetectionResult;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void)
{
#ifdef USE_GYRO_SPI_MPU6500
    if (mpu6500SpiDetect()) {
        mpuDetectionResult.sensor = MPU_65xx_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu6500ReadRegister;
        mpuConfiguration.write = mpu6500WriteRegister;
        return true;
    }
#endif

    return false;
}
#endif

static void mpu6050FindRevision(void)
{
    bool ack;
    UNUSED(ack);

    uint8_t readBuffer[6];
    uint8_t revision;
    uint8_t productId;

    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and accel revision
    ack = mpuConfiguration.read(MPU_RA_XA_OFFS_H, 6, readBuffer);
    revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (revision) {
        /* Congrats, these parts are better. */
        if (revision == 1) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else if ((revision == 3) || (revision == 7)) {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        ack = mpuConfiguration.read(MPU_RA_PRODUCT_ID, 1, &productId);
        revision = productId & 0x0F;
        if (!revision) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}

void MPU_DATA_READY_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(mpuIntExtiConfig->exti_line) == RESET) {
        return;
    }

    EXTI_ClearITPendingBit(mpuIntExtiConfig->exti_line);

    if (sensor_link.gyro.p_task) sensor_link.gyro.p_task->immediate = true;;

#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
    // Measure the delta in micro seconds between calls to the interrupt handler
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;

    uint32_t now = micros();
    callDelta = now - lastCalledAt;

    //UNUSED(callDelta);
    debug[0] = callDelta;

    lastCalledAt = now;
#endif
}

static void configureMPUDataReadyInterruptHandling(void)
{
#ifdef USE_MPU_DATA_READY_SIGNAL

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(mpuIntExtiConfig->exti_port_source, mpuIntExtiConfig->exti_pin_source);
#endif

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = GPIO_ReadInputDataBit(mpuIntExtiConfig->gpioPort, mpuIntExtiConfig->gpioPin);
    if (status) {
        return;
    }
#endif

    registerExtiCallbackHandler(mpuIntExtiConfig->exti_irqn, MPU_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(mpuIntExtiConfig->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = mpuIntExtiConfig->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = mpuIntExtiConfig->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

bool mpuIntExtiInit()
{
    gpio_config_t gpio;

    static bool mpuExtiInitDone = false;

    if (!mpuIntExtiConfig) return false;

    if (mpuExtiInitDone) return true;

#ifdef STM32F10X
        if (mpuIntExtiConfig->gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(mpuIntExtiConfig->gpioAPB2Peripherals, ENABLE);
        }
#endif

    gpio.pin = mpuIntExtiConfig->gpioPin;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(mpuIntExtiConfig->gpioPort, &gpio);

    configureMPUDataReadyInterruptHandling();

    mpuExtiInitDone = true;
    return true;
}

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data)
{
    bool ack = i2cRead(MPU_ADDRESS, reg, length, data);
    return ack;
}

static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
    bool ack = i2cWrite(MPU_ADDRESS, reg, data);
    return ack;
}

bool mpuAccRead(int32_t *accel)
{
    uint8_t data[6];
    int16_t accData[AXIS_COUNT];
    int axis;

    if (sensor_link.imu_sensor._measure & IMU_MEASURE_ACCELERO) {
        return pifImuSensor_ReadAccel4(&sensor_link.imu_sensor, accel);
    }

    bool ack = mpuConfiguration.read(MPU_RA_ACCEL_XOUT_H, 6, data);
    if (!ack) {
        return false;
    }

    accData[0] = (int16_t)((data[0] << 8) | data[1]);
    accData[1] = (int16_t)((data[2] << 8) | data[3]);
    accData[2] = (int16_t)((data[4] << 8) | data[5]);

    for (axis = 0; axis < AXIS_COUNT; axis++) accel[axis] = accData[axis];  // int32_t copy to work with
    alignSensors(accel, accel, sensor_link.acc.align);

    return true;
}

bool mpuGyroRead(int32_t *gyro)
{
    uint8_t data[6];
    int16_t gyroADC[AXIS_COUNT];
    int axis;

    if (sensor_link.imu_sensor._measure & IMU_MEASURE_GYROSCOPE) {
        return pifImuSensor_ReadGyro4(&sensor_link.imu_sensor, gyro);
    }

    bool ack = mpuConfiguration.read(mpuConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

    gyroADC[0] = (int16_t)((data[0] << 8) | data[1]);
    gyroADC[1] = (int16_t)((data[2] << 8) | data[3]);
    gyroADC[2] = (int16_t)((data[4] << 8) | data[5]);

    for (axis = 0; axis < AXIS_COUNT; axis++) gyro[axis] = gyroADC[axis];  // int32_t copy to work with
    alignSensors(gyro, gyro, sensor_link.gyro.align);

    return true;
}
