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

#pragma once

#define GYRO_NONE       0
#define GYRO_DEFAULT    SENSOR_DEFAULT
#define GYRO_MPU6050    2
#define GYRO_MPU3050    3
#define GYRO_MPU6500    4

// Type of accelerometer used/detected
#define ACC_NONE        0
#define ACC_DEFAULT     SENSOR_DEFAULT
#define ACC_ADXL345     2
#define ACC_MPU6050     3
#define ACC_MMA8452     4
#define ACC_BMA280      5
#define ACC_MPU6500     6

// Type of magnetometer used/detected
#define MAG_NONE        0
#define MAG_DEFAULT     SENSOR_DEFAULT
#define MAG_HMC5883     2
#define MAG_QMC5883     3

#define BARO_NONE       0
#define BARO_DEFAULT    SENSOR_DEFAULT
#define BARO_BMP085     2
#define BARO_MS5611     3
#define BARO_BMP280     4
