#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sensors.h"

sensor_link_t sensor_link;

void initSensorLink(boardAlignment_t *boardAlignment)
{
    memset(&sensor_link, 0, sizeof(sensor_link_t));
    sensor_link.acc.acc_1G = 256;               // this is the 1G measured acceleration.

    pifImuSensor_Init(&sensor_link.imu_sensor);
    pifImuSensor_InitBoardAlignment(&sensor_link.imu_sensor, boardAlignment->rollDegrees, boardAlignment->pitchDegrees, boardAlignment->yawDegrees);
}
