#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "sensors.h"

sensor_link_t sensor_link;

void initSensorLink()
{
    memset(&sensor_link, 0, sizeof(sensor_link_t));
    sensor_link.acc.acc_1G = 256;               // this is the 1G measured acceleration.
}