#include <stdbool.h>

#include "core/pif_i2c.h"

#include "bus_i2c.h"

PifI2cPort g_i2c_port;

PifI2cReturn actI2cWrite(uint8_t addr_, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
    (void)isize;

    i2cWriteBuffer(addr_, iaddr, size, p_data);
    return IR_COMPLETE;
}

PifI2cReturn actI2cRead(uint8_t addr_, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
    (void)isize;

    i2cRead(addr_, iaddr, size, p_data);
    return IR_COMPLETE;
}

BOOL initI2cDevice(I2CDevice index)
{
    i2cInit(index);

    if (!pifI2cPort_Init(&g_i2c_port, PIF_ID_AUTO, 5, 16)) return FALSE;
    g_i2c_port.act_read = actI2cRead;
    g_i2c_port.act_write = actI2cWrite;
    return TRUE;
}
