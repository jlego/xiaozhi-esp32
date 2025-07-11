#include "cw2015.h"
#include <cmath>
#include <esp_log.h>

#define TAG "CW2015"

#define REG_VERSION 0x00
#define REG_VCELL   0x02
#define REG_SOC     0x04
#define REG_ALERT   0x06
#define REG_CONFIG  0x08
#define REG_MODE    0x0A

CW2015::CW2015(i2c_master_bus_handle_t i2c_bus, uint8_t i2c_addr) 
    : I2cDevice(i2c_bus, i2c_addr) 
{
    qstart();
}

float CW2015::voltage() {
    uint8_t buffer[2];
    ReadRegs(REG_VCELL, buffer, 2);
    uint16_t raw = (buffer[0] << 8) | buffer[1];
    return raw * 0.305f / 1000.0f;
}

float CW2015::capacity() {
    uint8_t buffer[2];
    ReadRegs(REG_SOC, buffer, 2);
    return buffer[0] / 256.0f;
}

bool CW2015::isCharging() {
    uint8_t buffer[2];
    ReadRegs(REG_SOC, buffer, 2);
    return (buffer[0] & 0x01) != 0;
}

void CW2015::qstart() {
    WriteReg(REG_MODE, 0x00);
}

void CW2015::goSleep() {
    WriteReg(REG_MODE, 0x11);
}
