#pragma once

#include "i2c_device.h"
#include <driver/i2c_master.h>

class CW2015 : public I2cDevice {
public:
    explicit CW2015(i2c_master_bus_handle_t i2c_bus, uint8_t i2c_addr = 0x62);

    float voltage();
    float capacity();
    bool isCharging();

    void qstart();
    void goSleep();

private:
    void init_i2c();
};
