/**
 * Name: LiMAX17048G
 * Author: Sarojit Koley <koley.sarojit.67@gmail.com>
 * Version: 1.0
 * Description: A library for interfacing the Analog Devices MAXIM MAX17048G+T10
 * 				Lithium fuel gauges. These ICs report the relative state of charge
 * 				of the connected Lithium Ion Polymer battery, and the library 
 * 				can help you configure them and communicate with them
 * Source: https://github.com/SAROJIT67/LiMAX17048G
 * License: Copyright (c) 2024 Sarojit Koley
 *          This library is licensed under the MIT license
 * Filename: LiMAX17048G.h
 */

//#include "Arduino.h"

#ifndef _MAX17048_H
#define _MAX17048_H

#include "i2c_device.h"
#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>

// Funtion ISR for the ALERT Interrupt
typedef void (*func)();

// Names of the two supported ICs
// Used for reporting the correct voltage measurement (see getVoltage method)
enum gaugeType
{
	MAX17048 = 1,
	MAX17049 = 2
};

// MAX1704X register addresses
#define MAX1704X_ADDR 0x36

#define MAX1704X_VCELL_ADDR 0x02
#define MAX1704X_SOC_ADDR 0x04
#define MAX1704X_MODE_ADDR 0x06
#define MAX1704X_VERSION_ADDR 0x08
#define MAX1704X_CONFIG_ADDR 0x0C
#define MAX1704X_RCOMP_ADDR 0x0C
#define MAX1704X_ATHRD_ADDR 0x0D
#define MAX1704X_COMMAND_ADDR 0xFE

class MAX17048G : public I2cDevice {
    public:
        MAX17048G(i2c_master_bus_handle_t i2c_bus, gaugeType ic);
        MAX17048G(i2c_master_bus_handle_t i2c_bus, gaugeType ic, gpio_num_t intr, func f);
        double getVoltage();
        double getSOC();
        int getVersion();
        uint8_t getCompensation();
        uint8_t getAlertThreshold();
        uint8_t setCompensation(uint8_t comp);
        uint8_t setAlertThreshold(uint8_t thrd);
        uint8_t clearAlertInterrupt();
        uint8_t sleep();
        uint8_t wake();
        bool sleeping();
        // uint8_t quickStart();
        uint8_t reset();
        bool isCharging(float last_voltage, float last_soc);
    
    private:
        gaugeType _ic;
        func _f;
        uint8_t getStatus();
};

#endif