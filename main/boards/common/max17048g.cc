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
 * Filename: LiMAX17048G.cpp
 */


#include "max17048g.h"
#include <esp_log.h>

#define TAG "MAX17048G"

// Initializes variables and the Wire library
MAX17048G::MAX17048G(i2c_master_bus_handle_t i2c_bus, gaugeType ic) 
    : I2cDevice(i2c_bus, MAX1704X_ADDR), _ic(ic), _f(NULL) 
{ 
}

// Initializes varaibles and the Wire library
// Assigns ISR f to interrupt intr (for Alert Interrupt)
MAX17048G::MAX17048G(i2c_master_bus_handle_t i2c_bus, gaugeType ic, gpio_num_t intr, func f) 
    : I2cDevice(i2c_bus, MAX1704X_ADDR), _ic(ic), _f(f)
{
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.pin_bit_mask = (1ULL << intr);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&io_conf);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(intr, (gpio_isr_t)f, NULL);
}

// Returns a measurement of the voltage of the connected 1S / 2S LiIon battery
// 0-5V range w/ 1.25mV resolution for the MAX17048
// 0-10V range w/ 2.5mV resolution for the MAX17049
double MAX17048G::getVoltage()
{
    uint8_t buffer[2];
    ReadRegs(MAX1704X_VCELL_ADDR, buffer, 2);
	return ( (buffer[0] << 4) + (buffer[1] >> 4) ) * 0.00125 * (double)_ic;
}

// Returns the relative state of charge of the connected LiIon Polymer battery
// as a percentage of the full capacity w/ resolution 1/256%
double MAX17048G::getSOC()
{
    uint8_t buffer[2];
    ReadRegs(MAX1704X_SOC_ADDR, buffer, 2);
	return buffer[0] + (double) buffer[1] / 256;
}

// Returns the production version of the IC
int MAX17048G::getVersion() 
{
	uint8_t buffer[2];
    ReadRegs(MAX1704X_VERSION_ADDR, buffer, 2);
	return ( buffer[0] << 8 ) + buffer[1];
}

// Return the value used to optimize IC performance to different operating conditions
uint8_t MAX17048G::getCompensation()
{
	return ReadReg(MAX1704X_RCOMP_ADDR);
}

// Return the alert threshold as a percentage, below an alert interrupt is generated
uint8_t MAX17048G::getAlertThreshold() 
{
    return ( ~getStatus() & 0x1F ) + 1;
}

// Return the LSByte of the CONFIG register
uint8_t MAX17048G::getStatus()
{
	return ReadReg(MAX1704X_ATHRD_ADDR);
}

// Sets a value to the MSB of the CONFIG register used 
// to optimizethe  IC performance to different operating conditions
uint8_t MAX17048G::setCompensation(uint8_t comp)
{	
	uint8_t status = getStatus();
	WriteReg(MAX1704X_CONFIG_ADDR, comp);
	WriteReg(MAX1704X_CONFIG_ADDR + 1, status);
	return 0;
}

// Sets the alert threshold below which an alert interrupt is generated
// The acceptable range is 1-32%. Default threshold is 4%
uint8_t MAX17048G::setAlertThreshold(uint8_t thrd)
{
	if ( thrd > 32 ) thrd = 32;
	else if ( thrd < 1 ) thrd = 1;
	thrd = ( ~thrd + 1 ) & 0x1F;
	
	uint8_t comp = ReadReg(MAX1704X_CONFIG_ADDR);
	uint8_t sleepBit = ReadReg(MAX1704X_CONFIG_ADDR + 1) & 0x80;
	
	WriteReg(MAX1704X_CONFIG_ADDR, comp);
	WriteReg(MAX1704X_CONFIG_ADDR + 1, sleepBit | thrd);
	return 0;
}

// After an alert interrupt has been generated,
// it clears the alert bit on the CONFIG register
uint8_t MAX17048G::clearAlertInterrupt()
{
	uint8_t compensation = getCompensation();
	uint8_t status = getStatus();
	WriteReg(MAX1704X_CONFIG_ADDR, compensation);
	WriteReg(MAX1704X_CONFIG_ADDR + 1, 0xDF & status);
	return 0;
}

// It puts the MAX1704X to sleep
// All IC operations are halted
uint8_t MAX17048G::sleep()
{
	uint8_t compensation = getCompensation();
	uint8_t threshold = getAlertThreshold();
	
	WriteReg(MAX1704X_CONFIG_ADDR, compensation);
	WriteReg(MAX1704X_CONFIG_ADDR + 1, 0x80 | threshold);
	return 0;
}

// It wakes the MAX1704X from sleep mode
uint8_t MAX17048G::wake()
{
	uint8_t compensation = getCompensation();
	uint8_t threshold = getAlertThreshold();
	
	WriteReg(MAX1704X_CONFIG_ADDR, compensation);
	WriteReg(MAX1704X_CONFIG_ADDR + 1, 0x7F & threshold);
	return 0;
}

// whether the MAX1704X is in sleep mode
bool MAX17048G::sleeping()
{
	return ( getStatus() & 0x80 ) == 0x80;
}

// It forces the MAX1704X to
// restart fuel-gauge calculations
// uint8_t MAX17048G::quickStart()
// {
// 	WriteReg(MAX1704X_MODE_ADDR, 0x40);
// 	WriteReg(MAX1704X_MODE_ADDR + 1, 0x00);
// 	return 0;
// }

// It forces the MAX1704X to completely reset
uint8_t MAX17048G::reset()
{
	WriteReg(MAX1704X_COMMAND_ADDR, 0x54);
	WriteReg(MAX1704X_COMMAND_ADDR + 1, 0x00);
	return 0;
}

bool MAX17048G::isCharging(float last_voltage, float last_soc) {
	float voltage = getVoltage();
	float soc = getSOC();
	return (voltage - last_voltage > 0.01f) && (soc - last_soc > 0.1f);
}