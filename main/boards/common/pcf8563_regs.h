#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// PCF8563寄存器地址定义
#define REG_CTRL_STATUS1 0x00
#define REG_CTRL_STATUS2 0x01
#define REG_VL_SECONDS   0x02
#define REG_MINUTES      0x03
#define REG_HOURS        0x04
#define REG_DAYS         0x05
#define REG_WEEKDAYS     0x06
#define REG_CENT_MONTHS  0x07
#define REG_YEARS        0x08
#define REG_ALARM_MIN    0x09
#define REG_ALARM_HOUR   0x0a
#define REG_ALARM_DAY    0x0b
#define REG_ALARM_WDAY   0x0c
#define REG_CLKOUT       0x0d
#define REG_TIMER_CTRL   0x0e
#define REG_TIMER        0x0f

// 位定义
#define BIT_STOP         5
#define BIT_VL           7
#define BIT_AE           7
#define BIT_CLKOUT_FD    0
#define BIT_CLKOUT_FE    7

#define BIT_CTRL_STATUS2_TIE 0
#define BIT_CTRL_STATUS2_AIE 1
#define BIT_CTRL_STATUS2_TF  3
#define BIT_CTRL_STATUS2_AF  4

#define BIT_TIMER_CTRL_TE 7

#define MASK_TIMER_CTRL_TD 0x03
#define MASK_ALARM        0x7f

#define MASK_MIN         0x7f
#define MASK_HOUR        0x3f
#define MASK_MDAY        0x3f
#define MASK_WDAY        0x07
#define MASK_MON         0x1f

#ifdef __cplusplus
}
#endif