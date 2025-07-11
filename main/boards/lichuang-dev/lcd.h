#ifndef __LCD_H
#define __LCD_H		

#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// LCD设备结构体
typedef struct  
{										    
	uint16_t width;		    // LCD宽度
	uint16_t height;	    // LCD高度
	uint16_t id;		    // LCD ID
	uint8_t  dir;		    // 显示方向
	uint16_t wramcmd;		// 写GRAM命令
	uint16_t setxcmd;		// 设置X坐标命令
	uint16_t setycmd;		// 设置Y坐标命令
	uint8_t  xoffset;       // X偏移
	uint8_t	 yoffset;	    // Y偏移
} _lcd_dev;

extern _lcd_dev lcddev;	

#define USE_HORIZONTAL  	 0
#define USE_VERTICAL_90      1  // 90度旋转，横屏
#define USE_VERTICAL_180     2  // 180度旋转
#define USE_VERTICAL_270     3  // 270度旋转，横屏

// LCD尺寸定义 - 按照Arduino版本
#define LCD_W 240
#define LCD_H 283
 
extern uint16_t POINT_COLOR;  
extern uint16_t BACK_COLOR; 

// GPIO引脚定义（按 ESP-IDF 格式）
#define LCD_LED  (gpio_num_t)42        
#define LCD_CS   (gpio_num_t)16       
#define LCD_RS   (gpio_num_t)39        // LCD_DC    
#define LCD_RST  (gpio_num_t)-1        // 可选复位引脚，如未使用设置为 -1

// SPI引脚定义
#define VSPI_MISO (gpio_num_t)-1
#define VSPI_MOSI (gpio_num_t)40
#define VSPI_SCLK (gpio_num_t)41
#define VSPI_SS   (gpio_num_t)16       // CS

// GPIO控制宏定义
#define LCD_BLK_Clr()  gpio_set_level(LCD_LED, 0)
#define LCD_BLK_Set()  gpio_set_level(LCD_LED, 1)

#define LCD_CS_SET     gpio_set_level(LCD_CS, 1)
#define LCD_CS_CLR     gpio_set_level(LCD_CS, 0)

#define LCD_RS_SET     gpio_set_level(LCD_RS, 1)
#define LCD_RS_CLR     gpio_set_level(LCD_RS, 0)

#define LCD_RST_SET()  if (LCD_RST >= 0) gpio_set_level(LCD_RST, 1)
#define LCD_RST_CLR()  if (LCD_RST >= 0) gpio_set_level(LCD_RST, 0)

// 颜色定义 - 使用更饱和的颜色
#define WHITE       0xFFFF
#define BLACK      	0x0000
#define BLUE       	0x001F
#define BRED        0xF81F
#define GRED 		0xFFE0
#define GBLUE		0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 		0xBC40 
#define BRRED 		0xFC07
#define GRAY  		0x8430 
#define GRAY0       0xEF7D 
#define GRAY1       0x8410      	
#define GRAY2       0x4208 

// 更饱和的颜色定义
#define DARK_RED    0x8000
#define DARK_GREEN  0x0400
#define DARK_BLUE   0x0010
#define BRIGHT_RED  0xF800
#define BRIGHT_GREEN 0x07E0
#define BRIGHT_BLUE  0x001F

#define DARKBLUE    0x01CF
#define LIGHTBLUE   0x7D7C
#define GRAYBLUE    0x5458 

#define LIGHTGREEN  0x841F 
#define LIGHTGRAY   0xEF5B 
#define LGRAY 		0xC618 

#define LGRAYBLUE   0xA651
#define LBBLUE      0x2B12

// 函数声明（保留原函数名）
void LCD_Init(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Clear(uint16_t Color);	 
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_DrawPoint(uint16_t x, uint16_t y);
uint16_t LCD_ReadPoint(uint16_t x, uint16_t y); 
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);		   

void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd);

uint16_t LCD_RD_DATA(void);							    
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WR_DATA(uint8_t data);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(uint16_t RGB_Code);
uint16_t LCD_ReadRAM(void);		   

uint16_t LCD_BGR2RGB(uint16_t c);
void LCD_SetParam(void);
void Lcd_WriteData_16Bit(uint16_t Data);
void LCD_set_direction(uint8_t lcd_direction);
void LCD_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);			 
void LCD_Fill_hv(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);

// 内部函数
void LCD_WR_REG(uint8_t data);
void LCD_GPIOInit(void);
void LCD_RESET(void);
uint8_t SPI_WriteByte(int SPIx, uint8_t Byte, uint8_t cmd);
uint8_t SPI_WritePixels(uint16_t* pixels, int count);
// uint8_t LCD_WritePixels_Batch(uint16_t* pixels, int count);  // 暂时禁用

#ifdef __cplusplus
}
#endif

#endif
