#ifndef __LCD_H
#define __LCD_H		
//#include "sys.h"	 

#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <lvgl.h>
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

typedef struct  
{										    
	uint16_t width;			
	uint16_t height;			
	uint16_t id;				
	uint8_t  dir;			 
	uint16_t	 wramcmd;		
	uint16_t  setxcmd;		
	uint16_t  setycmd;		 
	uint8_t   xoffset;    
  uint8_t	 yoffset;	
}_lcd_dev; 


extern _lcd_dev lcddev;	

#define USE_HORIZONTAL  	 	 0
#define USE_VERTICAL_90      1  // 90度旋转，横屏
#define USE_VERTICAL_180     2  // 180度旋转
#define USE_VERTICAL_270     3  // 270度旋转，横屏

//////////////////////////////////////////////////////////////////////////////////	  

extern uint16_t POINT_COLOR;  
extern uint16_t BACK_COLOR; 

// ==========================C3A2小板v2.1 1.73寸IPS==============================
// #if CONFIG_BOARD_TYPE_ESP_WATCH_C3A2
#define LCD_LED  (gpio_num_t)8       
#define LCD_CS   (gpio_num_t)10
#define LCD_DC   (gpio_num_t)-1  
#define LCD_RST  (gpio_num_t)-1       
#define VSPI_MISO (gpio_num_t)-1
#define VSPI_MOSI (gpio_num_t)13
#define VSPI_SCLK (gpio_num_t)12
#define VSPI_SS   (gpio_num_t)10
// #elseif CONFIG_BOARD_TYPE_ESP_WATCH_C3A
// ==========================C3A小板v2.0 1.73寸
// #define LCD_LED  (gpio_num_t)20       
// #define LCD_CS   (gpio_num_t)21
// #define LCD_DC   (gpio_num_t)-1  
// #define LCD_RST  (gpio_num_t)-1       
// #define VSPI_MISO (gpio_num_t)-1
// #define VSPI_MOSI (gpio_num_t)4
// #define VSPI_SCLK (gpio_num_t)12
// #define VSPI_SS   (gpio_num_t)21
// #else
// #define LCD_LED  (gpio_num_t)8       
// #define LCD_CS   (gpio_num_t)9
// #define LCD_DC   (gpio_num_t)10  
// #define LCD_RST  (gpio_num_t)11       
// #define VSPI_MISO (gpio_num_t)12
// #define VSPI_MOSI (gpio_num_t)13
// #define VSPI_SCLK (gpio_num_t)14
// #define VSPI_SS   (gpio_num_t)9
// #endif

#define LCD_BLK_CLR()  gpio_set_level(LCD_LED, 0)
#define LCD_BLK_SET()  gpio_set_level(LCD_LED, 1)

#define LCD_RST_SET()  if (LCD_RST >= 0) gpio_set_level(LCD_RST, 1)
#define LCD_RST_CLR()  if (LCD_RST >= 0) gpio_set_level(LCD_RST, 0)

// inline void LCD_CS_CLR() { GPIO.out_w1tc = (1 << LCD_CS); } // 拉低
// inline void LCD_CS_SET() { GPIO.out_w1ts = (1 << LCD_CS); } // 拉高
// inline void LCD_DC_CLR() { GPIO.out_w1tc = (1 << LCD_DC); } // 拉低
// inline void LCD_DC_SET() { GPIO.out_w1ts = (1 << LCD_DC); } // 拉高

#define LCD_CS_CLR()  gpio_set_level(LCD_CS, 0)
#define LCD_CS_SET()  gpio_set_level(LCD_CS, 1)
#define LCD_DC_CLR()  gpio_set_level(LCD_DC, 0)
#define LCD_DC_SET()  gpio_set_level(LCD_DC, 1)

#define WHITE       0xFFFF
#define BLACK      	0x0000
#define BLUE       	0x001F
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 
#define BRRED 			0XFC07
#define GRAY  			0X8430 
#define GRAY0       0xEF7D 
#define GRAY1       0x8410      	
#define GRAY2       0x4208 


#define DARKBLUE      	 0X01CF
#define LIGHTBLUE      	 0X7D7C
#define GRAYBLUE       	 0X5458 

 
#define LIGHTGREEN     	0X841F 
#define LIGHTGRAY     0XEF5B 
#define LGRAY 			 		0XC618 

#define LGRAYBLUE      	0XA651
#define LBBLUE          0X2B12
	    															  
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
#endif  
