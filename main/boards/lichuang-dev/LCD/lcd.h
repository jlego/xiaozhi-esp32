/****************************************************************************************************
//=========================================电源接线================================================//
//     LCD模块                STM32单片机
//      VCC          接          3.3V        //电源
//      GND          接          GND         //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为SPI总线
//     LCD模块                STM32单片机    
//    SDI(MOSI)      接          PA7         //液晶屏SPI总线数据写信号
//    SDO(MISO)      接          PA6         //液晶屏SPI总线数据读信号，如果不需要读，可以不接线
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 					      STM32单片机 
//       BLK         接          PB6         //液晶屏背光控制信号，如果不需要控制，接5V或3.3V
//       SCK         接          PA5         //液晶屏SPI总线时钟信号
//      DC/RS        接          PB7         //液晶屏数据/命令控制信号
//       RST         接          PB8         //液晶屏复位控制信号
//       CS          接          PB9         //液晶屏片选控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	 1.22寸模块模块             STM32单片机 
//      CTP_INT        接          PA1         //触摸屏触摸中断信号
//      CTP_SDA        接          PA9         //触摸屏SPI总线写信号
//      CTP_RST        接          PA10        //触摸屏片选控制信号
//      CTP_SCL        接          PB5         //触摸屏SPI总线时钟信号
**************************************************************************************************/		
#ifndef __LCD_H
#define __LCD_H		
#include "sys.h"	 
#include "stdlib.h"


//LCD重要参数集
typedef struct  
{										    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				  //LCD ID
	uint8_t  dir;			  //横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint16_t	 wramcmd;		//开始写gram指令
	uint16_t  setxcmd;		//设置x坐标指令
	uint16_t  setycmd;		//设置y坐标指令	 
	uint8_t   xoffset;    
  uint8_t	 yoffset;	
}_lcd_dev; 

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
/////////////////////////////////////用户配置区///////////////////////////////////	 
#define USE_HORIZONTAL  	 0//定义液晶屏顺时针旋转方向 	0-0度旋转，1-90度旋转，2-180度旋转，3-270度旋转

//////////////////////////////////////////////////////////////////////////////////	  
//定义LCD的尺寸
#define LCD_W 240
#define LCD_H 283

//TFTLCD部分外要调用的函数		   
extern u16  POINT_COLOR;//默认红色    
extern u16  BACK_COLOR; //背景颜色.默认为白色

////////////////////////////////////////////////////////////////////
//-----------------LCD端口定义---------------- 
#define GPIO_TYPE  GPIOB  //GPIO组类型
#define LED      6        //背光控制引脚        PB6
#define LCD_CS   9       //片选引脚            PB9
#define LCD_RS   7       //寄存器/数据选择引脚 PB7 
#define LCD_RST  8       //复位引脚            PB8


//QDtech全系列模块采用了三极管控制背光亮灭，用户也可以接PWM调节背光亮度
#define	LCD_LED PBout(LED) //LCD背光    		 PB6
//如果使用官方库函数定义下列底层，速度将会下降到14帧每秒，建议采用我司推荐方法
//以下IO定义直接操作寄存器，快速IO操作，刷屏速率可以达到28帧每秒！ 

//GPIO置位（拉高）
#define	LCD_CS_SET  GPIO_TYPE->BSRR=1<<LCD_CS    //片选端口  	
#define	LCD_RS_SET	GPIO_TYPE->BSRR=1<<LCD_RS    //数据/命令  	  
#define	LCD_RST_SET	GPIO_TYPE->BSRR=1<<LCD_RST   //复位			  

//GPIO复位（拉低）							    
#define	LCD_CS_CLR  GPIO_TYPE->BRR=1<<LCD_CS     //片选端口  
#define	LCD_RS_CLR	GPIO_TYPE->BRR=1<<LCD_RS     //数据/命令 	 
#define	LCD_RST_CLR	GPIO_TYPE->BRR=1<<LCD_RST    //复位			  

//画笔颜色
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
#define BROWN 			0XBC40 //棕色
#define BRRED 			0XFC07 //棕红色
#define GRAY  			0X8430 //灰色
#define GRAY0       0xEF7D 
#define GRAY1       0x8410      	//灰色1      00000 000000 00000
#define GRAY2       0x4208 
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	0X841F //浅绿色
#define LIGHTGRAY     0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 		0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE      	0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          0X2B12 //浅棕蓝色(选择条目的反色)
	    															  
void LCD_Init(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Clear(u16 Color);	 
void LCD_SetCursor(u16 Xpos, u16 Ypos);
void LCD_DrawPoint(u16 x,u16 y);//画点
u16  LCD_ReadPoint(u16 x,u16 y); //读点
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);		   
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);

u16 LCD_RD_DATA(void);//读取LCD数据									    
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
void LCD_WR_DATA(u8 data);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
u16 LCD_ReadRAM(void);		   
u16 LCD_BGR2RGB(u16 c);
void LCD_SetParam(void);
void Lcd_WriteData_16Bit(u16 Data);
void LCD_direction(u8 direction );
				  		 
#endif  

