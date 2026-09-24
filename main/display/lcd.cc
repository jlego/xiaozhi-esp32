#include "lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include "boards/codesfish/esp-watch-c3a2/config.h"

static const char *TAG = "LCD";

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define SPI1 SPI2_HOST  //SPI3_HOST

// SPI设备句柄
extern spi_device_handle_t spi_handle;
spi_device_handle_t spi_handle = NULL;
static const int spiClk = 40*1000*1000; // 提高到40MHz，提高传输速度
_lcd_dev lcddev;

// 延迟函数
#define delay_ms(x) vTaskDelay(pdMS_TO_TICKS(x))

uint16_t POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

u8 SPI_WriteByte(int SPIx, u8 Byte, u8 cmd) {
    spi_transaction_t t = {};
    uint8_t txbuf[2];
    txbuf[0] = cmd << 7 | Byte >> 1;
    txbuf[1] = Byte << 7;
    t.length = 16;                 // 发送 16 bit
    t.tx_buffer = txbuf;
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        return 1; // 返回错误，但不打印日志以减少开销
    }
    return 0; // 返回成功
}

/*****************************************************************************
 * @name       :void LCD_WR_REG(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(u8 data)
{ 
   LCD_CS_CLR();     
  //  LCD_DC_CLR();	  
   SPI_WriteByte(SPI1, data, 0);  // 移除错误检查，减少开销
   LCD_CS_SET(); 	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(u8 data)
{
   LCD_CS_CLR(); 
  //  LCD_DC_SET();
   SPI_WriteByte(SPI1, data, 1);  // 移除错误检查，减少开销
   LCD_CS_SET(); 
}

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(u16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(uint16_t Data)
{	
   // 保持原有的时序逻辑，但优化内部实现
   LCD_WR_DATA(Data>>8);
   LCD_WR_DATA(Data);
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :LCD_WR_DATA()into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

// void Lcd_WriteData_16Bit(uint16_t Data)
// {	
//     static uint8_t txbuf1[2], txbuf2[2];  // 使用静态变量避免栈分配
//     txbuf1[0] = 1 << 7 | ((Data>>8) >> 1);
//     txbuf1[1] = (Data>>8) << 7;
//     txbuf2[0] = (1 << 7) | (Data >> 1);
//     txbuf2[1] = (Data << 7) & 0x80;
//     static spi_transaction_t t1 = {  // 静态变量避免重复初始化
//         .length = 2 * 8,
//     };
//     static spi_transaction_t t2 = {
//         .length = 2 * 8,
//     };
//     t1.tx_buffer = txbuf1;
//     t2.tx_buffer = txbuf2;
//     // 队列传输
//     spi_transaction_t *rtrans1, *rtrans2;
//     spi_device_queue_trans(spi_handle, &t1, portMAX_DELAY);
//     spi_device_queue_trans(spi_handle, &t2, portMAX_DELAY);
//     // 等待传输完成
//     spi_device_get_trans_result(spi_handle, &rtrans1, portMAX_DELAY);
//     spi_device_get_trans_result(spi_handle, &rtrans2, portMAX_DELAY);
// }

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen (for-loop version, no DMA)
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/
void LCD_Clear(uint16_t Color)
{
    // 使用标准窗口设置，按照Arduino版本
    ESP_LOGI(TAG, "LCD_Clear: setting window to (0,0,%d,%d), total pixels: %lu", 
             lcddev.width - 1, lcddev.height - 1, (unsigned long)(lcddev.width * lcddev.height));
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);
    uint32_t total_pixels = lcddev.width * lcddev.height;
    LCD_WriteRAM_Prepare();
    for (uint32_t i = 0; i < total_pixels; i++) {
        Lcd_WriteData_16Bit(Color);  // 使用16位写入函数
        // 可选：每N次让出CPU，防止看门狗
        if (i % 2048 == 0) {
            vTaskDelay(1);
        }
    }
    ESP_LOGI(TAG, "LCD_Clear (for-loop) completed, wrote %lu pixels", (unsigned long)total_pixels);
} 

/*****************************************************************************
 * @name       :void LCD_GPIOInit(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_GPIOInit(void)
{
    ESP_LOGI(TAG, "Initializing LCD GPIO...");
    
    // 配置GPIO引脚
    gpio_config_t io_conf = {};
    
    // 配置LCD_LED引脚
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LCD_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    // 配置LCD_CS引脚
    io_conf.pin_bit_mask = (1ULL << (uint32_t)(int)LCD_CS);
    gpio_config(&io_conf);
    
    // 配置LCD_RS引脚
    // io_conf.pin_bit_mask = (1ULL << LCD_DC);
    // gpio_config(&io_conf);
    
    // 配置LCD_RST引脚（如果使用）
    if (LCD_RST >= 0) {
        int rst_pin = (int)LCD_RST;
        io_conf.pin_bit_mask = (1ULL << rst_pin);
        gpio_config(&io_conf);
    }
    
    // 配置SPI设备
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = spiClk;
    devcfg.mode = 0;
    devcfg.spics_io_num = LCD_CS;
    devcfg.queue_size = 7;
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI1, &devcfg, &spi_handle));
    ESP_LOGI(TAG, "LCD GPIO and SPI initialized successfully");
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	if (LCD_RST >= 0) {
		LCD_RST_CLR();
		delay_ms(100);	
		LCD_RST_SET();
		delay_ms(50);
	}
}

/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
    ESP_LOGI(TAG, "Starting LCD_Init...");
    LCD_GPIOInit();                                 
    ESP_LOGI(TAG, "LCD GPIO initialized, now resetting...");
    LCD_RESET();
    ESP_LOGI(TAG, "LCD reset completed, now configuring registers...");
    
    // 在每个寄存器配置段后添加小延迟
    LCD_WR_REG(0x36); 
    LCD_WR_DATA(0x00);

    LCD_WR_REG(0x3A); 
    LCD_WR_DATA(0x05);

    LCD_WR_REG(0xB2);
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x33);
    LCD_WR_DATA(0x33);

    LCD_WR_REG(0xB7); 
    LCD_WR_DATA(0x35);  

    LCD_WR_REG(0xBB);
    LCD_WR_DATA(0x17);

    LCD_WR_REG(0xC0);
    LCD_WR_DATA(0x2C);

    LCD_WR_REG(0xC2);
    LCD_WR_DATA(0x01);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA(0x12);   

    LCD_WR_REG(0xC4);
    LCD_WR_DATA(0x20);  

    LCD_WR_REG(0xC6); 
    LCD_WR_DATA(0x0F);    

    LCD_WR_REG(0xD0); 
    LCD_WR_DATA(0xA4);
    LCD_WR_DATA(0xA1);

    LCD_WR_REG(0xE0);
    LCD_WR_DATA(0xD0);
    LCD_WR_DATA(0x04);
    LCD_WR_DATA(0x0D);
    LCD_WR_DATA(0x11);
    LCD_WR_DATA(0x13);
    LCD_WR_DATA(0x2B);
    LCD_WR_DATA(0x3F);
    LCD_WR_DATA(0x54);
    LCD_WR_DATA(0x4C);
    LCD_WR_DATA(0x18);
    LCD_WR_DATA(0x0D);
    LCD_WR_DATA(0x0B);
    LCD_WR_DATA(0x1F);
    LCD_WR_DATA(0x23);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA(0xD0);
    LCD_WR_DATA(0x04);
    LCD_WR_DATA(0x0C);
    LCD_WR_DATA(0x11);
    LCD_WR_DATA(0x13);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x3F);
    LCD_WR_DATA(0x44);
    LCD_WR_DATA(0x51);
    LCD_WR_DATA(0x2F);
    LCD_WR_DATA(0x1F);
    LCD_WR_DATA(0x1F);
    LCD_WR_DATA(0x20);
    LCD_WR_DATA(0x23);

    LCD_WR_REG(0x21); 

    LCD_WR_REG(0x11); 
    delay_ms(120); 

    LCD_WR_REG(0x29); 
    
    // 设置颜色反转，提高对比度
    LCD_WR_REG(0x3A); 
    LCD_WR_DATA(0x05);  // 16-bit/pixel

    LCD_set_direction(USE_HORIZONTAL);
    
    // 确保背光完全开启
    LCD_BLK_SET();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "LCD backlight turned on");
    
    // LCD_Clear(WHITE);
    // ESP_LOGI(TAG, "LCD cleared with white color");
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(u16 xStar, u16 yStar, u16 xEnd, u16 yEnd)
{
    LCD_WR_REG(0x2A); // 列地址设置
    LCD_WR_DATA(xStar >> 8);
    LCD_WR_DATA(xStar & 0xFF);
    LCD_WR_DATA(xEnd >> 8);
    LCD_WR_DATA(xEnd & 0xFF);

    LCD_WR_REG(0x2B); // 行地址设置
    LCD_WR_DATA(yStar >> 8);
    LCD_WR_DATA(yStar & 0xFF);
    LCD_WR_DATA(yEnd >> 8);
    LCD_WR_DATA(yEnd & 0xFF);

    LCD_WriteRAM_Prepare(); // 0x2C
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_set_direction(u8 lcd_direction)
{
    lcddev.setxcmd = 0x2A;
    lcddev.setycmd = 0x2B;
    lcddev.wramcmd = 0x2C;
    lcddev.dir = lcd_direction % 4;
    
    switch(lcddev.dir) {
        case 0:  // 0度方向（默认）
            lcddev.width = DISPLAY_WIDTH;
            lcddev.height = DISPLAY_HEIGHT;
            lcddev.xoffset = 0;
            lcddev.yoffset = DISPLAY_OFFSET_Y;
            LCD_WriteReg(0x36, 0x00); // BGR=1, MY=0, MX=0, MV=0
            break;
        case 1:  // 90度方向
            lcddev.width = DISPLAY_HEIGHT;
            lcddev.height = DISPLAY_WIDTH;
            lcddev.xoffset = 0;
            lcddev.yoffset = DISPLAY_OFFSET_Y;
            LCD_WriteReg(0x36, (1<<6)|(1<<5)); // BGR=1, MY=1, MX=0, MV=1
            break;
        case 2:  // 180度方向
            lcddev.width = DISPLAY_WIDTH;
            lcddev.height = DISPLAY_HEIGHT;
            lcddev.xoffset = 0;
            lcddev.yoffset = DISPLAY_OFFSET_Y;
            LCD_WriteReg(0x36, (1<<6)|(1<<7)); // BGR=1, MY=0, MX=1, MV=0
            break;
        case 3:  // 270度方向
            lcddev.width = DISPLAY_HEIGHT;
            lcddev.height = DISPLAY_WIDTH;
            lcddev.xoffset = 0;
            lcddev.yoffset = DISPLAY_OFFSET_Y;
            LCD_WriteReg(0x36, (1<<7)|(1<<5)); // BGR=1, MY=1, MX=1, MV=1
            break;
        default:
            break;
    }
    ESP_LOGI(TAG, "LCD_set_direction: direction=%d, width=%d, height=%d, 0x36=0x%02X", 
             lcddev.dir, lcddev.width, lcddev.height, lcddev.dir == 0 ? 0x00 : 
             lcddev.dir == 1 ? 0x60 : lcddev.dir == 2 ? 0xC0 : 0xA0);
}	 


void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{    
  u16 width=ex-sx+1;    
  u16 height=ey-sy+1;   
  LCD_SetWindows(sx,sy,ex,ey);
  
  // 使用简单的for循环方式，避免DMA问题
  uint32_t total_pixels = width * height;
  for (uint32_t i = 0; i < total_pixels; i++) {
    Lcd_WriteData_16Bit(color);
  }
}

#define PI_BUF_SIZE 256
void showImage(int32_t x, int32_t y, int32_t w, int32_t h, const unsigned char  *data){
  int32_t dx = 0;
  int32_t dy = 0;
  int32_t dw = w;
  int32_t dh = h*2;
 
  if (x < 0) { dw += x; dx = -x; x = 0; }
  if (y < 0) { dh += y; dy = -y; y = 0; }
 
  if (dw < 1 || dh < 1) return;
  data += dx + dy * w;
 
  uint16_t  buffer[PI_BUF_SIZE];
  uint16_t* pix_buffer = buffer;
  uint16_t  high,low;
 
  // Work out the number whole buffers to send
  uint16_t nb = (dw * dh) / (2 * PI_BUF_SIZE);
 
  // Fill and send "nb" buffers to TFT
  for (int32_t i = 0; i < nb; i++) {
    for (int32_t j = 0; j < PI_BUF_SIZE; j++) {
      high = data[(i * 2 * PI_BUF_SIZE) + 2 * j + 1];
      low = data[(i * 2 * PI_BUF_SIZE) + 2 * j ];
      pix_buffer[j] = (high<<8)+low;
    }
    // 这里可以添加批量像素传输的代码
  }
 
  // Work out number of pixels not yet sent
  uint16_t np = (dw * dh) % (2 * PI_BUF_SIZE);
 
  // Send any partial buffer left over
  if (np) {
    for (int32_t i = 0; i < np; i++)
    {
      high = data[(nb * 2 * PI_BUF_SIZE) + 2 * i + 1];
      low = data[(nb * 2 * PI_BUF_SIZE) + 2 * i ];
      pix_buffer[i] = (high<<8)+low;
    }
    // 这里可以添加批量像素传输的代码
  }
}

void LCD_Fill_hv(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{    
  u16 i,j;      
  u16 width=ex-sx+1;    
  u16 height=ey-sy+1;   
  LCD_SetWindows(sx,sy,ex,ey);
  
  // 使用SPI批量传输优化
  char txbuf[3];
  txbuf[0] = 0;
  txbuf[1] = color >> 1;
  txbuf[2] = color << 15;
  
  spi_transaction_t t = {};
  t.length = 24; // 3字节 * 8位
  t.tx_buffer = txbuf;
  t.rx_buffer = NULL;
  
  for(i=0;i<height;i++)
  {
    for(j=0;j<width;j++){
      LCD_CS_CLR(); 
      esp_err_t ret = spi_device_transmit(spi_handle, &t);
      if (ret != ESP_OK) {
          ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
      }
      LCD_CS_SET(); 
    }
  }
  LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}