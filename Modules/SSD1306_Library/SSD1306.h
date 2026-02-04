/*
 * SSD1306.hpp
 *
 *  Created on: Oct 29, 2023
 *      Author: KiteBuilder
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include <stm32f4xx_hal.h>

#define SSD1306_I2C_ADDR        (0x3C << 1)
#define SSD1306_HEIGHT          64
#define SSD1306_WIDTH           128
#define SSD1306_BUFFER_SIZE     SSD1306_WIDTH * SSD1306_HEIGHT / 8

//SSD1306 command set 
#define SSD1306_SET_LOWER_COLUMN            0x00
#define SSD1306_SET_HIGHER_COLUMN           0x10
#define SSD1306_SET_MEM_ADDR_MODE           0x20
#define SSD1306_SET_START_LINE              0x40
#define SSD1306_SET_CONTRAST                0x81
#define SSD1306_SET_CHARGE_PUMP             0x8D
#define SSD1306_SET_SEGMENT_REMAP_0         0xA0
#define SSD1306_SET_SEGMENT_REMAP_1         0xA1
#define SSD1306_ENTIRE_DISPLAY_ON           0xA4
#define SSD1306_SET_NORMAL_COLOR            0xA6
#define SSD1306_SET_INVERSE_COLOR           0xA7
#define SSD1306_SET_MUX_RATIO               0xA8
#define SSD1306_TURN_OFF_PANEL              0xAE
#define SSD1306_TURN_ON_PANEL               0xAF
#define SSD1306_SET_PAGE_START_ADDR         0xB0
#define SSD1306_SET_COM_OUTPUT              0xC0
#define SSD1306_SCAN_DIRECTION              0xC8
#define SSD1306_SET_DISPLAY_OFFSET          0xD3
#define SSD1306_SET_DISPLAY_CLOCK           0xD5
#define SSD1306_SET_VCOMH                   0xDB
#define SSD1306_SET_PRE_CHARGE_PERIOD       0xD9
#define SSD1306_SET_COM_PINS_CONFIG         0xDA

// Enumeration for screen colors
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

typedef struct {
    const uint8_t width;    //font width in pixels
    const uint8_t height;   //font height in pixels
    const uint16_t *data;   //pointer to the font array
    const uint8_t ascii_start_num; //first declared ascii symbol in the table
    const uint8_t ascii_end_num;   //last declared ascii symbol in the table
} SSD1306_FontDef;

//******************************************************************************
//
//******************************************************************************
class SSD1306_Interface {

    uint16_t current_x;
    uint16_t current_y;
    uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

    virtual void ssd1306_WriteCommand(uint8_t byte) = 0;
    virtual void ssd1306_WriteData(uint8_t* buffer, size_t buff_size) = 0;
    virtual void ssd1306_WriteStream(uint8_t* buffer, size_t buff_size) = 0;

    virtual void ssd1306_Reset(void) = 0;

    uint32_t calc_sqrt(uint32_t val);

    uint8_t tx_buf[SSD1306_WIDTH + 7];

public:
    void ssd1306_Init(void);
    void ssd1306_UpdateScreen(void);
    void ssd1306_UpdateScreenDMA(void);
    void ssd1306_Fill(SSD1306_COLOR color);
    void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
    void ssd1306_SetContrast(const uint8_t value);
    void ssd1306_Mirror(bool mirror);
    void ssd1306_InverseColor(bool inverse);
    void ssd1306_SetCursor(uint8_t x, uint8_t y);
    int ssd1306_WriteChar(char ch, SSD1306_FontDef font, SSD1306_COLOR color);
    char ssd1306_WriteString(char* str, SSD1306_FontDef font, SSD1306_COLOR color);

    void ssd1306_DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius, SSD1306_COLOR color);
    void ssd1306_DrawFillCircle(uint8_t x0, uint8_t y0, uint8_t radius, SSD1306_COLOR color);
    void ssd1306_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR color);
    void ssd1306_DrawRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR color);
    void ssd1306_DrawFillRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR color);

    SSD1306_Interface() = default;
    ~SSD1306_Interface() = default;
};

//******************************************************************************
//
//******************************************************************************
class SSD1306_I2C_Display : public SSD1306_Interface{

    I2C_HandleTypeDef *p_hi2c;

    static SSD1306_I2C_Display *p_instance;

    void ssd1306_WriteCommand(uint8_t byte);
    void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);
    void ssd1306_WriteStream(uint8_t* buffer, size_t buff_size);
    void ssd1306_Reset(void);

    SSD1306_I2C_Display(I2C_HandleTypeDef *p_handler)
    {
        p_hi2c = p_handler;
    }

public:
    SSD1306_I2C_Display(const SSD1306_I2C_Display &obj) = delete;

    static SSD1306_I2C_Display* getInstance()
    {
        return p_instance;
    }

    ~SSD1306_I2C_Display() = default;
};

#if 0

typedef struct{
    GPIO_TypeDef* gpio;
    uint16_t pin;
} SSD1306_Port;

//******************************************************************************
//
//******************************************************************************
class SSD1306_SPI_Display : public SSD1306_Interface{

    SPI_HandleTypeDef *p_spi;
    SSD1306_Port *p_cs, *p_dc, *p_rst;

    static SSD1306_SPI_Display *p_instance;

    void ssd1306_WriteCommand(uint8_t byte);
    void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);
    void ssd1306_Reset(void);

    SSD1306_SPI_Display(SPI_HandleTypeDef *p_handler, SSD1306_Port *p_port_cs, SSD1306_Port *p_port_dc, SSD1306_Port *p_port_rst)
    {
        p_spi = p_handler;
        p_cs = p_port_cs;
        p_dc = p_port_dc;
        p_rst = p_port_rst;
    }

public:
    SSD1306_SPI_Display(const SSD1306_SPI_Display &obj) = delete;

    static SSD1306_SPI_Display* getInstance()
    {
        return p_instance;
    }

    ~SSD1306_SPI_Display() = default;
};
#endif

#endif /* SSD1306_H_ */
