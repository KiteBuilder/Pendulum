/*
 * SSD1306.cpp
 *
 *  Created on: Oct 29, 2023
 *      Author: KiteBuilder
 */

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <SSD1306.h>

using namespace std;

//******************************************************************************
//  SSD1306_Interface methods
//******************************************************************************

/**
  * @brief
  * @param None
  * @retval None
  */
void SSD1306_Interface::ssd1306_Init(void)
{
    // Reset OLED
    ssd1306_Reset();

    // Wait while screen is booting
    HAL_Delay(100);

    // Init OLED
    ssd1306_WriteCommand(SSD1306_TURN_OFF_PANEL); //turn off SSD1306 panel

    ssd1306_WriteCommand(SSD1306_SET_MEM_ADDR_MODE); //Set Memory Addressing Mode
    ssd1306_WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                // 10b,Page Addressing Mode (RESET); 11b,Invalid

    ssd1306_WriteCommand(SSD1306_SET_PAGE_START_ADDR); //Set Page Start Address for Page Addressing Mode,0-7

    ssd1306_WriteCommand(SSD1306_SET_LOWER_COLUMN); //set low column address

    ssd1306_WriteCommand(SSD1306_SET_HIGHER_COLUMN); //set high column address

    ssd1306_WriteCommand(SSD1306_SET_START_LINE); //set start line address

    ssd1306_SetContrast(0xFF);

    ssd1306_Mirror(false);      //default vertical and horizontal orientation

    ssd1306_InverseColor(false); //set standard color

    ssd1306_WriteCommand(SSD1306_SET_MUX_RATIO); //set multiplex ratio(1 to 64)
    ssd1306_WriteCommand(0x3F); //

    ssd1306_WriteCommand(SSD1306_ENTIRE_DISPLAY_ON); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    ssd1306_WriteCommand(SSD1306_SET_DISPLAY_OFFSET); //set display offset - CHECK
    ssd1306_WriteCommand(0x00); //no offset

    ssd1306_WriteCommand(SSD1306_SET_DISPLAY_CLOCK); //set display clock divide ratio/oscillator frequency
    ssd1306_WriteCommand(0xF0); //set divide ratio

    ssd1306_WriteCommand(SSD1306_SET_PRE_CHARGE_PERIOD); //set pre-charge period
    ssd1306_WriteCommand(0x22); //

    ssd1306_WriteCommand(SSD1306_SET_COM_PINS_CONFIG); //set com pins hardware configuration
    ssd1306_WriteCommand(0x12);

    ssd1306_WriteCommand(SSD1306_SET_VCOMH); //set vcomh
    ssd1306_WriteCommand(0x20); //0x20,0.77xVcc

    ssd1306_WriteCommand(SSD1306_SET_CHARGE_PUMP); //set DC-DC enable
    ssd1306_WriteCommand(0x14); //

    ssd1306_WriteCommand(SSD1306_TURN_ON_PANEL); //turn on SSD1306 panel

    // Clear screen
    ssd1306_Fill(Black);

    // Flush buffer to screen
    ssd1306_UpdateScreen();

    // Set default values for screen object
    current_x = 0;
    current_y = 0;
}

/**
  * @brief
  * Write data to each page of RAM. Number of pages
  * depends on the screen height:
  *
  * 32px   ==  4 pages
  * 64px   ==  8 pages
  * 128px  ==  16 pages
  *
  * @param None
  * @retval None
  */
void SSD1306_Interface::ssd1306_UpdateScreen(void)
{
    for (uint8_t i = 0; i < SSD1306_HEIGHT/8; i++)
    {
        ssd1306_WriteCommand(SSD1306_SET_PAGE_START_ADDR + i); // Set the current RAM page address.
        ssd1306_WriteCommand(SSD1306_SET_LOWER_COLUMN);
        ssd1306_WriteCommand(SSD1306_SET_HIGHER_COLUMN);

        ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}

/**
  * @brief
  * Write data to each page of RAM. Number of pages
  * depends on the screen height:
  *
  * 32px   ==  4 pages
  * 64px   ==  8 pages
  * 128px  ==  16 pages
  *
  * @param None
  * @retval None
  */
void SSD1306_Interface::ssd1306_UpdateScreenDMA(void)
{
    static uint8_t i = 0;

    if (i == SSD1306_HEIGHT/8)
    {
        i = 0;
        return;
    }

    tx_buf[0] = 0x80; // Co bit set, Command transfer selected
    tx_buf[1] = SSD1306_SET_PAGE_START_ADDR + i;
    tx_buf[2] = 0x80; //Co bit set, Command transfer selected
    tx_buf[3] = SSD1306_SET_LOWER_COLUMN;
    tx_buf[4] = 0x80; //Co bit set,  Command transfer selected
    tx_buf[5] = SSD1306_SET_HIGHER_COLUMN;
    tx_buf[6] = 0x40; //Co bit reset (data flow), Data transfer selected

    std::memcpy((tx_buf + 7), &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);

    ssd1306_WriteStream(tx_buf, SSD1306_WIDTH + 7);

    i++;
}

/**
  * @brief Fill the whole screen with the given color
  * @param color: color of the pixel 1-white, 0-black
  * @retval None
  */
void SSD1306_Interface::ssd1306_Fill(SSD1306_COLOR color)
{
    for (uint32_t i = 0; i < sizeof(SSD1306_Buffer); i++)
    {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

/**
  * @brief Set or reset one monochrome pixel in a buffer
  * @param
  * x: x coordinate of the pixel
  * y: y coordinate of the pixel
  * color: color of the pixel 1-white, 0-black
  * @retval None
  */
void SSD1306_Interface::ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        // Don't write outside the buffer
        return;
    }

    // Draw in the right color
    if(color == White) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

/**
  * @brief Set display contrast
  * @param value: Contrast value from 0x00 to 0xFF
  * @retval None
  */
void SSD1306_Interface::ssd1306_SetContrast(const uint8_t value)
{
    ssd1306_WriteCommand(SSD1306_SET_CONTRAST);
    ssd1306_WriteCommand(value);
}

/**
  * @brief Mirror screen image
  * @param mirror: flip image 180grad
  * @retval None
  */
void SSD1306_Interface::ssd1306_Mirror(bool mirror)
{
    if (mirror)
    {
        ssd1306_WriteCommand(SSD1306_SET_COM_OUTPUT); // normal mode (RESET) Scan from COM0 to COM[N –1]
        ssd1306_WriteCommand(SSD1306_SET_SEGMENT_REMAP_0); // column address 0 is mapped to SEG0 (RESET)
    }
    else
    {
        ssd1306_WriteCommand(SSD1306_SCAN_DIRECTION); // remapped mode. Scan from COM[N-1] to COM0
        ssd1306_WriteCommand(SSD1306_SET_SEGMENT_REMAP_1); // column address 127 is mapped to SEG0
    }
}

/**
  * @brief Inverse display color
  * @param inverse: inverse color if true
  * @retval None
  */
void SSD1306_Interface::ssd1306_InverseColor(bool inverse)
{
    if (inverse)
    {
        ssd1306_WriteCommand(SSD1306_SET_INVERSE_COLOR); //--set inverse color
    }
    else
    {
        ssd1306_WriteCommand(SSD1306_SET_NORMAL_COLOR); //--set normal color
    }
}

/**
  * @brief Set current cursor position
  * @param x: current x coordinate
  *        y: current y coordinate
  * @retval None
  */
void SSD1306_Interface::ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    current_x =  (x < SSD1306_WIDTH) ? x : SSD1306_WIDTH - 1;
    current_y =  (y < SSD1306_HEIGHT) ? y : SSD1306_HEIGHT - 1;
}

/**
  * @brief Draw one char to the screen buffer
  * @param ch: ASCII char symbol
  *        font: currently used font
  *        color: currently used color
  * @retval Error = -1, Success = 0
  */
int SSD1306_Interface::ssd1306_WriteChar(char ch, SSD1306_FontDef font, SSD1306_COLOR color)
{
    uint16_t pix_row;

    // Check if character declared in the array
    if (ch < font.ascii_start_num || ch > font.ascii_end_num)
    {
        return -1;
    }

    // Check remaining space on current line
    if (SSD1306_WIDTH < (current_x + font.width) ||
        SSD1306_HEIGHT < (current_y + font.height) )
    {
        // Not enough space on current line
        return -1;
    }

    for (uint32_t i = 0; i < font.height; i++)
    {
        pix_row = font.data[(ch - font.ascii_start_num) * font.height + i];

        for (uint32_t j = 0; j < font.width; j++)
        {
            if ((pix_row << j) & 0x8000)
            {
                ssd1306_DrawPixel(current_x + j, current_y + i, color);
            }
            else
            {
                ssd1306_DrawPixel(current_x + j, current_y + i, (SSD1306_COLOR)!color);
            }
        }
    }

    current_x += font.width;

    return 0;
}

/**
  * @brief Write string to the screen buffer
  * @param str: ASCII char string buffer
  *        font: currently used font
  *        color: currently used color
  * @retval last not transferred character in the case of the error
  */
char SSD1306_Interface::ssd1306_WriteString(char* str, SSD1306_FontDef font, SSD1306_COLOR color)
{
    while (*str != '\0')
    {
        if (ssd1306_WriteChar(*str, font, color) != 0)
        {
            // Char that wasn't written
            return *str;
        }
        str++;
    }

    // Everything ok
    return *str;
}

/**
  * @brief Draw circle - Bresenham’s circle drawing algorithm
  * @param (x0,y0) - circle center coordinates
  *         radius - circle radius
  *         color: currently used color
  * @retval None
  */
void SSD1306_Interface::ssd1306_DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius, SSD1306_COLOR color)
{
    if (x0 >= SSD1306_WIDTH || y0 >= SSD1306_HEIGHT)
    {
        return;
    }

    uint8_t x = 0;
    uint8_t y = radius;
    int32_t P = 3 - (2 * radius);

    do
    {
        ssd1306_DrawPixel(x0 + x, y0 + y, color);
        ssd1306_DrawPixel(x0 + y, y0 + x, color);

        ssd1306_DrawPixel(x0 - x, y0 + y, color);
        ssd1306_DrawPixel(x0 - y, y0 + x, color);

        ssd1306_DrawPixel(x0 + x, y0 - y, color);
        ssd1306_DrawPixel(x0 + y, y0 - x, color);

        ssd1306_DrawPixel(x0 - x, y0 - y, color);
        ssd1306_DrawPixel(x0 - y, y0 - x, color);

        if (P < 0)
        {
            ++x;
            P = P + 4*x + 6;
        }
        else
        {
            ++x;
            --y;
            P = P + 4*(x - y) + 10;
        }
    } while (x <= y);
}

/**
  * @brief Draw Filled circle - Bresenham’s circle drawing algorithm
  * @param (x0,y0) - circle center coordinates
  *         radius - circle radius
  *         color: currently used color
  * @retval None
  */
void SSD1306_Interface::ssd1306_DrawFillCircle(uint8_t x0, uint8_t y0, uint8_t radius, SSD1306_COLOR color)
{
    if (x0 >= SSD1306_WIDTH || y0 >= SSD1306_HEIGHT)
    {
        return;
    }

    uint8_t x = 0;
    uint8_t y = radius;
    int32_t P = 3 - (2 * radius);

    do
    {
        for (uint8_t x1 = (x0 - x); x1 <= (x0 + x); x1++)
        {
            for (uint8_t y1 = (y0 - y); y1 <= (y0 + y); y1++)
            {
                ssd1306_DrawPixel(x1, y1, color);
            }
        }

        for (uint8_t y1 = (y0 - x); y1 <= (y0 + x); y1++)
        {
            ssd1306_DrawPixel(x0 - y, y1, color);
        }

        for (uint8_t y1 = (y0 - x); y1 <= (y0 + x); y1++)
        {
            ssd1306_DrawPixel(x0 + y, y1, color);
        }

        if (P < 0)
        {
            ++x;
            P = P + 4*x + 6;
        }
        else
        {
            ++x;
            --y;
            P = P + 4*(x - y) + 10;
        }
    } while (x <= y);
}

/**
  * @brief Calculate the square root
  * @param val - value
  * @retval result
  */
uint32_t SSD1306_Interface::calc_sqrt(uint32_t val)
{
    const float accuracy = 0.1;

    // for 0 and 1, the square roots are themselves
    if (val < 2)
        return val;

    // considering the equation values
    float n = val;
    float k = (n + (val / n)) / 2;

    // difference should not exceed 0.01
    while ( (n - k) >= accuracy)
    {
        n = k;
        k = (n + (val / n)) / 2;
    }
    return (uint8_t)k;
}

/**
  * @brief Draw line
  * @param (x0,y0) - line start coordinates
  *        (x1,y1) - line end coordinates
  *        color: currently used color
  * @retval None
  */
void SSD1306_Interface::ssd1306_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR color)
{
    int16_t sign = 0;
    int16_t a = y0 - y1;
    int16_t b = x1 - x0;
    int32_t c = (x0 * y1) - (y0 * x1);

    if (b != 0)
    {
        if (abs(b) > abs(a))
        {
            sign = (x0 > x1) ? -1 : 1;
        }
        else
        {
            sign = (y0 > y1) ? -1 : 1;
        }
    }
    else if (a != 0)
    {
        sign = (y0 > y1) ? -1 : 1;
    }

    do
    {
        ssd1306_DrawPixel(x0, y0, color);

        if (b != 0)
        {
            if (abs(b) > abs(a))
            {
                x0 += sign;
                y0 = ((-a * x0) - c) / b;
            }
            else
            {
                y0 += sign;
                x0 = (y0*b + c)/-a;
            }
        }
        else if (a != 0)
        {
            y0 += sign;
        }

    } while ((x0 != x1) || (y0 != y1));

    ssd1306_DrawPixel(x0, y0, color);
}

/**
  * @brief Draw Rectangle
  * @param (x0,y0) - up left corner coordinates
  *        (x1,y1) - down right corner coordinates
  *        color: currently used color
  * @retval None
  */
void SSD1306_Interface::ssd1306_DrawRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR color)
{
    ssd1306_DrawLine(x0, y0, x0, y1, color);
    ssd1306_DrawLine(x1, y0, x1, y1, color);
    ssd1306_DrawLine(x0, y0, x1, y0, color);
    ssd1306_DrawLine(x0, y1, x1, y1, color);
}

/**
  * @brief Draw Filled Rectangle
  * @param (x0,y0) - up left corner coordinates
  *        (x1,y1) - down right corner coordinates
  *        color: currently used color
  * @retval None
  */
void SSD1306_Interface::ssd1306_DrawFillRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, SSD1306_COLOR color)
{
    uint8_t x_start = (x0 < x1) ? x0 : x1;
    uint8_t y_start = (y0 < y1) ? y0 : y1;
    uint8_t x_end = (x0 < x1) ? x1 : x0;
    uint8_t y_end = (y0 < y1) ? y1 : y0;

    for (uint8_t x = x_start; x <= x_end; x++)
    {
        for (uint8_t y = y_start; y <= y_end; y++)
        {
            ssd1306_DrawPixel(x, y, color);
        }
    }
}

//******************************************************************************
// SSD1306_Display I2C methods
//******************************************************************************

/**
  * @brief Write command
  * @param byte: byte sized command
  * @retval None
  */
void SSD1306_I2C_Display::ssd1306_WriteCommand(uint8_t byte)
{
    HAL_I2C_Mem_Write(p_hi2c, SSD1306_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

/**
  * @brief Write data sequence
  * @param buffer: pointer to data buffer
  *        buff_size: amount of bytes to transfer
  * @retval None
  */
void SSD1306_I2C_Display::ssd1306_WriteData(uint8_t* buffer, size_t buff_size)
{
    HAL_I2C_Mem_Write(p_hi2c, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}

/**
  * @brief Write data sequence as a stream of data 
  * @param buffer: pointer to data buffer
  *        buff_size: amount of bytes to transfer
  * @retval None
  */
void SSD1306_I2C_Display::ssd1306_WriteStream(uint8_t* buffer, size_t buff_size)
{
    HAL_I2C_Master_Transmit_DMA(p_hi2c, SSD1306_I2C_ADDR, buffer, buff_size);
}

/**
  * @brief Reset display
  * @param None
  * @retval None
  */
void SSD1306_I2C_Display::ssd1306_Reset(void)
{
    //Empty function for the I2C
}

#if 0

//******************************************************************************
// SSD1306_Display SPI methods
//******************************************************************************
/**
  * @brief Write command
  * @param byte: byte sized command
  * @retval None
  */
void SSD1306_SPI_Display::ssd1306_WriteCommand(uint8_t byte)
{
    HAL_GPIO_WritePin(p_cs->gpio, p_cs->pin, GPIO_PIN_RESET); //Select OLED
    HAL_GPIO_WritePin(p_dc->gpio, p_dc->pin, GPIO_PIN_RESET); //Command mode
    HAL_SPI_Transmit(p_spi, (uint8_t *) &byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(p_cs->gpio, p_cs->pin, GPIO_PIN_SET);   //Un-select OLED
}

/**
  * @brief Write data sequence
  * @param buffer: pointer to data buffer
  *        buff_size: amount of bytes to transfer
  * @retval None
  */
void SSD1306_SPI_Display::ssd1306_WriteData(uint8_t* buffer, size_t buff_size)
{
    HAL_GPIO_WritePin(p_cs->gpio, p_cs->pin, GPIO_PIN_RESET); //Select OLED
    HAL_GPIO_WritePin(p_dc->gpio, p_dc->pin, GPIO_PIN_SET);   //Data mode
    HAL_SPI_Transmit(p_spi, buffer, buff_size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(p_cs->gpio, p_cs->pin, GPIO_PIN_SET);   //Un-select OLED
}

/**
  * @brief Reset display
  * @param None
  * @retval None
  */
void SSD1306_SPI_Display::ssd1306_Reset(void)
{
    // CS = High (not selected)
    HAL_GPIO_WritePin(p_cs->gpio, p_cs->pin, GPIO_PIN_SET);

    // Reset the OLED
    HAL_GPIO_WritePin(p_rst->gpio, p_rst->pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(p_rst->gpio, p_rst->pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

#endif