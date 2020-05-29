/*
 * ST7789.h
 *
 */

#ifndef __ST7789_H__
#define __ST7789_H__

#include "fonts.h"
#include "ST7789_cfg.h"
#include <stdbool.h>



#define ST7789_MADCTL_MY 0x80
#define ST7789_MADCTL_MX 0x40
#define ST7789_MADCTL_MV 0x20
#define ST7789_MADCTL_RGB 0x00
#define ST7789_MADCTL_BGR 0x08


#define ST7789_TFTWIDTH 	240
#define ST7789_TFTHEIGHT 	240

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0


#define ST7789_NOP 0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID 0x04
#define ST7789_RDDST 0x09

#define ST7789_SLPIN 0x10
#define ST7789_SLPOUT 0x11
#define ST7789_PTLON 0x12
#define ST7789_NORON 0x13

#define ST7789_INVOFF 0x20
#define ST7789_INVON 0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON 0x29
#define ST7789_CASET 0x2A
#define ST7789_RASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_RAMRD 0x2E

#define ST7789_PTLAR 0x30
#define ST7789_COLMOD 0x3A
#define ST7789_MADCTL 0x36

#define ST7789_FRMCTR1 0xB1
#define ST7789_FRMCTR2 0xB2
#define ST7789_FRMCTR3 0xB3
#define ST7789_INVCTR 0xB4
#define ST7789_DISSET5 0xB6

#define ST7789_PWCTR1 0xC0
#define ST7789_PWCTR2 0xC1
#define ST7789_PWCTR3 0xC2
#define ST7789_PWCTR4 0xC3
#define ST7789_PWCTR5 0xC4
#define ST7789_VMCTR1 0xC5

#define ST7789_RDID1 0xDA
#define ST7789_RDID2 0xDB
#define ST7789_RDID3 0xDC
#define ST7789_RDID4 0xDD

#define ST7789_PWCTR6 0xFC

#define ST7789_GMCTRP1 0xE0
#define ST7789_GMCTRN1 0xE1

// Color definitions
#define ST7789_BLACK 0x0000
#define ST7789_BLUE 0x001F
#define ST7789_RED 0xF800
#define ST7789_GREEN 0x07E0
#define ST7789_CYAN 0x07FF
#define ST7789_MAGENTA 0xF81F
#define ST7789_YELLOW 0xFFE0
#define ST7789_WHITE 0xFFFF

void ST7789_Backlight_On(void);
void ST7789_Backlight_Off(void);
void ST7789_Init(void);
uint16_t ST7789_Color565(uint8_t r, uint8_t g, uint8_t b);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_FillScreen(uint16_t color);
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h,  uint16_t* data);
void ST7789_InvertColors(bool invert);
void ST7789_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7789_DrawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void ST7789_FillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7789_FillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void ST7789_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
void ST7789_FillEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color);
void ST7789_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7789_DrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7789_FillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7789_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7789_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7789_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7789_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void ST7789_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void ST7789_SetRotation(uint8_t m);
uint8_t ST7789_GetRotation(void);
int16_t ST7789_GetHeight(void);
int16_t ST7789_GetWidth(void);
void ST7789_DrawChar(char ch, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor);
void ST7789_DrawText(const char* str, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor);

#endif // __ST7789_H__
