/*
 * ST7789_cfg.h
 *
 */

#ifndef ST7789_CFG_H_
#define ST7789_CFG_H_

#include "main.h"

#define ST7789_SPI_PORT SPI1 //hspi1, hspi2, hspi3...
#define USE_SPI_DMA //if used DMA for SPI bus

#define USING_240X240
//Port and pin connected signal 'RES' (reset) ST7789 display
#ifndef ST7789_RES_Pin
#define ST7789_RES_Pin GPIO_PIN_12
#endif
#ifndef ST7789_RES_GPIO_Port
#define ST7789_RES_GPIO_Port GPIOB
#endif
//Port and pin connected signal 'DC' (data or command) ST7789 display
#ifndef ST7789_DC_Pin
#define ST7789_DC_Pin GPIO_PIN_13
#endif
#ifndef ST7789_DC_GPIO_Port
#define ST7789_DC_GPIO_Port GPIOB
#endif
//Port and pin connected signal 'CS' (chip select) ST7789 display
#ifndef ST7789_CS_Pin
#define ST7789_CS_Pin GPIO_PIN_14
#endif
#ifndef ST7789_CS_GPIO_Port
#define ST7789_CS_GPIO_Port GPIOB
#endif
//Port and pin connected signal 'BL' (back light) ST7789 display
#ifndef ST7789_BL_Pin
#define ST7789_BL_Pin GPIO_PIN_15
#endif
#ifndef ST7789_BL_GPIO_Port
#define ST7789_BL_GPIO_Port GPIOB
#endif

#endif /* ST7789_CFG_H_ */
