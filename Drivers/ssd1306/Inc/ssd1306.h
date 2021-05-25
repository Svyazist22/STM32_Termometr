
static uint8_t SSD1306_Buffer[1024];
#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stddef.h>
#include "main.h"


#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"




#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif

#ifndef BUFFER_SIZE
#define BUFFER_SIZE   SSD1306_WIDTH * SSD1306_HEIGHT / 8
#endif


#define CS_SET GPIOA->BSRR|=GPIO_BSRR_BS2
#define CS_RES GPIOA->BSRR|=GPIO_BSRR_BR2
#define RESET_SET GPIOA->BSRR|=GPIO_BSRR_BS1
#define RESET_RES GPIOA->BSRR|=GPIO_BSRR_BR1
#define DATA GPIOA->BSRR|=GPIO_BSRR_BS3
#define COMMAND GPIOA->BSRR|=GPIO_BSRR_BR3

typedef enum COLOR
	{
	BLACK,
	WHITE
	}COLOR;


typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
    uint8_t DisplayOn;
} SSD1306_t;
typedef struct {
    uint8_t x;
    uint8_t y;
} SSD1306_VERTEX;

// Procedure definitions
void ssd1306_Init(void);
void ssd1306_Fill(COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, COLOR color);
char ssd1306_WriteString(char* str, FontDef Font, COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, COLOR color);
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, COLOR color);
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, COLOR color);
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, COLOR color);
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, COLOR color);
void spiTransmit(uint8_t data);
void ssd1306RunDisplayUPD();
void ssd1306StopDisplayUPD();
void ssd1306_WriteCommand(uint8_t byte);
void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);
/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void ssd1306_SetContrast(const uint8_t value);
/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void ssd1306_SetDisplayOn(const uint8_t on);
/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
//uint8_t ssd1306_GetDisplayOn();

// Low-level procedures
void ssd1306_Reset(void);





#endif // __SSD1306_H__
