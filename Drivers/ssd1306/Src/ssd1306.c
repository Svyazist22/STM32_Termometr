#include "stm32f10x.h"
#include "ssd1306.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>  

 void spiTransmit(uint8_t data)
{
	CS_RES;	
	SPI1->DR = data;
	while((SPI1->SR & SPI_SR_BSY)){};
	CS_SET;
}

// Send a byte to the command register
void ssd1306_WriteCommand(uint8_t byte) {
  COMMAND;
	spiTransmit(byte);
	DATA;
}
static SSD1306_t SSD1306;

// Initialize the oled screen
void ssd1306_Init(void) {
	uint16_t i;
	GPIOA->CRL|= GPIO_CRL_MODE2 |GPIO_CRL_MODE1 | GPIO_CRL_MODE3;
	GPIOA->CRL&= ~(GPIO_CRL_CNF1 | GPIO_CRL_CNF2 | GPIO_CRL_CNF3);
	RESET_RES;
	
	for(i=0;i<BUFFER_SIZE;i++)
	{
		SSD1306_Buffer[i]=0;
	}
	RESET_SET;
	CS_SET;
	ssd1306_WriteCommand(0xAE); //display off
	ssd1306_WriteCommand(0xD5); //Set Memory Addressing Mode
	ssd1306_WriteCommand(0x80); //00,Horizontal Addressing Mode;01,Vertical
	ssd1306_WriteCommand(0xA8); //Set Page Start Address for Page Addressing
	ssd1306_WriteCommand(0x3F); //Set COM Output Scan Direction
	ssd1306_WriteCommand(0xD3); //set low column address
	ssd1306_WriteCommand(0x00); //set high column address
	ssd1306_WriteCommand(0x40); //set start line address
	ssd1306_WriteCommand(0x8D); //set contrast control register
	ssd1306_WriteCommand(0x14);
	ssd1306_WriteCommand(0x20); //set segment re-map 0 to 127
	ssd1306_WriteCommand(0x00); //set normal display
	ssd1306_WriteCommand(0xA1); //set multiplex ratio(1 to 64)
	ssd1306_WriteCommand(0xC8); //
	ssd1306_WriteCommand(0xDA); //0xa4,Output follows RAM
	ssd1306_WriteCommand(0x12); //set display offset
	ssd1306_WriteCommand(0x81); //not offset
	ssd1306_WriteCommand(0x8F); //set display clock divide ratio/oscillator frequency
	ssd1306_WriteCommand(0xD9); //set divide ratio
	ssd1306_WriteCommand(0xF1); //set pre-charge period
	ssd1306_WriteCommand(0xDB); 
	ssd1306_WriteCommand(0x40); //set com pins hardware configuration
	ssd1306_WriteCommand(0xA4);
	ssd1306_WriteCommand(0xA6); //set vcomh
	ssd1306_WriteCommand(0xAF); //0x20,0.77xVcc

	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	SSD1306.Initialized = 1;
}


void ssd1306_Fill(COLOR color) 
{
	for(uint16_t i=0;i<SSD1306_HEIGHT*SSD1306_WIDTH;i++)
	{
		if(color==WHITE)
			SSD1306_Buffer[i]=0xFF;
		else if(color==BLACK)
			SSD1306_Buffer[i]=0;
	}
}


void ssd1306_DrawPixel(uint8_t x, uint8_t y, COLOR color) 
{
  if(x<SSD1306_WIDTH && y <SSD1306_HEIGHT && x>=0 && y>=0)
	{
		if(color==WHITE)
		{
			SSD1306_Buffer[x+(y/8)*SSD1306_WIDTH]|=(1<<(y%8));
		}
		else if(color==BLACK)
		{
			SSD1306_Buffer[x+(y/8)*SSD1306_WIDTH]&=~(1<<(y%8));
		}
	}
}


char ssd1306_WriteChar(char ch, FontDef Font, COLOR color) 
{
    uint32_t i, b, j;
    
    if (ch < 32 || ch > 126) 
        return 0;
    
    if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
    {
        return 0; 
    }
    
    for(i = 0; i < Font.FontHeight; i++) 
		{
			b = Font.data[(ch - 32) * Font.FontHeight + i];
      for(j = 0; j < Font.FontWidth; j++) 
			{
				if((b << j) & 0x8000)  
				{
					ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (COLOR) color);
        } 
				else 
				{
        ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (COLOR)!color);
        }
      }
    }
    SSD1306.CurrentX += Font.FontWidth;
    
    return ch;
}


char ssd1306_WriteString(char* str, FontDef Font, COLOR color) 
{
	while (*str) 
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str) 
		{
			return *str; 
    }    
		str++;
  }
    return *str; 
}

// Position the cursor
void ssd1306_SetCursor(uint8_t x, uint8_t y) 
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

// Draw line by Bresenhem's algorithm
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, COLOR color) {
  int32_t deltaX = abs(x2 - x1);
  int32_t deltaY = abs(y2 - y1);
  int32_t signX = ((x1 < x2) ? 1 : -1);
  int32_t signY = ((y1 < y2) ? 1 : -1);
  int32_t error = deltaX - deltaY;
  int32_t error2;
    
  ssd1306_DrawPixel(x2, y2, color);
	
	while((x1 != x2) || (y1 != y2))
	{
		ssd1306_DrawPixel(x1, y1, color);
		error2 = error * 2;
		if(error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}
		else
		{
		/*nothing to do*/
		}
				
		if(error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
		else
		{
		/*nothing to do*/
		}
  }
  return;
}


void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, COLOR color) 
{
  uint16_t i;
  if(par_vertex != 0)
	{
    for(i = 1; i < par_size; i++)
		{
			ssd1306_Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
		}
  }
  else
  {
    /*nothing to do*/
  }
  return;
}

/*Convert Degrees to Radians*/
static float ssd1306_DegToRad(float par_deg) {
    return par_deg * 3.14 / 180.0;
}
/*Normalize degree to [0;360]*/
static uint16_t ssd1306_NormalizeTo0_360(uint16_t par_deg) {
  uint16_t loc_angle;
  if(par_deg <= 360)
  {
    loc_angle = par_deg;
  }
  else
  {
    loc_angle = par_deg % 360;
    loc_angle = ((par_deg != 0)?par_deg:360);
  }
  return loc_angle;
}


void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, COLOR color) {
    #define CIRCLE_APPROXIMATION_SEGMENTS 36
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1,xp2;
    uint8_t yp1,yp2;
    uint32_t count = 0;
    uint32_t loc_sweep = 0;
    float rad;
    
    loc_sweep = ssd1306_NormalizeTo0_360(sweep);
    
    count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;
    while(count < approx_segments)
    {
			rad = ssd1306_DegToRad(count*approx_degree);
			xp1 = x + (int8_t)(sin(rad)*radius);
			yp1 = y + (int8_t)(cos(rad)*radius);    
			count++;
			if(count != approx_segments)
			{
					rad = ssd1306_DegToRad(count*approx_degree);
			}
			else
			{            
					rad = ssd1306_DegToRad(loc_sweep);
			}
			xp2 = x + (int8_t)(sin(rad)*radius);
			yp2 = y + (int8_t)(cos(rad)*radius);    
			ssd1306_Line(xp1,yp1,xp2,yp2,color);
    }
    return;
}

//Draw circle by Bresenhem's algorithm
void ssd1306_DrawCircle(uint8_t par_x,uint8_t par_y,uint8_t par_r,COLOR par_color) {
  int32_t x = -par_r;
  int32_t y = 0;
  int32_t err = 2 - 2 * par_r;
  int32_t e2;

  if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT) 
	{
    return;
  }

   do {
      ssd1306_DrawPixel(par_x - x, par_y + y, par_color);
      ssd1306_DrawPixel(par_x + x, par_y + y, par_color);
      ssd1306_DrawPixel(par_x + x, par_y - y, par_color);
      ssd1306_DrawPixel(par_x - x, par_y - y, par_color);
        e2 = err;
        if (e2 <= y) {
            y++;
            err = err + (y * 2 + 1);
            if(-x == y && e2 <= x) {
              e2 = 0;
            }
            else
            {
              /*nothing to do*/
            }
        }
        else
        {
          /*nothing to do*/
        }
        if(e2 > x) {
          x++;
          err = err + (x * 2 + 1);
        }
        else
        {
          /*nothing to do*/
        }
    } while(x <= 0);
    return;
}

//Draw rectangle
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, COLOR color) {
  ssd1306_Line(x1,y1,x2,y1,color);
  ssd1306_Line(x2,y1,x2,y2,color);
  ssd1306_Line(x2,y2,x1,y2,color);
  ssd1306_Line(x1,y2,x1,y1,color);

  return;
}

void ssd1306_SetContrast(const uint8_t value) {
    const uint8_t kSetContrastControlRegister = 0x81;
    ssd1306_WriteCommand(kSetContrastControlRegister);
    ssd1306_WriteCommand(value);
}

void ssd1306_SetDisplayOn(const uint8_t on) {
    uint8_t value;
    if (on) {
        value = 0xAF;   // Display on
        SSD1306.DisplayOn = 1;
    } else {
        value = 0xAE;   // Display off
        SSD1306.DisplayOn = 0;
    }
    ssd1306_WriteCommand(value);
}

uint8_t ssd1306_GetDisplayOn() {
    return SSD1306.DisplayOn;
}

void ssd1306RunDisplayUPD()
{
	DATA;
	DMA1_Channel3->CCR&=~(DMA_CCR1_EN);
	DMA1_Channel3->CPAR=(uint32_t)(&SPI1->DR);
	DMA1_Channel3->CMAR=(uint32_t)&SSD1306_Buffer;
	DMA1_Channel3->CNDTR=sizeof(SSD1306_Buffer);
	DMA1->IFCR&=~(DMA_IFCR_CGIF3);
	CS_RES;
	DMA1_Channel3->CCR|=DMA_CCR1_CIRC;
	DMA1_Channel3->CCR|=DMA_CCR1_EN;
}

void ssd1306StopDispayUPD()
{
	CS_SET;
	DMA1_Channel3->CCR&=~(DMA_CCR1_EN);
	DMA1_Channel3->CCR&=~DMA_CCR1_CIRC;
}


