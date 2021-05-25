#include <string.h>
#include "main.h"


#define SYSCLOCK 72000000U
static void Clock_Init();
static void TIM2_Init(void);
static void TIM3_Init(void);
static void USART_Init(void);
static void USART_Send(char *str);
static void I2C1_Init(void);
static void I2C2_Init(void);
static void I2C_Write(uint16_t addr,uint8_t *buf, uint16_t bytes_count);
static void I2C_Read(uint16_t addr, uint8_t *buf, uint16_t bytes_count);
static void SPI1_Init(void);
static void AHT10_Write(uint8_t *buf, uint16_t bytes_count);
static void AHT10_Read(uint8_t *buf, uint16_t bytes_count);
static void Delay(uint32_t time_delay);
static void EXTI_Iint();
static void EEPROM_Shift(uint16_t addr,uint8_t count_repeat,uint8_t size_array);
static void RCC_DeInit(void);

#define I2C_OWNADDRESS1_7BIT             0x00004000U
#define I2C_MODE_I2C                     0x00000000U
#define SLAVE_OWN_ADDRESS                       0xA0 //0x50<<1
#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01
#define SLAVE_OWN_ADDRESS2                      0x72
#define AHT10_ADRESS 														0x70 //0x38<<1

#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)

__IO uint32_t tmpreg;
__IO uint8_t tim2_count = 0;

uint8_t rd_value[64] = {0};

uint8_t AHT10_RX_Data[6];
uint32_t AHT10_ADC_Raw;
uint8_t AHT10_TmpHum_Cmd[3] = {0xAC, 0x33, 0x00};

float AHT10_Temperature;
float AHT10_Humidity;
uint16_t temperature;
uint16_t humidity;
uint8_t wr_data[4];
char str[128];	
uint8_t Timer = 0;
uint8_t r_w = 255;
uint16_t hour = 0;
uint16_t min_2 = 0;

int display = 0;

int main(void) 
{
	USART_Init();
	RCC_DeInit();
	Clock_Init();
	TIM2_Init();
	TIM3_Init();
	I2C1_Init();
	I2C2_Init();
	SPI1_Init();
	ssd1306_Init();
	EXTI_Iint();
	
	ssd1306RunDisplayUPD();
	ssd1306_Fill(BLACK);
	
	ssd1306_WriteString("Loading",Font_11x18,WHITE);
	TIM_EnableIT_UPDATE(TIM2);
	TIM_EnableCounter(TIM2);
	TIM_EnableIT_UPDATE(TIM3);
	TIM_EnableCounter(TIM3);
	
	uint16_t k = 0;
	while (k<20)
	{
		wr_data[k]=0;
		k++;
	}
	I2C_Write(0,wr_data,20);

	while(1)
	{
	
		if (Timer)
		{ 	
			if (r_w)
			{
				AHT10_Write(AHT10_TmpHum_Cmd,3);	
				r_w=0;
			}
			else
			{
				AHT10_Read(AHT10_RX_Data,6);
				r_w=255;
			}	
			
			wr_data[0] = temperature/10;
			wr_data[1] = temperature%10;
			wr_data[2] = humidity/10;
			wr_data[3] = humidity%10;

			sprintf(str,"t = +%i.%i",temperature/10,temperature%10);	
			ssd1306_SetCursor(0,0);
			ssd1306_WriteString(str,Font_11x18,WHITE);
			sprintf(str,"h = %i.%i%% ",humidity/10,humidity%10);
			ssd1306_SetCursor(0,35);
			ssd1306_WriteString(str,Font_11x18,WHITE);
			
			ssd1306_SetCursor(0,20);	
			if (temperature/10>26)
			{
				ssd1306_WriteString("HIGHT ",Font_7x10,WHITE);
			}
			else if(temperature/10<23)
			{
				ssd1306_WriteString("LOW",Font_7x10,WHITE);
			}
			else
			{
				ssd1306_WriteString("NORMAL",Font_7x10,WHITE);
			}
			
			ssd1306_SetCursor(0,54);	
			if (humidity/10>40)
			{
				ssd1306_WriteString("HIGHT ",Font_7x10,WHITE);
			}
			else if(humidity/10<30)
			{
				ssd1306_WriteString("LOW",Font_7x10,WHITE);
			}
			else
			{
				ssd1306_WriteString("NORMAL",Font_7x10,WHITE);
			}
			Timer = 0;	
		}
	}
}


static void USART_Init()
{
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_USART1EN);  //USART1 clock enable 
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_IOPAEN); //I/O port A clock enable 
	SET_BIT(USART1->CR1,USART_CR1_TE|USART_CR1_RE|USART_CR1_UE|USART_CR1_RXNEIE);
	WRITE_REG(USART1->BRR,0x1D52); // set Baud rate 9600
	MODIFY_REG(GPIOA->CRH,GPIO_CRH_MODE10|GPIO_CRH_CNF10_1|GPIO_CRH_CNF9_0, //(reg,reset,set)
												GPIO_CRH_MODE9_0|GPIO_CRH_CNF9_1|GPIO_CRH_CNF10_0); //10-Floating input (reset state) 9-Output mode, max speed 10 MHz.Alternate function output Push-pull  
	NVIC_EnableIRQ (USART1_IRQn);
}

static void USART_Send(char *str)
{
	int i=0;
	while(str[i])
	{
		while(!(USART1->SR & USART_SR_TC)){}
		USART1->DR = str[i++];
	}
}

void USART1_IRQHandler(void)
{
	char str2[]="";
	uint8_t data;
	if (USART1->SR & USART_SR_RXNE)
	{
		USART1->DR = (USART1->DR);
	}
}

static void I2C2_Init()
{
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_IOPBEN); //I/O port B clock enable 
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_I2C2EN); //I2C2 clock enable

	SET_BIT(GPIOB->CRH, GPIO_CRH_CNF11_1 | GPIO_CRH_CNF10_1 | GPIO_CRH_CNF11_0 | GPIO_CRH_CNF10_0 |\
                  GPIO_CRH_MODE11_1 | GPIO_CRH_MODE10_1 | GPIO_CRH_MODE11_0 | GPIO_CRH_MODE10_0); //Output mode, max speed 50 MHz.Alternate function output Open-drain
	
	tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
	
  CLEAR_BIT(I2C2->OAR2, I2C_OAR2_ENDUAL);
  CLEAR_BIT(I2C2->CR1, I2C_CR1_ENGC);
  CLEAR_BIT(I2C2->CR1, I2C_CR1_NOSTRETCH);
	
	CLEAR_BIT(I2C2->CR1, I2C_CR1_PE); //Peripheral disable
	
	MODIFY_REG(I2C2->CR2,I2C_CR2_FREQ,36); 
	MODIFY_REG(I2C2->TRISE, I2C_TRISE_TRISE, 37);
	MODIFY_REG(I2C2->CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), 180);
	
	MODIFY_REG(I2C2->OAR1, I2C_OAR1_ADD0 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD8_9 | I2C_OAR1_ADDMODE, I2C_OWNADDRESS1_7BIT);
  MODIFY_REG(I2C2->CR1, I2C_CR1_SMBUS | I2C_CR1_SMBTYPE | I2C_CR1_ENARP, I2C_MODE_I2C);
	
	SET_BIT(I2C2->CR1, I2C_CR1_PE); //Peripheral enable
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);//Acknowledge enable 
	
	MODIFY_REG(I2C2->OAR2, I2C_OAR2_ADD2, 0);
	USART_Send("\nI2C2_Init OK");
}

static void I2C1_Init()
{
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_IOPBEN); //I/O port B clock enable 
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_I2C1EN); //I2C2 clock enable

	SET_BIT(GPIOB->CRL, GPIO_CRL_CNF7_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_0 | GPIO_CRL_CNF6_0 |\
                  GPIO_CRL_MODE7_1 | GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE6_0); //Output mode, max speed 50 MHz.Alternate function output Open-drain
	
	tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
	
  CLEAR_BIT(I2C1->OAR2, I2C_OAR2_ENDUAL);
  CLEAR_BIT(I2C1->CR1, I2C_CR1_ENGC);
  CLEAR_BIT(I2C1->CR1, I2C_CR1_NOSTRETCH);
	
	CLEAR_BIT(I2C1->CR1, I2C_CR1_PE); //Peripheral disable
	
	MODIFY_REG(I2C1->CR2,I2C_CR2_FREQ,36); 
	MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE, 37);
	MODIFY_REG(I2C1->CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), 180);
	
	MODIFY_REG(I2C1->OAR1, I2C_OAR1_ADD0 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD8_9 | I2C_OAR1_ADDMODE, I2C_OWNADDRESS1_7BIT);
  MODIFY_REG(I2C1->CR1, I2C_CR1_SMBUS | I2C_CR1_SMBTYPE | I2C_CR1_ENARP, I2C_MODE_I2C);
	
	SET_BIT(I2C1->CR1, I2C_CR1_PE); //Peripheral enable
	MODIFY_REG(I2C1->CR1, I2C_CR1_ACK, I2C_CR1_ACK);//Acknowledge enable 
	
	MODIFY_REG(I2C1->OAR2, I2C_OAR2_ADD2, 0);
	USART_Send("\nI2C1_Init OK");
}

static void I2C_Write(uint16_t addr,uint8_t *buf, uint16_t bytes_count)
{
	uint8_t i;
	
	CLEAR_BIT(I2C2->CR1,I2C_CR1_POS);	
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	
	SET_BIT(I2C2->CR1,I2C_CR1_START);
	while (!READ_BIT(I2C2->SR1, I2C_SR1_SB)){}; //Set when a Start condition generated. 
		
	(void) I2C2->SR1;
	MODIFY_REG(I2C2->DR, I2C_DR_DR, SLAVE_OWN_ADDRESS | I2C_REQUEST_WRITE); 
	while (!READ_BIT(I2C2->SR1, I2C_SR1_ADDR)){};
		
	(void) I2C2->SR1;
  (void) I2C2->SR2;
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (uint8_t)(addr>>8));
  while (!READ_BIT(I2C2->SR1, I2C_SR1_TXE)){};
		
  MODIFY_REG(I2C2->DR, I2C_DR_DR, (uint8_t) addr);
  while (!READ_BIT(I2C2->SR1, I2C_SR1_TXE)){};
		
	for(i=0;i<bytes_count;i++)
  {
    MODIFY_REG(I2C2->DR, I2C_DR_DR, buf[i]);
    while (!READ_BIT(I2C2->SR1, I2C_SR1_TXE)){};
  }
	
	SET_BIT(I2C2->CR1, I2C_CR1_STOP);
	USART_Send("\nI2C_Write OK");	
}

static void I2C_Read(uint16_t addr, uint8_t *buf, uint16_t bytes_count)
{
	char str[128];
	uint16_t i;
	
  CLEAR_BIT(I2C2->CR1, I2C_CR1_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register.
  SET_BIT(I2C2->CR1,I2C_CR1_ACK); //Acknowledge enable 
  SET_BIT(I2C2->CR1, I2C_CR1_START); //Start generation 
  while (!READ_BIT(I2C2->SR1, I2C_SR1_SB)){}; //Start condition generated
		
	WRITE_REG(I2C2->DR, SLAVE_OWN_ADDRESS | I2C_REQUEST_WRITE); //Transmitting the address
  while (!READ_BIT(I2C2->SR1, I2C_SR1_ADDR)){}; //Address sent
		
	(void) I2C2->SR1;
	(void) I2C2->SR2;
	WRITE_REG(I2C2->DR, (uint8_t) (addr>>8)); //Transmitting the register address
  while (!READ_BIT(I2C2->SR1, I2C_SR1_TXE)){}; //Data register empty
		
	WRITE_REG(I2C2->DR, (uint8_t) (addr)); //Transmitting the register address
  while (!READ_BIT(I2C2->SR1, I2C_SR1_TXE)){}; //Data register empty
		
	SET_BIT(I2C2->CR1, I2C_CR1_START); //Start generation 
  while (!READ_BIT(I2C2->SR1, I2C_SR1_SB)){}; //Start condition generated
		
	WRITE_REG(I2C2->DR, SLAVE_OWN_ADDRESS | I2C_REQUEST_READ); //Receiving the register value
  while (!READ_BIT(I2C2->SR1, I2C_SR1_ADDR)){}; //Received address matched
		
	(void) I2C2->SR1;
	(void) I2C2->SR2;
		
	for(i=0;i<bytes_count;i++)
  {
    if(i<(bytes_count-1))
    {
      while (!READ_BIT(I2C2->SR1, I2C_SR1_RXNE)){}; //Data register not empty 
      buf[i] = READ_BIT(I2C2->DR, I2C_DR_DR); //Writing data to the buffer 
    }
    else
    {
      while (!READ_BIT(I2C2->SR1, I2C_SR1_RXNE)){}; //Data register not empty 
      buf[i] = READ_BIT(I2C2->DR, I2C_DR_DR); //Writing data to the buffer
			CLEAR_BIT(I2C2->CR1, I2C_CR1_ACK); //No acknowledge returned 
      SET_BIT(I2C2->CR1, I2C_CR1_STOP); //Stop generation
    }
	}
	USART_Send("\nI2C_Read OK");	
}

static void AHT10_Write(uint8_t *buf, uint16_t bytes_count)
{
	char str[40];
	uint8_t i;
	
	CLEAR_BIT(I2C1->CR1,I2C_CR1_POS);	
	MODIFY_REG(I2C1->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	
	SET_BIT(I2C1->CR1,I2C_CR1_START);
	while (!READ_BIT(I2C1->SR1, I2C_SR1_SB)){}; //Set when a Start condition generated. 
		
	(void) I2C1->SR1;
	MODIFY_REG(I2C1->DR, I2C_DR_DR, AHT10_ADRESS | I2C_REQUEST_WRITE); 
	while (!READ_BIT(I2C1->SR1, I2C_SR1_ADDR)){};
		
	(void) I2C1->SR1;
  (void) I2C1->SR2;
		
	for(i=0;i<bytes_count;i++)
  {
    MODIFY_REG(I2C1->DR, I2C_DR_DR, buf[i]);
    while (!READ_BIT(I2C1->SR1, I2C_SR1_TXE)){};
  }
	USART_Send("\n\nAHT10_Write OK");	
	SET_BIT(I2C1->CR1, I2C_CR1_STOP);
}

static void AHT10_Read(uint8_t *buf, uint16_t bytes_count)
{
	char str[128];
	uint16_t i;
	Delay(100);
  CLEAR_BIT(I2C1->CR1, I2C_CR1_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register.
  SET_BIT(I2C1->CR1,I2C_CR1_ACK); //Acknowledge enable 
	
  SET_BIT(I2C1->CR1, I2C_CR1_START); //Start generation 
  while (!READ_BIT(I2C1->SR1, I2C_SR1_SB)){}; //Start condition generated
	Delay(100);	
	WRITE_REG(I2C1->DR, AHT10_ADRESS | I2C_REQUEST_READ); //Transmitting the address
  while (!READ_BIT(I2C1->SR1, I2C_SR1_ADDR)){}; //Address sent
		
	(void) I2C1->SR1;
	(void) I2C1->SR2;
		
	for(i=0;i<bytes_count;i++)
  {
    if(i<(bytes_count-1))
    {
     while (!READ_BIT(I2C1->SR1, I2C_SR1_RXNE)){}; //Data register not empty 
      buf[i] = READ_BIT(I2C1->DR, I2C_DR_DR); //Writing data to the buffer 
    }
    else
    {
      while (!READ_BIT(I2C1->SR1, I2C_SR1_RXNE)){}; //Data register not empty 
      buf[i] = READ_BIT(I2C1->DR, I2C_DR_DR); //Writing data to the buffer
			CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK); //No acknowledge returned 
      SET_BIT(I2C1->CR1, I2C_CR1_STOP); //Stop generation
    }
	}
	USART_Send("\nI2C1_Read OK ");	

	AHT10_ADC_Raw = (((uint32_t)AHT10_RX_Data[3] & 15) << 16) | ((uint32_t)AHT10_RX_Data[4] << 8) | AHT10_RX_Data[5];
	AHT10_Temperature = (float)(AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;
	AHT10_ADC_Raw = ((uint32_t)AHT10_RX_Data[1] << 12) | ((uint32_t)AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
	AHT10_Humidity = (float)(AHT10_ADC_Raw*100.00/1048576.00);

	temperature = (int)((AHT10_Temperature)*10);
	
	humidity = (int)((AHT10_Humidity)*10);
	
	USART_Send("\nResult:");
	sprintf(str,"\nt = +%i.%i",temperature/10,temperature%10);
	USART_Send(str);
	sprintf(str,"\nh = %i.%i%%",humidity/10,humidity%10);
	USART_Send(str);
}


static void Delay(uint32_t time_delay)
{	
	uint32_t i;
	for(i = 0; i < time_delay; i++);
}

static void TIM2_Init()
{
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM2EN);
	NVIC_EnableIRQ(TIM2_IRQn);
	WRITE_REG(TIM2->PSC,36000-1);
	WRITE_REG(TIM2->ARR,2000);
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->SR &= ~(TIM_SR_UIF);
	USART_Send("\nTIM2_Init OK ");
}


static void TIM3_Init()
{
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM3EN);
	NVIC_EnableIRQ(TIM3_IRQn);
	WRITE_REG(TIM3->PSC,48000-1);
	WRITE_REG(TIM3->ARR,45000);
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->SR &= ~(TIM_SR_UIF);
	USART_Send("\nTIM3_Init OK ");
}

void TIM2_IRQHandler(void)
{
	char str[128];
	if(READ_BIT(TIM2->SR, TIM_SR_UIF))
  {
		CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
		Timer = 255;	
	}
}

void TIM3_IRQHandler(void)
{
	if(READ_BIT(TIM3->SR, TIM_SR_UIF))
  {
		CLEAR_BIT(TIM3->SR, TIM_SR_UIF);
		min_2 +=1;
		if (min_2 == 120)
		{
			EEPROM_Shift(0,5,4);
			I2C_Write(0,wr_data,4);
			min_2 = 0;
		}
	}	
}

static void Clock_Init()
{
	SET_BIT(RCC->CR,RCC_CR_HSEON);
	while (!READ_BIT(RCC->CR,RCC_CR_HSERDY)){};

	CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
  SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);
		
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL,
             RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);		
	
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	while (!READ_BIT(RCC->CR,RCC_CR_PLLRDY)){};

	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
  while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

	USART_Send("\nClock_Init OK");
}

static void RCC_DeInit(void)
{
	SET_BIT(RCC->CR, RCC_CR_HSION);
  while(READ_BIT(RCC->CR, RCC_CR_HSIRDY == RESET)) {}
	MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
  CLEAR_REG(RCC->CFGR);
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET) {}
	CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
  while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET) {}
	CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
  while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET) {}
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
  SET_BIT(RCC->CSR, RCC_CSR_RMVF);
	CLEAR_REG(RCC->CIR);
}


static void SPI1_Init()
{
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_IOPAEN);
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN);
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_AFIOEN);	
	SET_BIT(RCC->AHBENR,RCC_AHBENR_DMA1EN);
/*
	MODIFY_REG(GPIOA->CRL,GPIO_CRL_MODE5_0|GPIO_CRL_MODE7_0|GPIO_CRL_CNF5_0|GPIO_CRL_CNF7_0, //(reg,reset,set)
													GPIO_CRL_MODE5_1|GPIO_CRL_MODE7_1|GPIO_CRL_CNF5_1|GPIO_CRL_CNF7_1);  //SCK & MOSI
	
	MODIFY_REG(GPIOA->CRL,GPIO_CRL_MODE1_0|GPIO_CRL_MODE2_0|GPIO_CRL_CNF1|GPIO_CRL_CNF2, //(reg,reset,set)
													GPIO_CRL_MODE1_1|GPIO_CRL_MODE2_1);
	
	MODIFY_REG(GPIOA->CRL,GPIO_CRL_MODE3_0|GPIO_CRL_CNF3, //(reg,reset,set)
													GPIO_CRL_MODE3_1); //DC
	*/
	GPIOA->CRL|= GPIO_CRL_MODE5 | GPIO_CRL_MODE7;
	GPIOA->CRL&= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF7);
	GPIOA->CRL|=  GPIO_CRL_CNF5_1 | GPIO_CRL_CNF7_1;
	
	
	
	SET_BIT(GPIOA->BSRR,GPIO_BSRR_BS2);  //SSD_1306 Reset
	SET_BIT(GPIOA->BSRR,GPIO_BSRR_BR1); //
	SET_BIT(GPIOA->BSRR,GPIO_BSRR_BS1);

	
	MODIFY_REG(SPI1->CR1,SPI_CR1_BR,SPI_CR1_BR_0); // Fpclk/x
	CLEAR_BIT(SPI1->CR1,SPI_CR1_CPOL); //clock to 0 when idle 
	CLEAR_BIT(SPI1->CR1,SPI_CR1_CPHA); // The first clock transition is the first data capture edge 
	CLEAR_BIT(SPI1->CR1,SPI_CR1_DFF); //8bit
	CLEAR_BIT(SPI1->CR1,SPI_CR1_LSBFIRST); // MSB transmitted first 
	SET_BIT(SPI1->CR1,SPI_CR1_SSM); //Software slave management enabled 
	SET_BIT(SPI1->CR1,SPI_CR1_SSI); // 
	SET_BIT(SPI1->CR2,SPI_CR2_TXDMAEN);
	SET_BIT(SPI1->CR1,SPI_CR1_MSTR);//Master configuration
	SET_BIT(SPI1->CR1,SPI_CR1_SPE); //SPI1 on
	
	SET_BIT(DMA1_Channel3->CCR,DMA_CCR1_PSIZE_0);
	SET_BIT(DMA1_Channel3->CCR,DMA_CCR1_DIR);
	SET_BIT(DMA1_Channel3->CCR,DMA_CCR1_MINC);
	SET_BIT(DMA1_Channel3->CCR,DMA_CCR1_PL);
	
	USART_Send("\nSPI1_Init OK ");
}


static void EXTI_Iint()
{
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_AFIOEN);
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_IOPAEN);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE0|GPIO_CRL_CNF0_0,GPIO_CRL_CNF0_1);	
		
	//SET_BIT(GPIOA->ODR,GPIO_ODR_ODR0);
	AFIO->EXTICR [0] |= AFIO_EXTICR1_EXTI0_PA;	
	
	SET_BIT(EXTI->RTSR,EXTI_RTSR_TR0);
	SET_BIT(EXTI->IMR,EXTI_IMR_MR0);
	SET_BIT(EXTI->PR,EXTI_PR_PR0);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void)
{
	SET_BIT(EXTI->PR,EXTI_PR_PR0);
	I2C_Read(0,rd_value,20);
	for(uint16_t i=1200;i<2400;i++)
	{
		SSD1306_Buffer[i]=0;
	}	
	while (READ_BIT(GPIOA->IDR,GPIO_IDR_IDR0)==1)
	{
		sprintf(str,"1h/a t=+%i.%i h=%i ",rd_value[0],rd_value[1],rd_value[2]);	
		ssd1306_SetCursor(0,0);
		ssd1306_WriteString(str,Font_7x10,WHITE);
			
		sprintf(str,"2h/a t=+%i.%i h=%i ",rd_value[4],rd_value[5],rd_value[6]);	
		ssd1306_SetCursor(0,12);
		ssd1306_WriteString(str,Font_7x10,WHITE);	

		sprintf(str,"3h/a t=+%i.%i h=%i ",rd_value[8],rd_value[9],rd_value[10]);	
		ssd1306_SetCursor(0,24);
		ssd1306_WriteString(str,Font_7x10,WHITE);
		
		sprintf(str,"4h/a t=+%i.%i h=%i ",rd_value[12],rd_value[13],rd_value[14]);	
		ssd1306_SetCursor(0,36);
		ssd1306_WriteString(str,Font_7x10,WHITE);
		
		sprintf(str,"5h/a t=+%i.%i h=%i ",rd_value[16],rd_value[17],rd_value[18]);	
		ssd1306_SetCursor(0,48);
		ssd1306_WriteString(str,Font_7x10,WHITE);
	}
	for(uint16_t i=1200;i<2400;i++)
	{
		SSD1306_Buffer[i]=0;
	}	
	USART_Send("\nBUTTON");
}

static void EEPROM_Shift(uint16_t addr,uint8_t count_repeat,uint8_t size_array) // /size_array/.../.../.../ ..count_repeat.. /.../
{
	int i = size_array * (count_repeat - 1);
	uint8_t shift[4];
	while (i>=0)
	{
		I2C_Read(addr+i,shift,size_array);
		I2C_Write(addr+i+size_array,shift,size_array);
		i= i - size_array;	
	}
	USART_Send("\nShift_OK");
}	
