/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f10x.h"
#include "FT800.h"
#include <string>

using namespace std;

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 empty sample (trace via NONE).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the NONE output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
/*****************************************************/


volatile uint32_t msTick;

extern "C" void SysTick_Handler(void);

void Delay_ms(uint32_t time_ms);
uint32_t getTick();

void init_pins();
void init_spi();

uint8_t spi_transfer(uint8_t data);
void ft800cmdWrite(unsigned char ftCommand);
void ft800memWrite8(unsigned long ftAddress, unsigned char ftData8);
void ft800memWrite16(unsigned long ftAddress, unsigned int ftData16);
void ft800memWrite32(unsigned long ftAddress, unsigned int ftData32);
unsigned char ft800memRead8(unsigned long ftAddress);
unsigned int ft800memRead16(unsigned long ftAddress);
unsigned int incCMDOffset(unsigned int currentOffset, unsigned char commandSize);

void initDisplay();
void startFrame();
void endFrame();
void FTprint(uint16_t x, uint16_t y, string text, uint16_t option);

unsigned char ft800Gpio;
unsigned int cmdBufferRd = 0x0000;
unsigned int cmdBufferWr = 0x0000;
unsigned int cmdOffset = 0x0000;

int main(int argc, char* argv[])
{
	SystemInit();
	SystemCoreClockUpdate();

	SysTick_Config(SystemCoreClock / 1000);

	init_pins();
	init_spi();

	GPIO_SetBits(GPIOA, GPIO_Pin_3);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	Delay_ms(1000);

	ft800cmdWrite(FT800_ACTIVE);
	Delay_ms(5);
	//ft800cmdWrite(FT800_CLKEXT);
	//Delay_ms(5);
	ft800cmdWrite(FT800_CLK48M);
	Delay_ms(5);
	//ft800cmdWrite(FT800_CORERST);
	//Delay_ms(5);

	unsigned char id = ft800memRead8(REG_ID);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	uint32_t t_delay = 300;

	if(id != 0x7C)
	{
		t_delay=1000; //used for debugging, slow blinking light
	}

	initDisplay();

	//string text = "STM32F103";

	while (1)
	{
		while (cmdBufferWr != cmdBufferRd)
		{
			cmdBufferRd = ft800memRead16(REG_CMD_READ);
			cmdBufferWr = ft800memRead16(REG_CMD_WRITE);
		}

		cmdOffset = cmdBufferWr;

		startFrame();
		ft800memWrite32(RAM_CMD + cmdOffset, (DL_COLOR_RGB | RGB(100,255,0) ));
		cmdOffset = incCMDOffset(cmdOffset, 4);



		//FTprint(0,50,"+", OPT_CENTERX);


		FTprint(480/2, 0, "STM32F103", OPT_CENTERX);
		FTprint(480/2, 50, "+", OPT_CENTERX);
		FTprint(25, 100, "FT800",0);
		FTprint(175, 100, "(Gameduino 2)",0);
		FTprint(50, 200, "Radek",0);
		FTprint(180, 200, "S",0);

		endFrame();



		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		Delay_ms(t_delay);
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		Delay_ms(t_delay);

	}
}

void initDisplay()
{
	// Initialization for 480x272 display
	ft800memWrite8(REG_PCLK, 0);
	ft800memWrite8(REG_PWM_DUTY, 0);

	ft800memWrite16(REG_HSIZE, 480);
	ft800memWrite16(REG_HCYCLE, 548);
	ft800memWrite16(REG_HOFFSET, 43);
	ft800memWrite16(REG_HSYNC0, 0);
	ft800memWrite16(REG_HSYNC1, 41);
	ft800memWrite16(REG_VSIZE, 272);
	ft800memWrite16(REG_VCYCLE, 292);
	ft800memWrite16(REG_VOFFSET, 12);
	ft800memWrite16(REG_VSYNC0, 0);
	ft800memWrite16(REG_VSYNC1, 10);
	ft800memWrite8(REG_SWIZZLE, 0);
	ft800memWrite8(REG_PCLK_POL, 1);

	// disable touch
	ft800memWrite8(REG_TOUCH_MODE, 0);
	ft800memWrite16(REG_TOUCH_RZTHRESH, 0);

	// disable sound
	ft800memWrite8(REG_VOL_PB, 0);
	ft800memWrite8(REG_VOL_SOUND, 0);
	ft800memWrite16(REG_SOUND, 0x6000);

	// start display list and clear screen
	ft800memWrite32(RAM_DL, DL_CLEAR_RGB);
	ft800memWrite32(RAM_DL + 4, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
	ft800memWrite32(RAM_DL + 8	, DL_DISPLAY);
	ft800memWrite32(REG_DLSWAP, DLSWAP_FRAME);

	ft800Gpio = ft800memRead8(REG_GPIO);
	ft800Gpio = ft800Gpio | 0x80;
	ft800memWrite8(REG_GPIO, ft800Gpio);
	ft800memWrite8(REG_PCLK, 5);

	// ramp up brightness
	for(int duty = 0; duty <= 128; duty++)
	{
		ft800memWrite8(REG_PWM_DUTY, duty);
		Delay_ms(10);
	}
}

void startFrame()
{
	ft800memWrite32(RAM_CMD + cmdOffset, (CMD_DLSTART));
	cmdOffset = incCMDOffset(cmdOffset, 4);

	ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR_RGB | RGB(1,1,1)));
	cmdOffset = incCMDOffset(cmdOffset, 4);

	ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
	cmdOffset = incCMDOffset(cmdOffset, 4);
}

void endFrame()
{
	ft800memWrite32(RAM_CMD + cmdOffset, (DL_DISPLAY));
	cmdOffset = incCMDOffset(cmdOffset, 4);;

	ft800memWrite32(RAM_CMD + cmdOffset, (CMD_SWAP));
	cmdOffset = incCMDOffset(cmdOffset, 4);

	ft800memWrite16(REG_CMD_WRITE, (cmdOffset));
}

void FTprint(uint16_t x, uint16_t y, string text, uint16_t option)
{
	uint8_t len= text.length();

	ft800memWrite32(RAM_CMD + cmdOffset, (CMD_TEXT) );
	cmdOffset = incCMDOffset(cmdOffset, 4);

	ft800memWrite16(RAM_CMD + cmdOffset, x );
	cmdOffset = incCMDOffset(cmdOffset, 2);

	ft800memWrite16(RAM_CMD + cmdOffset, y );
	cmdOffset = incCMDOffset(cmdOffset, 2);

	ft800memWrite16(RAM_CMD + cmdOffset, 31 );
	cmdOffset = incCMDOffset(cmdOffset, 2);

	ft800memWrite16(RAM_CMD + cmdOffset, option);
	cmdOffset = incCMDOffset(cmdOffset, 2);

	ft800memWrite8(RAM_CMD + cmdOffset, ' ' );
			cmdOffset = incCMDOffset(cmdOffset, 1);
	for (uint8_t i=0; i<len; i++)
	{
		ft800memWrite8(RAM_CMD + cmdOffset, text.at(i) );
		cmdOffset = incCMDOffset(cmdOffset, 1);
	}

	ft800memWrite16(RAM_CMD + cmdOffset, '\0' );
	cmdOffset = incCMDOffset(cmdOffset, 2);

}

extern "C" void SysTick_Handler(void)
{
	msTick++;
}

void init_pins()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef gpioInitStruct;

	gpioInitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);

	gpioInitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
}

void init_spi()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitStruct;

	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}

uint8_t spi_transfer(uint8_t data)
{
	//uint8_t ret_data;

	//SPI_I2S_SendData( SPI1, data);
	SPI1->DR = data;
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) );
	while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) );
	while (SPI1->SR & (SPI_I2S_FLAG_BSY));
	//ret_data = SPI_I2S_ReceiveData( SPI1 );


	//return ret_data;
	return SPI1->DR;
}

void ft800cmdWrite(unsigned char ftCommand)
{
	unsigned char cZero = 0x00;
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	spi_transfer( ftCommand );
	spi_transfer( cZero );
	spi_transfer( cZero );
	GPIO_SetBits(GPIOA,GPIO_Pin_3);
}

void ft800memWrite8(unsigned long ftAddress, unsigned char ftData8)
{
	unsigned char cTempAddr[3];

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_WRITE;
	cTempAddr[1] = (char) (ftAddress >> 8);
	cTempAddr[0] = (char) (ftAddress);

	//GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	for (int i = 2; i >= 0; i--)
	{
		spi_transfer( cTempAddr[i]);
	}


	spi_transfer( ftData8);

	GPIO_SetBits(GPIOA,GPIO_Pin_3);

}

void ft800memWrite16(unsigned long ftAddress, unsigned int ftData16)
{
	unsigned char cTempAddr[3];
	unsigned char cTempData[2];

	cTempData[1] = (char) (ftData16 >> 8);
	cTempData[0] = (char) (ftData16);

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_WRITE;
	cTempAddr[1] = (char) (ftAddress >> 8);
	cTempAddr[0] = (char) (ftAddress);


	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	for (int i = 2; i >= 0; i--)
	{
		spi_transfer( cTempAddr[i]);
	}

	for (int j = 0; j < sizeof(cTempData); j++)
	{
		spi_transfer( cTempData[j]);
	}

	GPIO_SetBits(GPIOA,GPIO_Pin_3);

}

void ft800memWrite32(unsigned long ftAddress, unsigned int ftData32)
{
	unsigned char cTempAddr[3];
	unsigned char cTempData[4];

	cTempData[3] = (char) (ftData32 >> 24);
	cTempData[2] = (char) (ftData32 >> 16);
	cTempData[1] = (char) (ftData32 >> 8);
	cTempData[0] = (char) (ftData32);

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_WRITE;
	cTempAddr[1] = (char) (ftAddress >> 8);
	cTempAddr[0] = (char) (ftAddress);


	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	for (int i = 2; i >= 0; i--)
	{
		spi_transfer( cTempAddr[i]);
	}

	for (int j = 0; j < sizeof(cTempData); j++)
	{
		spi_transfer( cTempData[j]);
	}

	GPIO_SetBits(GPIOA,GPIO_Pin_3);

}

unsigned char ft800memRead8(unsigned long ftAddress)
{
	unsigned char ftData8 = ZERO;
	unsigned char cTempAddr[3];
	unsigned char cZeroFill = ZERO;

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_READ;
	cTempAddr[1] = (char) (ftAddress >> 8);
	cTempAddr[0] = (char) (ftAddress);


	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	for (int i = 2; i >= 0; i--)
	{

		spi_transfer( cTempAddr[i]);
	}


	spi_transfer( cZeroFill);  // send dummy byte

	ftData8 = spi_transfer(cZeroFill);


	GPIO_SetBits(GPIOA,GPIO_Pin_3);



	return ftData8;																				// Return 8-bits
}

unsigned int ft800memRead16(unsigned long ftAddress)
{
	unsigned int ftData16;
	unsigned char cTempAddr[3];
	unsigned char cTempData[2];
	unsigned char cZeroFill = ZERO;

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_READ;
	cTempAddr[1] = (char) (ftAddress >> 8);
	cTempAddr[0] = (char) (ftAddress);


	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	for (int i = 2; i >= 0; i--)
	{
		spi_transfer( cTempAddr[i]);
	}


	spi_transfer( cZeroFill);

	for (int j = 0; j < sizeof(cTempData); j++)
	{
		cTempData[j] = spi_transfer( cZeroFill);
	}

	ftData16 = (cTempData[1]<< 8) | (cTempData[0]);

	GPIO_SetBits(GPIOA,GPIO_Pin_3);

	return ftData16;																				// Return 8-bits
}

void Delay_ms(uint32_t time_ms)
{
	uint32_t start, end;
	start = getTick();
	end = getTick() + time_ms;


	if(getTick() < end)
	{
		while((getTick() >= start) && (getTick() < end))
		{

		}
	}
	else
	{
		while((getTick() >= start) || (getTick() < end))
		{

		}
	}
}

uint32_t getTick()
{
	return msTick;
}

unsigned int incCMDOffset(unsigned int currentOffset, unsigned char commandSize)
{
	unsigned int newOffset;
	newOffset = currentOffset + commandSize;
	if(newOffset > 4095)
	{
		newOffset = (newOffset - 4096);
	}
	return newOffset;																		// Return new offset
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
