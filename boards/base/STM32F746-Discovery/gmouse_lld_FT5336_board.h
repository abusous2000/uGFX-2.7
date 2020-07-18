/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#ifndef _GINPUT_LLD_MOUSE_BOARD_H
#define _GINPUT_LLD_MOUSE_BOARD_H

#include "stm32f7xx.h"
#include "stm32f7_i2c.h"

// Resolution and Accuracy Settings
#define GMOUSE_FT5336_PEN_CALIBRATE_ERROR		8
#define GMOUSE_FT5336_PEN_CLICK_ERROR			6
#define GMOUSE_FT5336_PEN_MOVE_ERROR			4
#define GMOUSE_FT5336_FINGER_CALIBRATE_ERROR	14
#define GMOUSE_FT5336_FINGER_CLICK_ERROR		18
#define GMOUSE_FT5336_FINGER_MOVE_ERROR			14

// How much extra data to allocate at the end of the GMouse structure for the board's use
#define GMOUSE_FT5336_BOARD_DATA_SIZE			0

// The FT5336 I2C slave address (including the R/W bit)
#define FT5336_SLAVE_ADDR 0x70

#if !GFX_USE_OS_CHIBIOS
	#define AFRL	AFR[0]
	#define AFRH	AFR[1]
#endif
#ifdef LCD_I2C_ALREADY_INITIALIZED
extern uint32_t FT5336_write_reg(uint8_t reg, uint8_t val);
extern uint16_t FT5336_read_word(uint8_t reg);
extern uint8_t FT5336_read_byte(uint8_t reg);
#endif

static bool_t init_board(GMouse* m, unsigned instance){(void)m;(void)instance;

/*
 * changed by AbuSous2000
 * LCD Used I2C3 when audio decoder is using it; in that case it is better off
 * to leave ChibiOS control the driver. Three functions where taken out and
 * you are expected to implement
 */
#ifndef LCD_I2C_ALREADY_INITIALIZED
	// I2C3_SCL    GPIOH7, alternate, opendrain, highspeed
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;					// Enable clock for
	GPIOH->MODER |= GPIO_MODER_MODER7_1;					// Alternate function
	GPIOH->OTYPER |= GPIO_OTYPER_OT_7;						// OpenDrain
	GPIOH->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR7;				// LowSpeed
	GPIOH->AFRL |= ((uint32_t)0x04 << 4*7);							// AF4

	// I2C3_SDA    GPIOH8, alternate, opendrain, highspeed
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;					// Enable clock
	GPIOH->MODER |= GPIO_MODER_MODER8_1;					// Alternate function
	GPIOH->OTYPER |= GPIO_OTYPER_OT_8;						// OpenDrain
	GPIOH->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR8;				// LowSpeed
	GPIOH->AFRH |= ((uint32_t)0x04 << 4*0);							// AF4

	// Initialize the I2C3 peripheral
	if (!(i2cSTM32Init(I2C3))) {
		return FALSE;
	}
#endif
	return TRUE;
}

static void write_reg(GMouse* m, uint8_t reg, uint8_t val){(void)m;

#ifndef LCD_I2C_ALREADY_INITIALIZED
	i2cWriteReg(I2C3, FT5336_SLAVE_ADDR, reg, val);
#else
	FT5336_write_reg(reg,val);
#endif
}

static uint8_t read_byte(GMouse* m, uint8_t reg){(void)m;

#ifndef LCD_I2C_ALREADY_INITIALIZED
	return i2cReadByte(I2C4, FT5336_SLAVE_ADDR, reg);
#else
	return FT5336_read_byte(reg);
#endif
}

static uint16_t read_word(GMouse* m, uint8_t reg){(void)m;

#ifndef LCD_I2C_ALREADY_INITIALIZED
	return i2cReadWord(I2C4, FT5336_SLAVE_ADDR, reg);
#else
	return FT5336_read_word(reg);
#endif
}

#endif /* _GINPUT_LLD_MOUSE_BOARD_H */
