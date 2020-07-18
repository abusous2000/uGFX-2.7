/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#include "gfx.h"
#include "stm32f769i_discovery_sdram.h"

#ifndef GFX_LTDC_USE_DIRECTIO
	#define GFX_LTDC_USE_DIRECTIO	TRUE
#endif
#ifndef GFX_LTDC_TIMING_SET
	/* Options are:
	 *	0 - don't initialise the display VCO and LTDC clocks - something else will do it
	 *	1 - uGFX preferred timings (default)
	 *	2 - Alternate timings	
	 */
	#define GFX_LTDC_TIMING_SET		0
#endif

#if GFX_USE_OS_CHIBIOS && !GFX_USE_DIRECTIO
	#include "hal.h"
	#include "ch.h"
	#include "STM32DSI.h"
#else
	#include "stm32f7xx_hal_rcc.h"
	#include "stm32f7xx_hal_gpio.h"
#endif
#include <string.h>

#if !GFX_USE_OS_CHIBIOS
	#define AFRL	AFR[0]
	#define AFRH	AFR[1]
#endif

#define ALLOW_2ND_LAYER		TRUE
#define LTDC_UNUSED_LAYER_CONFIG	{ 0, 1, 1, 1, LTDC_FMT_L8, 0, 0, 1, 1, 0x000000, 0x000000, LTDC_BLEND_FIX1_FIX2, 0, 0, 0, 0 }
#define LTDC_PIXELFORMAT	LTDC_FMT_ARGB8888
#define LTDC_PIXELBYTES		4
#define LTDC_PIXELBITS		32
static ltdcConfig driverCfg = {
		800, 800,								//This is hack... on STM32F769i..I couldn't make it work unless width=height= 800
		120, 12,								// Horizontal, Vertical sync (pixels)
		120, 12,								// Horizontal, Vertical back porch (pixels)
		120, 12,								// Horizontal, Vertical front porch (pixels)
	//	41, 10,									// Horizontal, Vertical sync (pixels)
	//	13, 2,									// Horizontal, Vertical back porch (pixels)
	//	32, 2,									// Horizontal, Vertical front porch (pixels)
		0x00000000,								// Sync flags
		0x000000,								// Clear color (RGB888)
		{										// Background layer config
			(LLDCOLOR_TYPE *)SDRAM_DEVICE_ADDR,	// Frame buffer address
			800, 800,							// Width, Height (pixels)
			800 * LTDC_PIXELBYTES,				// Line pitch (bytes)
			LTDC_PIXELFORMAT,					// Pixel format
			0, 0,								// Start pixel position (x, y)
			800, 800,							// Size of virtual layer (cx, cy)
			LTDC_COLOR_FUCHSIA,					// Default color (ARGB8888)
			0x980088,							// Color key (RGB888)
			LTDC_BLEND_FIX1_FIX2,				// Blending factors
			0,									// Palette (RGB888, can be NULL)
			0,									// Palette length
			0xFF,								// Constant alpha factor
			LTDC_LEF_ENABLE						// Layer configuration flags
		},

		LTDC_UNUSED_LAYER_CONFIG				// Foreground layer config
	};

orientation_t lcdOrientation = GDISP_DEFAULT_ORIENTATION;


static inline void init_board(GDisplay *g) {
	(void) g;
	lcdOrientation = GDISP_DEFAULT_ORIENTATION;
	initDSI(lcdOrientation);
}

static inline void post_init_board(GDisplay* g) {
	(void) g;

	postInitDSI(lcdOrientation);
}
#define LCD_OTM8009A_ID  ((uint32_t) 0)
#define DSI_DCS_SHORT_PKT_WRITE_P1  0x00000015U
static inline void set_backlight(GDisplay* g, uint8_t percent) {
	(void) g; (void)percent;
}
/**
  * @brief  Set the brightness value
  * @param  BrightnessValue: [00: Min (black), 100 Max]
  */
int16_t getLCDTFTWidth(void){
	return driverCfg.width;
}
int16_t getLCDTFTHeight(void){
	return driverCfg.height;
}
#endif /* _GDISP_LLD_BOARD_H */
