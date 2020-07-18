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
    #include "otm8009a.h"
	#include "STM32DSI.h"
#else
	#include "stm32f4xx_hal_gpio.h"
	#include "stm32f4xx_hal_ltdc.h"

	#include "stm32469i_discovery_lcd.h"
#endif

static ltdcConfig driverCfg = {
	800, 800,								//This is hack... on STM32F769i..I couldn't make it work unless width=height= 800
	120, 12,									// Horizontal, Vertical sync (pixels)
	120, 12,									// Horizontal, Vertical back porch (pixels)
	120, 12,									// Horizontal, Vertical front porch (pixels)
//	41, 10,									// Horizontal, Vertical sync (pixels)
//	13, 2,									// Horizontal, Vertical back porch (pixels)
//	32, 2,									// Horizontal, Vertical front porch (pixels)
	0,										// Sync flags
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

static DSI_VidCfgTypeDef hdsivideo_handle;
extern DSI_HandleTypeDef hdsi_eval;
orientation_t lcdOrientation = GDISP_DEFAULT_ORIENTATION;
static inline void init_board(GDisplay* g) {
	g->board = 0;
  DSI_PLLInitTypeDef dsiPllInit;
  uint32_t LcdClock  = 27429; /*!< LcdClk = 27429 kHz */
  
  uint32_t laneByteClk_kHz = 0;
  uint32_t                   VSA; /*!< Vertical start active time in units of lines */
  uint32_t                   VBP; /*!< Vertical Back Porch time in units of lines */
  uint32_t                   VFP; /*!< Vertical Front Porch time in units of lines */
  uint32_t                   VACT; /*!< Vertical Active time in units of lines = imageSize Y in pixels to display */
  uint32_t                   HSA; /*!< Horizontal start active time in units of lcdClk */
  uint32_t                   HBP; /*!< Horizontal Back Porch time in units of lcdClk */
  uint32_t                   HFP; /*!< Horizontal Front Porch time in units of lcdClk */
  uint32_t                   HACT; /*!< Horizontal Active time in units of lcdClk = imageSize X in pixels to display */
  
  
  /* Toggle Hardware Reset of the DSI LCD using
  * its XRES signal (active low) */
  BSP_LCD_Reset();
  
  /* Call first MSP Initialize only in case of first initialization
  * This will set IP blocks LTDC, DSI and DMA2D
  * - out of reset
  * - clocked
  * - NVIC IRQ related to IP blocks enabled
  */
  BSP_LCD_MspInit();
  
/*************************DSI Initialization***********************************/  
  
  /* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
  hdsi_eval.Instance = DSI;
  
  HAL_DSI_DeInit(&(hdsi_eval));
  

  dsiPllInit.PLLNDIV  = 100;
  dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV5;
  dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;
  laneByteClk_kHz = 62500; /* 500 MHz / 8 = 62.5 MHz = 62500 kHz */
  
  /* Set number of Lanes */
  hdsi_eval.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  
  /* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
  hdsi_eval.Init.TXEscapeCkdiv = laneByteClk_kHz/15620; 
  
  HAL_DSI_Init(&(hdsi_eval), &(dsiPllInit));
  
  /* Timing parameters for all Video modes
  * Set Timing parameters of LTDC depending on its chosen orientation
  */
	/* lcd_orientation == LCD_ORIENTATION_LANDSCAPE */
	VSA  = OTM8009A_800X480_VSYNC;        /* 12 */
	VBP  = OTM8009A_800X480_VBP;          /* 12 */
	VFP  = OTM8009A_800X480_VFP;          /* 12 */
	HSA  = OTM8009A_800X480_HSYNC;        /* 120 */
	HBP  = OTM8009A_800X480_HBP;          /* 120 */
	HFP  = OTM8009A_800X480_HFP;          /* 120 */
	uint32_t lcd_x_size = 480;//OTM8009A_800X480_WIDTH;  /* 800 */
	uint32_t lcd_y_size = 800;//OTM8009A_800X480_HEIGHT; /* 480 */
	lcdOrientation = GDISP_DEFAULT_ORIENTATION;
//	if(lcdOrientation == LCD_ORIENTATION_PORTRAIT)
//	{
//		driverCfg.bglayer.cx = driverCfg.bglayer.width  = driverCfg.width  = lcd_x_size = OTM8009A_480X800_WIDTH;  /* 480 */
//		driverCfg.bglayer.cy = driverCfg.bglayer.height = driverCfg.height = lcd_y_size = OTM8009A_480X800_HEIGHT; /* 800 */
//	}
//	else
//	{
//		driverCfg.bglayer.cx = driverCfg.bglayer.width  = driverCfg.width  = lcd_x_size = OTM8009A_800X480_WIDTH;  /* 800 */
//		driverCfg.bglayer.cy = driverCfg.bglayer.height = driverCfg.height = lcd_y_size = OTM8009A_800X480_HEIGHT; /* 480 */
//	}
  
  HACT = lcd_x_size;
  VACT = lcd_y_size;
  
  
  hdsivideo_handle.VirtualChannelID = LCD_OTM8009A_ID;
  hdsivideo_handle.ColorCoding = LCD_DSI_PIXEL_DATA_FMT_RBG888;
  hdsivideo_handle.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  hdsivideo_handle.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  hdsivideo_handle.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;  
  hdsivideo_handle.Mode = DSI_VID_MODE_BURST; /* Mode Video burst ie : one LgP per line */
  hdsivideo_handle.NullPacketSize = 0xFFF;
  hdsivideo_handle.NumberOfChunks = 0;
  hdsivideo_handle.PacketSize                = HACT; /* Value depending on display orientation choice portrait/landscape */ 
  hdsivideo_handle.HorizontalSyncActive      = (HSA * laneByteClk_kHz) / LcdClock;
  hdsivideo_handle.HorizontalBackPorch       = (HBP * laneByteClk_kHz) / LcdClock;
  hdsivideo_handle.HorizontalLine            = ((HACT + HSA + HBP + HFP) * laneByteClk_kHz) / LcdClock; /* Value depending on display orientation choice portrait/landscape */
  hdsivideo_handle.VerticalSyncActive        = VSA;
  hdsivideo_handle.VerticalBackPorch         = VBP;
  hdsivideo_handle.VerticalFrontPorch        = VFP;
  hdsivideo_handle.VerticalActive            = VACT; /* Value depending on display orientation choice portrait/landscape */
  
  /* Enable or disable sending LP command while streaming is active in video mode */
  hdsivideo_handle.LPCommandEnable = DSI_LP_COMMAND_ENABLE; /* Enable sending commands in mode LP (Low Power) */
  
  /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP regions */
  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
  hdsivideo_handle.LPLargestPacketSize = 64;
  
  /* Largest packet size possible to transmit in LP mode in HFP region during VACT period */
  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
  hdsivideo_handle.LPVACTLargestPacketSize = 64;
  
  
  /* Specify for each region of the video frame, if the transmission of command in LP mode is allowed in this region */
  /* while streaming is active in video mode                                                                         */
  hdsivideo_handle.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;   /* Allow sending LP commands during HFP period */
  hdsivideo_handle.LPHorizontalBackPorchEnable  = DSI_LP_HBP_ENABLE;   /* Allow sending LP commands during HBP period */
  hdsivideo_handle.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;  /* Allow sending LP commands during VACT period */
  hdsivideo_handle.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;   /* Allow sending LP commands during VFP period */
  hdsivideo_handle.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;   /* Allow sending LP commands during VBP period */
  hdsivideo_handle.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE; /* Allow sending LP commands during VSync = VSA period */
  
  /* Configure DSI Video mode timings with settings set above */
  HAL_DSI_ConfigVideoMode(&(hdsi_eval), &(hdsivideo_handle));
  
  /* Enable the DSI host and wrapper : but LTDC is not started yet at this stage */
  HAL_DSI_Start(&(hdsi_eval));
/*************************End DSI Initialization*******************************/ 
}


static inline void post_init_board(GDisplay* g)
{
	(void)g;
	
	BSP_SDRAM_Init();
  
  /* Initialize the font */
//  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  
	/************************End LTDC Initialization*******************************/
		
		
	/***********************OTM8009A Initialization********************************/  
  
  /* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver)
  *  depending on configuration set in 'hdsivideo_handle'.
  */
  OTM8009A_Init(hdsivideo_handle.ColorCoding, lcdOrientation==GDISP_ROTATE_PORTRAIT?OTM8009A_ORIENTATION_PORTRAIT:OTM8009A_ORIENTATION_LANDSCAPE);
  
	/***********************End OTM8009A Initialization****************************/ 

	// ------------------------------------------------------------------------
	
	//BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);
	//BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER_BACKGROUND);
	//BSP_LCD_SetLayerVisible(LTDC_ACTIVE_LAYER_FOREGROUND, DISABLE);
}

static inline void set_backlight(GDisplay* g, uint8_t percent)
{
	(void)g;
	(void)percent;
}
int16_t getLCDTFTWidth(void){
	return driverCfg.width;
}
int16_t getLCDTFTHeight(void){
	return driverCfg.height;
}
#endif /* _GDISP_LLD_BOARD_H */
