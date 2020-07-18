/*
 * STM32DSI.c
 *
 *  Created on: May 11, 2020
 *      Author: abusous2000
 */
#include "Strust4EmbeddedConf.h"
#if USE_LCD_TFT == 1
#include "ch.h"
#include "hal.h"
#include "gfx.h"
#include "STM32DSI.h"
#include "otm8009a.h"

extern uint8_t BSP_SDRAM_Init(void);
DSI_HandleTypeDef hdsi_eval;

void BSP_LCD_Reset(void){
    palSetPadMode(GPIOJ, GPIOJ_DSI_RESET,  PAL_STM32_MODE_OUTPUT | PAL_STM32_OSPEED_HIGHEST|  PAL_STM32_PUPDR_PULLUP );
    /* Activate XRES active low */
    palClearLine(LINE_DSI_RESET);
    chThdSleepMilliseconds(40); /* wait 20 ms */
    /* Desactivate XRES */
    palSetLine(LINE_DSI_RESET);
    /* Wait for 10ms after releasing XRES before sending commands */
    chThdSleepMilliseconds(20);
}

HAL_StatusTypeDef HAL_DSI_ConfigVideoMode(DSI_HandleTypeDef *hdsi, DSI_VidCfgTypeDef *VidCfg)
{
  if (VidCfg->ColorCoding == DSI_RGB666)
  {
    ;//assert_param(IS_DSI_LOOSELY_PACKED(VidCfg->LooselyPacked));
  }

  /* Select video mode by resetting CMDM and DSIM bits */
  hdsi->Instance->MCR &= ~DSI_MCR_CMDM;
  hdsi->Instance->WCFGR &= ~DSI_WCFGR_DSIM;

  /* Configure the video mode transmission type */
  hdsi->Instance->VMCR &= ~DSI_VMCR_VMT;
  hdsi->Instance->VMCR |= VidCfg->Mode;

  /* Configure the video packet size */
  hdsi->Instance->VPCR &= ~DSI_VPCR_VPSIZE;
  hdsi->Instance->VPCR |= VidCfg->PacketSize;

  /* Set the chunks number to be transmitted through the DSI link */
  hdsi->Instance->VCCR &= ~DSI_VCCR_NUMC;
  hdsi->Instance->VCCR |= VidCfg->NumberOfChunks;

  /* Set the size of the null packet */
  hdsi->Instance->VNPCR &= ~DSI_VNPCR_NPSIZE;
  hdsi->Instance->VNPCR |= VidCfg->NullPacketSize;

  /* Select the virtual channel for the LTDC interface traffic */
  hdsi->Instance->LVCIDR &= ~DSI_LVCIDR_VCID;
  hdsi->Instance->LVCIDR |= VidCfg->VirtualChannelID;

  /* Configure the polarity of control signals */
  hdsi->Instance->LPCR &= ~(DSI_LPCR_DEP | DSI_LPCR_VSP | DSI_LPCR_HSP);
  hdsi->Instance->LPCR |= (VidCfg->DEPolarity | VidCfg->VSPolarity | VidCfg->HSPolarity);

  /* Select the color coding for the host */
  hdsi->Instance->LCOLCR &= ~DSI_LCOLCR_COLC;
  hdsi->Instance->LCOLCR |= VidCfg->ColorCoding;

  /* Select the color coding for the wrapper */
  hdsi->Instance->WCFGR &= ~DSI_WCFGR_COLMUX;
  hdsi->Instance->WCFGR |= ((VidCfg->ColorCoding) << 1U);

  /* Enable/disable the loosely packed variant to 18-bit configuration */
  if (VidCfg->ColorCoding == DSI_RGB666)
  {
    hdsi->Instance->LCOLCR &= ~DSI_LCOLCR_LPE;
    hdsi->Instance->LCOLCR |= VidCfg->LooselyPacked;
  }

  /* Set the Horizontal Synchronization Active (HSA) in lane byte clock cycles */
  hdsi->Instance->VHSACR &= ~DSI_VHSACR_HSA;
  hdsi->Instance->VHSACR |= VidCfg->HorizontalSyncActive;

  /* Set the Horizontal Back Porch (HBP) in lane byte clock cycles */
  hdsi->Instance->VHBPCR &= ~DSI_VHBPCR_HBP;
  hdsi->Instance->VHBPCR |= VidCfg->HorizontalBackPorch;

  /* Set the total line time (HLINE=HSA+HBP+HACT+HFP) in lane byte clock cycles */
  hdsi->Instance->VLCR &= ~DSI_VLCR_HLINE;
  hdsi->Instance->VLCR |= VidCfg->HorizontalLine;

  /* Set the Vertical Synchronization Active (VSA) */
  hdsi->Instance->VVSACR &= ~DSI_VVSACR_VSA;
  hdsi->Instance->VVSACR |= VidCfg->VerticalSyncActive;

  /* Set the Vertical Back Porch (VBP)*/
  hdsi->Instance->VVBPCR &= ~DSI_VVBPCR_VBP;
  hdsi->Instance->VVBPCR |= VidCfg->VerticalBackPorch;

  /* Set the Vertical Front Porch (VFP)*/
  hdsi->Instance->VVFPCR &= ~DSI_VVFPCR_VFP;
  hdsi->Instance->VVFPCR |= VidCfg->VerticalFrontPorch;

  /* Set the Vertical Active period*/
  hdsi->Instance->VVACR &= ~DSI_VVACR_VA;
  hdsi->Instance->VVACR |= VidCfg->VerticalActive;

  /* Configure the command transmission mode */
  hdsi->Instance->VMCR &= ~DSI_VMCR_LPCE;
  hdsi->Instance->VMCR |= VidCfg->LPCommandEnable;

  /* Low power largest packet size */
  hdsi->Instance->LPMCR &= ~DSI_LPMCR_LPSIZE;
  hdsi->Instance->LPMCR |= ((VidCfg->LPLargestPacketSize) << 16U);

  /* Low power VACT largest packet size */
  hdsi->Instance->LPMCR &= ~DSI_LPMCR_VLPSIZE;
  hdsi->Instance->LPMCR |= VidCfg->LPVACTLargestPacketSize;

  /* Enable LP transition in HFP period */
  hdsi->Instance->VMCR &= ~DSI_VMCR_LPHFPE;
  hdsi->Instance->VMCR |= VidCfg->LPHorizontalFrontPorchEnable;

  /* Enable LP transition in HBP period */
  hdsi->Instance->VMCR &= ~DSI_VMCR_LPHBPE;
  hdsi->Instance->VMCR |= VidCfg->LPHorizontalBackPorchEnable;

  /* Enable LP transition in VACT period */
  hdsi->Instance->VMCR &= ~DSI_VMCR_LPVAE;
  hdsi->Instance->VMCR |= VidCfg->LPVerticalActiveEnable;

  /* Enable LP transition in VFP period */
  hdsi->Instance->VMCR &= ~DSI_VMCR_LPVFPE;
  hdsi->Instance->VMCR |= VidCfg->LPVerticalFrontPorchEnable;

  /* Enable LP transition in VBP period */
  hdsi->Instance->VMCR &= ~DSI_VMCR_LPVBPE;
  hdsi->Instance->VMCR |= VidCfg->LPVerticalBackPorchEnable;

  /* Enable LP transition in vertical sync period */
  hdsi->Instance->VMCR &= ~DSI_VMCR_LPVSAE;
  hdsi->Instance->VMCR |= VidCfg->LPVerticalSyncActiveEnable;

  /* Enable the request for an acknowledge response at the end of a frame */
  hdsi->Instance->VMCR &= ~DSI_VMCR_FBTAAE;
  hdsi->Instance->VMCR |= VidCfg->FrameBTAAcknowledgeEnable;


  return HAL_OK;
}
void HAL_DSI_MspInit(DSI_HandleTypeDef *hdsi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdsi);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DSI_MspInit could be implemented in the user file
   */
}
HAL_StatusTypeDef HAL_DSI_Start(DSI_HandleTypeDef *hdsi)
{UNUSED(hdsi);
//  /* Process locked */
//  __HAL_LOCK(hdsi);

  /* Enable the DSI host */
  __HAL_DSI_ENABLE(DSI);

  /* Enable the DSI wrapper */
  __HAL_DSI_WRAPPER_ENABLE(DSI);

  /* Process unlocked */
//  __HAL_UNLOCK(hdsi);
//
  return HAL_OK;
}
HAL_StatusTypeDef HAL_DSI_Init(DSI_HandleTypeDef *hdsi, DSI_PLLInitTypeDef *PLLInit)
{
	  uint32_t tickstart;
	  uint32_t unitIntervalx4;
	  uint32_t tempIDF;

	  /* Check the DSI handle allocation */
	  if (hdsi == NULL)
	  {
	    return HAL_ERROR;
	  }

	  /* Check function parameters */
		#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
	  if (hdsi->State == HAL_DSI_STATE_RESET)
	  {
	    /* Reset the DSI callback to the legacy weak callbacks */
	    hdsi->TearingEffectCallback = HAL_DSI_TearingEffectCallback; /* Legacy weak TearingEffectCallback */
	    hdsi->EndOfRefreshCallback  = HAL_DSI_EndOfRefreshCallback;  /* Legacy weak EndOfRefreshCallback  */
	    hdsi->ErrorCallback         = HAL_DSI_ErrorCallback;         /* Legacy weak ErrorCallback         */

	    if (hdsi->MspInitCallback == NULL)
	    {
	      hdsi->MspInitCallback = HAL_DSI_MspInit;
	    }
	    /* Initialize the low level hardware */
	    hdsi->MspInitCallback(hdsi);
	  }
	#else
	  if (hdsi->State == HAL_DSI_STATE_RESET)
	  {
	    /* Initialize the low level hardware */
	    HAL_DSI_MspInit(hdsi);
	  }
	#endif /* USE_HAL_DSI_REGISTER_CALLBACKS */

	  /* Change DSI peripheral state */
	  hdsi->State = HAL_DSI_STATE_BUSY;

	  /**************** Turn on the regulator and enable the DSI PLL ****************/

	  /* Enable the regulator */
	  __HAL_DSI_REG_ENABLE(DSI);

//	  /* Get tick */
//	  tickstart = HAL_GetTick();
//
//	  /* Wait until the regulator is ready */
//	  while (__HAL_DSI_GET_FLAG(hdsi, DSI_FLAG_RRS) == 0U)
//	  {
//	    /* Check for the Timeout */
//	    if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
//	    {
//	      return HAL_TIMEOUT;
//	    }
//	  }
	  __HAL_DSI_REG_ENABLE(DSI);

	   /* Enable the regulator */
	  WAIT_FOR_DSI_FLAG(DSI->WISR & DSI_WISR_RRS);

	  /* Set the PLL division factors */
	  hdsi->Instance->WRPCR &= ~(DSI_WRPCR_PLL_NDIV | DSI_WRPCR_PLL_IDF | DSI_WRPCR_PLL_ODF);
	  hdsi->Instance->WRPCR |= (((PLLInit->PLLNDIV) << 2U) | ((PLLInit->PLLIDF) << 11U) | ((PLLInit->PLLODF) << 16U));

	  /* Enable the DSI PLL */
	  __HAL_DSI_PLL_ENABLE();

	  /* Get tick */
//	  tickstart = HAL_GetTick();
//
//	  /* Wait for the lock of the PLL */
//	  while (__HAL_DSI_GET_FLAG(hdsi, DSI_FLAG_PLLLS) == 0U)
//	  {
//	    /* Check for the Timeout */
//	    if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
//	    {
//	      return HAL_TIMEOUT;
//	    }
//	  }
	  WAIT_FOR_DSI_FLAG(DSI->WISR & DSI_FLAG_PLLLS);
	  /*************************** Set the PHY parameters ***************************/

	  /* D-PHY clock and digital enable*/
	  hdsi->Instance->PCTLR |= (DSI_PCTLR_CKE | DSI_PCTLR_DEN);

	  /* Clock lane configuration */
	  hdsi->Instance->CLCR &= ~(DSI_CLCR_DPCC | DSI_CLCR_ACR);
	  hdsi->Instance->CLCR |= (DSI_CLCR_DPCC | hdsi->Init.AutomaticClockLaneControl);

	  /* Configure the number of active data lanes */
	  hdsi->Instance->PCONFR &= ~DSI_PCONFR_NL;
	  hdsi->Instance->PCONFR |= hdsi->Init.NumberOfLanes;

	  /************************ Set the DSI clock parameters ************************/

	  /* Set the TX escape clock division factor */
	  hdsi->Instance->CCR &= ~DSI_CCR_TXECKDIV;
	  hdsi->Instance->CCR |= hdsi->Init.TXEscapeCkdiv;

	  /* Calculate the bit period in high-speed mode in unit of 0.25 ns (UIX4) */
	  /* The equation is : UIX4 = IntegerPart( (1000/F_PHY_Mhz) * 4 )          */
	  /* Where : F_PHY_Mhz = (NDIV * HSE_Mhz) / (IDF * ODF)                    */
	  tempIDF = (PLLInit->PLLIDF > 0U) ? PLLInit->PLLIDF : 1U;
	  unitIntervalx4 = (4000000U * tempIDF * ((1UL << (0x3U & PLLInit->PLLODF)))) / ((HSE_VALUE / 1000U) * PLLInit->PLLNDIV);

	  /* Set the bit period in high-speed mode */
	  hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_UIX4;
	  hdsi->Instance->WPCR[0U] |= unitIntervalx4;

	  /****************************** Error management *****************************/

	  /* Disable all error interrupts and reset the Error Mask */
	  hdsi->Instance->IER[0U] = 0U;
	  hdsi->Instance->IER[1U] = 0U;
	  hdsi->ErrorMsk = 0U;

	  /* Initialise the error code */
	  hdsi->ErrorCode = HAL_DSI_ERROR_NONE;

	  /* Initialize the DSI state*/
	  hdsi->State = HAL_DSI_STATE_READY;

	  return HAL_OK;
}

void BSP_LCD_MspInit(void)
{
	 __HAL_RCC_LTDC_CLK_ENABLE();
		  /** @brief Toggle Sw reset of LTDC IP */
		  __HAL_RCC_LTDC_FORCE_RESET();
		  __HAL_RCC_LTDC_RELEASE_RESET();
		  /** @brief Enable the DMA2D clock */
		  __HAL_RCC_DMA2D_CLK_ENABLE();
		  /** @brief Toggle Sw reset of DMA2D IP */
		  __HAL_RCC_DMA2D_FORCE_RESET();
		  __HAL_RCC_DMA2D_RELEASE_RESET();
		  /** @brief Enable DSI Host and wrapper clocks */
		  __HAL_RCC_DSI_CLK_ENABLE();
		  /** @brief Soft Reset the DSI Host and wrapper */
		  __HAL_RCC_DSI_FORCE_RESET();
		  __HAL_RCC_DSI_RELEASE_RESET();

		  //			  NVIC_SetPriority(SDRAM_DMAx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	////			  /** @brief NVIC configuration for LTDC interrupt that is now enabled */
	//			  nvicEnableVector(LTDC_IRQn, 3);//			  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
	//			  HAL_NVIC_EnableIRQ(LTDC_IRQn);
		  NVIC_SetPriority(LTDC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
		  /* Enable interrupt */
		  NVIC_EnableIRQ(LTDC_IRQn);

	//			  /** @brief NVIC configuration for DMA2D interrupt that is now enabled */
	//			  nvicEnableVector(DMA2D_IRQn, 3);//			  HAL_NVIC_SetPriority(DMA2D_IRQn, 3, 0);
	//			  HAL_NVIC_EnableIRQ(DMA2D_IRQn);
		  NVIC_SetPriority(DMA2D_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
		  /* Enable interrupt */
		  NVIC_EnableIRQ(DMA2D_IRQn);
}
int initDSI(orientation_t orientation){
	 BSP_LCD_Reset();
	//No need to initialize clocks, it was done in ChibiOS ...see mcuconf.h
//	see line 360 BSP_LCD_InitEx@stm32f769i_discovery_lcd.c
			//1-Make sure ur close is set at 200MHz
//			  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//			  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//			  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//			  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//			  RCC_OscInitStruct.PLL.PLLM = 25;
//			  RCC_OscInitStruct.PLL.PLLN = 400;
//			  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//			  RCC_OscInitStruct.PLL.PLLQ = 8;
//			  RCC_OscInitStruct.PLL.PLLR = 7;
			//2-#define STM32_LCDTFT_REQUIRED               TRUE
//			 3-BSP_LCD_Reset(void)
//			Star--->BSP_LCD_MspInit()@stm32f769i_discovery_lcd.c

	 __HAL_RCC_LTDC_CLK_ENABLE();
	  /** @brief Toggle Sw reset of LTDC IP */
	  __HAL_RCC_LTDC_FORCE_RESET();
	  __HAL_RCC_LTDC_RELEASE_RESET();
	  /** @brief Enable the DMA2D clock */
	  __HAL_RCC_DMA2D_CLK_ENABLE();
	  /** @brief Toggle Sw reset of DMA2D IP */
	  __HAL_RCC_DMA2D_FORCE_RESET();
	  __HAL_RCC_DMA2D_RELEASE_RESET();
	  /** @brief Enable DSI Host and wrapper clocks */
	  __HAL_RCC_DSI_CLK_ENABLE();
	  /** @brief Soft Reset the DSI Host and wrapper */
	  __HAL_RCC_DSI_FORCE_RESET();
	  __HAL_RCC_DSI_RELEASE_RESET();

	  //			  NVIC_SetPriority(SDRAM_DMAx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
////			  /** @brief NVIC configuration for LTDC interrupt that is now enabled */
//			  nvicEnableVector(LTDC_IRQn, 3);//			  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
//			  HAL_NVIC_EnableIRQ(LTDC_IRQn);
	  NVIC_SetPriority(LTDC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	  /* Enable interrupt */
	  NVIC_EnableIRQ(LTDC_IRQn);

//			  /** @brief NVIC configuration for DMA2D interrupt that is now enabled */
//			  nvicEnableVector(DMA2D_IRQn, 3);//			  HAL_NVIC_SetPriority(DMA2D_IRQn, 3, 0);
//			  HAL_NVIC_EnableIRQ(DMA2D_IRQn);
	  NVIC_SetPriority(DMA2D_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	  /* Enable interrupt */
	  NVIC_EnableIRQ(DMA2D_IRQn);

//			  /** @brief NVIC configuration for DSI interrupt that is now enabled */
//			  nvicEnableVector(DSI_IRQn, 3);//			  HAL_NVIC_SetPriority(DSI_IRQn, 3, 0);
//			  HAL_NVIC_EnableIRQ(DSI_IRQn);
	  NVIC_SetPriority(DSI_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	  /* Enable interrupt */
	  NVIC_EnableIRQ(DSI_IRQn);

//	Ends--->BSP_LCD_MspInit()@stm32f769i_discovery_lcd.c

	  //HAL_DSI_Init()--->Starts
	  uint32_t PLLNDIV    = 100;
	  uint32_t PLLIDF     = DSI_PLL_IN_DIV5;
	  uint32_t PLLODF     = DSI_PLL_OUT_DIV1;
	  /* Set number of Lanes */
	  uint32_t InitNumberOfLanes = DSI_TWO_DATA_LANES;
	  /************************ Set the DSI clock parameters ************************/
	  uint32_t laneByteClk_kHz = 62500; /* 500 MHz / 8 = 62.5 MHz = 62500 kHz */
	  /* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
	  uint32_t InitTXEscapeCkdiv = laneByteClk_kHz/15620;
	  /* Set the TX escape clock division factor */

	  ///Start----->HAL_DSI_DeInit(&(hdsi_discovery));
	  __HAL_DSI_WRAPPER_DISABLE(DSI);

	  /* Disable the DSI host */
	  __HAL_DSI_DISABLE(DSI);

	  /* D-PHY clock and digital disable */
	  DSI->PCTLR &= ~(DSI_PCTLR_CKE | DSI_PCTLR_DEN);

	  /* Turn off the DSI PLL */
	  __HAL_DSI_PLL_DISABLE(DSI);

	  /* Disable the regulator */
	  __HAL_DSI_REG_DISABLE(DSI);

	  ///ENDs----->HAL_DSI_DeInit(&(hdsi_discovery));

	  /* Turn off the DSI PLL */
	  __HAL_DSI_PLL_DISABLE(DSI);

	  /* Disable the regulator */
	  __HAL_DSI_REG_DISABLE(DSI);


	  //Start----> 4-HAL_DSI_Init() set up DSI clock and registers

	  __HAL_DSI_REG_ENABLE(DSI);

	   /* Enable the regulator */
	  WAIT_FOR_DSI_FLAG(DSI->WISR & DSI_WISR_RRS);


	  /* Set the PLL division factors */
	  DSI->WRPCR &= ~(DSI_WRPCR_PLL_NDIV | DSI_WRPCR_PLL_IDF | DSI_WRPCR_PLL_ODF);
	  DSI->WRPCR |= (((PLLNDIV) << 2U) | ((PLLIDF) << 11U) | ((PLLODF) << 16U));

	  __HAL_DSI_PLL_ENABLE();
	  WAIT_FOR_DSI_FLAG(DSI->WISR & DSI_FLAG_PLLLS);


    /*************************** Set the PHY parameters ***************************/
	  /* D-PHY clock and digital enable*/
	  DSI->PCTLR |= (DSI_PCTLR_CKE | DSI_PCTLR_DEN);
	  uint32_t AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
	  /* Clock lane configuration */
	  DSI->CLCR &= ~(DSI_CLCR_DPCC | DSI_CLCR_ACR);
	  DSI->CLCR |= (DSI_CLCR_DPCC | AutomaticClockLaneControl);


	  /* Configure the number of active data lanes */
	  DSI->PCONFR &= ~DSI_PCONFR_NL;
	  DSI->PCONFR |= InitNumberOfLanes;



	  DSI->CCR &= ~DSI_CCR_TXECKDIV;
	  DSI->CCR |= InitTXEscapeCkdiv;

	  /* Calculate the bit period in high-speed mode in unit of 0.25 ns (UIX4) */
	  /* The equation is : UIX4 = IntegerPart( (1000/F_PHY_Mhz) * 4 )          */
	  /* Where : F_PHY_Mhz = (NDIV * HSE_Mhz) / (IDF * ODF)                    */
	  uint32_t tempIDF = (PLLIDF > 0U) ? PLLIDF : 1U;
	  uint32_t unitIntervalx4 = (4000000U * tempIDF * ((1UL << (0x3U & PLLODF)))) / ((STM32_HSECLK  / 1000U) * PLLNDIV);

	  /* Set the bit period in high-speed mode */
	  DSI->WPCR[0U] &= ~DSI_WPCR0_UIX4;
	  DSI->WPCR[0U] |= unitIntervalx4;
	  /* Disable all error interrupts and reset the Error Mask */
	  DSI->IER[0U] = 0U;
	  DSI->IER[1U] = 0U;

	  //HAL_DSI_Init()-->Ends


      //5-See HAL_DSI_ConfigVideoMode() in stm32f7xx_hal_dsi.c

//			HAL_DSI_Init@stm32f7xx_hal_dsi.c
			  /* Get LTDC Configuration from DSI Configuration */
//			  HAL_LTDC_StructInitFromVideoConfig(&(hltdc_discovery), &(hdsivideo_handle));
	  /* Initialize the LTDC */
	  uint32_t lcd_x_size = OTM8009A_800X480_WIDTH;
	  uint32_t lcd_y_size = OTM8009A_800X480_HEIGHT;
	  if(orientation == GDISP_ROTATE_PORTRAIT)
	  {
	    lcd_x_size = OTM8009A_480X800_WIDTH;  /* 480 */
	    lcd_y_size = OTM8009A_480X800_HEIGHT; /* 800 */
	  }
	  else
	  {
	    lcd_x_size = OTM8009A_800X480_WIDTH;  /* 800 */
	    lcd_y_size = OTM8009A_800X480_HEIGHT; /* 480 */
	  }

	  uint32_t HACT = lcd_x_size;
	  uint32_t VACT = lcd_y_size;
	  uint32_t LcdClock  = 27429; /*!< LcdClk = 27429 kHz */
	  /* The following values are same for portrait and landscape orientations */
	  uint32_t VSA  = OTM8009A_480X800_VSYNC;        /* 12  */
	  uint32_t VBP  = OTM8009A_480X800_VBP;          /* 12  */
	  uint32_t VFP  = OTM8009A_480X800_VFP;          /* 12  */
	  uint32_t HSA  = OTM8009A_480X800_HSYNC;        /* 63  */
	  uint32_t HBP  = OTM8009A_480X800_HBP;          /* 120 */
	  uint32_t HFP  = OTM8009A_480X800_HFP;          /* 120 */

	  uint32_t VirtualChannelID = LCD_OTM8009A_ID;
	  uint32_t ColorCoding = LCD_DSI_PIXEL_DATA_FMT_RBG888;
	  uint32_t VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
	  uint32_t HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
	  uint32_t DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
	  uint32_t Mode = DSI_VID_MODE_BURST; /* Mode Video burst ie : one LgP per line */
	  uint32_t NullPacketSize = 0xFFF;
	  uint32_t NumberOfChunks = 0;
	  uint32_t PacketSize                = HACT; /* Value depending on display orientation choice portrait/landscape */
	  uint32_t HorizontalSyncActive      = (HSA * laneByteClk_kHz)/LcdClock;
	  uint32_t HorizontalBackPorch       = (HBP * laneByteClk_kHz)/LcdClock;
	  uint32_t HorizontalLine            = ((HACT + HSA + HBP + HFP) * laneByteClk_kHz)/LcdClock; /* Value depending on display orientation choice portrait/landscape */
	  uint32_t VerticalSyncActive        = VSA;
	  uint32_t VerticalBackPorch         = VBP;
	  uint32_t VerticalFrontPorch        = VFP;
	  uint32_t VerticalActive            = VACT; /* Value depending on display orientation choice portrait/landscape */
	  /* Enable or disable sending LP command while streaming is active in video mode */
	  uint32_t LPCommandEnable = DSI_LP_COMMAND_ENABLE; /* Enable sending commands in mode LP (Low Power) */

	  /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP regions */
	  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
	  uint32_t LPLargestPacketSize = 64;

	  /* Largest packet size possible to transmit in LP mode in HFP region during VACT period */
	  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
	  uint32_t LPVACTLargestPacketSize = 64;

	  /* while streaming is active in video mode                                                                         */
	  uint32_t LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;   /* Allow sending LP commands during HFP period */
	  uint32_t LPHorizontalBackPorchEnable  = DSI_LP_HBP_ENABLE;   /* Allow sending LP commands during HBP period */
	  uint32_t LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;  /* Allow sending LP commands during VACT period */
	  uint32_t LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;   /* Allow sending LP commands during VFP period */
	  uint32_t LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;   /* Allow sending LP commands during VBP period */
	  uint32_t LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE; /* Allow sending LP commands during VSync = VSA period */
	  uint32_t FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;
//			  if(orientation == LCD_ORIENTATION_PORTRAIT)
			//6-Now you need to see LCD TFT clock to 27MHz
			  /* Timing Configuration */

	  uint32_t  Init_HorizontalSync = (HSA - 1);
	  uint32_t  Init_AccumulatedHBP = (HSA + HBP - 1);
	  uint32_t  Init_AccumulatedActiveW = (lcd_x_size + HSA + HBP - 1);
	  uint32_t  Init_TotalWidth = (lcd_x_size + HSA + HBP + HFP - 1);


	  //HAL_DSI_ConfigVideoMode--->Starts
	  /* Initialize the LCD pixel width and pixel height */
	  /* Select video mode by resetting CMDM and DSIM bits */
	  DSI->MCR &= ~DSI_MCR_CMDM;
	  DSI->WCFGR &= ~DSI_WCFGR_DSIM;

	  /* Configure the video mode transmission type */
	  DSI->VMCR &= ~DSI_VMCR_VMT;
	  DSI->VMCR |= Mode;

	  /* Configure the video packet size */
	  DSI->VPCR &= ~DSI_VPCR_VPSIZE;
	  DSI->VPCR |= PacketSize;

	  /* Set the chunks number to be transmitted through the DSI link */
	  DSI->VCCR &= ~DSI_VCCR_NUMC;
	  DSI->VCCR |= NumberOfChunks;

	  /* Set the size of the null packet */
	  DSI->VNPCR &= ~DSI_VNPCR_NPSIZE;
	  DSI->VNPCR |= NullPacketSize;

	  /* Select the virtual channel for the LTDC interface traffic */
	  DSI->LVCIDR &= ~DSI_LVCIDR_VCID;
	  DSI->LVCIDR |= VirtualChannelID;

	  /* Configure the polarity of control signals */
	  DSI->LPCR &= ~(DSI_LPCR_DEP | DSI_LPCR_VSP | DSI_LPCR_HSP);
	  DSI->LPCR |= (DEPolarity | VSPolarity | HSPolarity);

	  /* Select the color coding for the host */
	  DSI->LCOLCR &= ~DSI_LCOLCR_COLC;
	  DSI->LCOLCR |= ColorCoding;

	  /* Select the color coding for the wrapper */
	  DSI->WCFGR &= ~DSI_WCFGR_COLMUX;
	  DSI->WCFGR |= (ColorCoding << 1U);
	  uint32_t LooselyPacked        = DSI_LOOSELY_PACKED_DISABLE;
	  /* Enable/disable the loosely packed variant to 18-bit configuration */
	  if (ColorCoding == DSI_RGB666)
	  {
		DSI->LCOLCR &= ~DSI_LCOLCR_LPE;
		DSI->LCOLCR |= LooselyPacked;
	  }

	  /* Set the Horizontal Synchronization Active (HSA) in lane byte clock cycles */
	  DSI->VHSACR &= ~DSI_VHSACR_HSA;
	  DSI->VHSACR |= HorizontalSyncActive;

	  /* Set the Horizontal Back Porch (HBP) in lane byte clock cycles */
	  DSI->VHBPCR &= ~DSI_VHBPCR_HBP;
	  DSI->VHBPCR |= HorizontalBackPorch;

	  /* Set the total line time (HLINE=HSA+HBP+HACT+HFP) in lane byte clock cycles */
	  DSI->VLCR &= ~DSI_VLCR_HLINE;
	  DSI->VLCR |= HorizontalLine;


	  /* Set the Vertical Synchronization Active (VSA) */
	  DSI->VVSACR &= ~DSI_VVSACR_VSA;
	  DSI->VVSACR |= VerticalSyncActive;

	  /* Set the Vertical Back Porch (VBP)*/
	  DSI->VVBPCR &= ~DSI_VVBPCR_VBP;
	  DSI->VVBPCR |= VerticalBackPorch;

	  /* Set the Vertical Front Porch (VFP)*/
	  DSI->VVFPCR &= ~DSI_VVFPCR_VFP;
	  DSI->VVFPCR |= VerticalFrontPorch;

	  /* Set the Vertical Active period*/
	  DSI->VVACR &= ~DSI_VVACR_VA;
	  DSI->VVACR |= VerticalActive;

	  /* Configure the command transmission mode */
	  DSI->VMCR &= ~DSI_VMCR_LPCE;
	  DSI->VMCR |= LPCommandEnable;

	  /* Low power largest packet size */
	  DSI->LPMCR &= ~DSI_LPMCR_LPSIZE;
	  DSI->LPMCR |= ((LPLargestPacketSize) << 16U);

	  /* Low power VACT largest packet size */
	  DSI->LPMCR &= ~DSI_LPMCR_VLPSIZE;
	  DSI->LPMCR |= LPVACTLargestPacketSize;

	  /* Enable LP transition in HFP period */
	  DSI->VMCR &= ~DSI_VMCR_LPHFPE;
	  DSI->VMCR |= LPHorizontalFrontPorchEnable;

	  /* Enable LP transition in HBP period */
	  DSI->VMCR &= ~DSI_VMCR_LPHBPE;
	  DSI->VMCR |= LPHorizontalBackPorchEnable;

	  /* Enable LP transition in VACT period */
	  DSI->VMCR &= ~DSI_VMCR_LPVAE;
	  DSI->VMCR |= LPVerticalActiveEnable;

	  /* Enable LP transition in VFP period */
	  DSI->VMCR &= ~DSI_VMCR_LPVFPE;
	  DSI->VMCR |= LPVerticalFrontPorchEnable;

	  /* Enable LP transition in VBP period */
	  DSI->VMCR &= ~DSI_VMCR_LPVBPE;
	  DSI->VMCR |= LPVerticalBackPorchEnable;

	  /* Enable LP transition in vertical sync period */
	  DSI->VMCR &= ~DSI_VMCR_LPVSAE;
	  DSI->VMCR |= LPVerticalSyncActiveEnable;


	  /* Enable the request for an acknowledge response at the end of a frame */
	  DSI->VMCR &= ~DSI_VMCR_FBTAAE;
	  DSI->VMCR |= FrameBTAAcknowledgeEnable;
	  //Ends---->HAL_DSI_ConfigVideoMode--->

	  //HAL_RCCEx_PeriphCLKConfig Already done in chibiOS

	/*
	 * LCD clock configuration has been already configured in ChibiOS
	 * see mcuconf.h for detail
	 *
	 */
	  /** LCD clock configuration:
			    * Note: The following values should not be changed as the PLLSAI is also used
			    *      to clock the USB FS
			    * PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz
			    * PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 384 Mhz
			    * PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 384 MHz / 7 = 54.85 MHz
			    * LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 54.85 MHz / 2 = 27.429 MHz

			  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
			  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
			  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
			  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
			  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
	   **/

	  //		HAL_StatusTypeDef HAL_LTDCEx_StructInitFromVideoConfig(LTDC_HandleTypeDef *hltdc, DSI_VidCfgTypeDef *VidCfg)
	  //		{
	  //		  /* Retrieve signal polarities from DSI */
	  //
	  //		  /* The following polarity is inverted:
	  //							 LTDC_DEPOLARITY_AL <-> LTDC_DEPOLARITY_AH */
	  //
	  //		  /* Note 1 : Code in line w/ Current LTDC specification */
	  			  uint32_t Init_DEPolarity = (DEPolarity == DSI_DATA_ENABLE_ACTIVE_HIGH) ? LTDC_DEPOLARITY_AL : LTDC_DEPOLARITY_AH;
	  			  uint32_t Init_VSPolarity = (VSPolarity == DSI_VSYNC_ACTIVE_HIGH) ? LTDC_VSPOLARITY_AH : LTDC_VSPOLARITY_AL;
	  			  uint32_t Init_HSPolarity = (HSPolarity == DSI_HSYNC_ACTIVE_HIGH) ? LTDC_HSPOLARITY_AH : LTDC_HSPOLARITY_AL;
	  			  ////
	  			  ////			  /* Note 2: Code to be used in case LTDC polarities inversion updated in the specification */
	  			  //			  int32_t Init_DEPolarity = DEPolarity << 29;
	  			  //			  int32_t Init_VSPolarity = VSPolarity << 29;
	  			  //			  int32_t Init_HSPolarity = HSPolarity << 29; */
	  			  //
	  			  //			  /* Retrieve vertical timing parameters from DSI */
	  			  int32_t Init_VerticalSync       = VerticalSyncActive - 1U;
	  			  int32_t Init_AccumulatedVBP     = VerticalSyncActive + VerticalBackPorch - 1U;
	  			  int32_t Init_AccumulatedActiveH = VerticalSyncActive + VerticalBackPorch + VerticalActive - 1U;
	  			  int32_t Init_TotalHeigh         = VerticalSyncActive + VerticalBackPorch + VerticalActive + VerticalFrontPorch - 1U;
	  			  uint8_t Init_Backcolor_Blue = 0;
	  			  uint8_t Init_Backcolor_Green = 0;
	  			  uint8_t Init_Backcolor_Red = 0;
	  			  int32_t Init_PCPolarity = LTDC_PCPOLARITY_IPC;
	  			  uint32_t  rgGCR = (uint32_t)(Init_HSPolarity | Init_VSPolarity | Init_DEPolarity | Init_PCPolarity);
	  			Init_Backcolor_Red = rgGCR;
#if 0 //def INCLUDES_ALL
			  ///HAL_LTDC_Init---->Starts

			  /* Configure the HS, VS, DE and PC polarity */
			  LTDC->GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
			  LTDC->GCR |= (uint32_t)(Init_HSPolarity | Init_VSPolarity | Init_DEPolarity | Init_PCPolarity);

			  /* Set Synchronization size */
			  LTDC->SSCR &= ~(LTDC_SSCR_VSH | LTDC_SSCR_HSW);
			  uint32_t tmp = (Init_HorizontalSync << 16U);
			  LTDC->SSCR |= (tmp | Init_VerticalSync);

			  /* Set Accumulated Back porch */
			  LTDC->BPCR &= ~(LTDC_BPCR_AVBP | LTDC_BPCR_AHBP);
			  tmp = (Init_AccumulatedHBP << 16U);
			  LTDC->BPCR |= (tmp | Init_AccumulatedVBP);

			  /* Set Accumulated Active Width */
			  LTDC->AWCR &= ~(LTDC_AWCR_AAH | LTDC_AWCR_AAW);
			  tmp = (Init_AccumulatedActiveW << 16U);
			  LTDC->AWCR |= (tmp | Init_AccumulatedActiveH);

			  /* Set Total Width */
			  LTDC->TWCR &= ~(LTDC_TWCR_TOTALH | LTDC_TWCR_TOTALW);
			  tmp = (Init_TotalWidth << 16U);
			  LTDC->TWCR |= (tmp | Init_TotalHeigh);

			  /* Set the background color value */
			  tmp = ((uint32_t)(Init_Backcolor_Green) << 8U);
			  uint32_t tmp1 = ((uint32_t)(Init_Backcolor_Red) << 16U);
			  LTDC->BCCR &= ~(LTDC_BCCR_BCBLUE | LTDC_BCCR_BCGREEN | LTDC_BCCR_BCRED);
			  LTDC->BCCR |= (tmp1 | tmp | Init_Backcolor_Blue);

			  /* Enable the Transfer Error and FIFO underrun interrupts */
			  __HAL_LTDC_ENABLE_IT(LTDC, LTDC_IT_TE | LTDC_IT_FU);

			  /* Enable LTDC by setting LTDCEN bit */
			  __HAL_LTDC_ENABLE(LTDC);
#endif
			  ///HAL_LTDC_Init---->Ends

			  //HAL_DSI_Start--->Start
			   __HAL_DSI_ENABLE(DSI);
			   __HAL_DSI_WRAPPER_ENABLE(DSI);
			   //HAL_DSI_Start--->Ends

#if 0//def INCLUDES_ALL
			   BSP_SDRAM_Init();
				  /* Initialize the font */
			//	BSP_LCD_SetFont(&LCD_DEFAULT_FONT);


				OTM8009A_Init(OTM8009A_FORMAT_RGB888, orientation);
				//
			//    BSP_TS_Init(800, 480);

//				BSP_LCD_DisplayOn();


			  ///Starts:BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);-->Starts
#define LTDC_LAYER(__HANDLE__, __LAYER__)              ((LTDC_Layer_TypeDef *)((uint32_t)(((uint32_t)((__HANDLE__))) + 0x84U + (0x80U*(__LAYER__)))))

			  uint32_t  Layercfg_WindowX0 = 0;
			  uint32_t  Layercfg_WindowX1 = lcd_x_size;
			  uint32_t  Layercfg_WindowY0 = 0;
			  uint32_t  Layercfg_WindowY1 = lcd_y_size;
			  //uint32_t  Layercfg_PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
			  uint32_t  Layercfg_PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
			  uint32_t  Layercfg_FBStartAdress = (uint32_t)LCD_FB_START_ADDRESS;
			  uint32_t  Layercfg_Alpha = 255;
			  uint32_t  Layercfg_Alpha0 = 0;
			  uint8_t   Layercfg_Backcolor_Blue = 0;
			  uint8_t   Layercfg_Backcolor_Green = 0;
			  uint8_t   Layercfg_Backcolor_Red = 0;
			  uint32_t  Layercfg_BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
			  uint32_t  Layercfg_BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
			  uint32_t  Layercfg_ImageWidth =  lcd_x_size;
			  uint32_t  Layercfg_ImageHeight = lcd_y_size;
			  uint8_t LayerIdx = 1;
			  /* Configure the horizontal start and stop position */
			  tmp = ((Layercfg_WindowX1 + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U)) << 16U);
			  LTDC_LAYER(LTDC, LayerIdx)->WHPCR &= ~(LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS);
			  LTDC_LAYER(LTDC, LayerIdx)->WHPCR = ((Layercfg_WindowX0 + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U) + 1U) | tmp);

			  /* Configure the vertical start and stop position */
			  tmp = ((Layercfg_WindowY1 + (LTDC->BPCR & LTDC_BPCR_AVBP)) << 16U);
			  LTDC_LAYER(LTDC, LayerIdx)->WVPCR &= ~(LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS);
			  LTDC_LAYER(LTDC, LayerIdx)->WVPCR  = ((Layercfg_WindowY0 + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1U) | tmp);

			  /* Specifies the pixel format */
			  LTDC_LAYER(LTDC, LayerIdx)->PFCR &= ~(LTDC_LxPFCR_PF);
			  LTDC_LAYER(LTDC, LayerIdx)->PFCR = (Layercfg_PixelFormat);

			  /* Configure the default color values */
			  tmp = ((uint32_t)(Layercfg_Backcolor_Green) << 8U);
			  tmp1 = ((uint32_t)(Layercfg_Backcolor_Red) << 16U);
			  uint32_t tmp2 = (Layercfg_Alpha0 << 24U);
			  LTDC_LAYER(LTDC, LayerIdx)->DCCR &= ~(LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA);
			  LTDC_LAYER(LTDC, LayerIdx)->DCCR = (Layercfg_Backcolor_Blue | tmp | tmp1 | tmp2);

			  /* Specifies the constant alpha value */
			  LTDC_LAYER(LTDC, LayerIdx)->CACR &= ~(LTDC_LxCACR_CONSTA);
			  LTDC_LAYER(LTDC, LayerIdx)->CACR = (Layercfg_Alpha);

			  /* Specifies the blending factors */
			  LTDC_LAYER(LTDC, LayerIdx)->BFCR &= ~(LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1);
			  LTDC_LAYER(LTDC, LayerIdx)->BFCR = (Layercfg_BlendingFactor1 | Layercfg_BlendingFactor2);

			  /* Configure the color frame buffer start address */
			  LTDC_LAYER(LTDC, LayerIdx)->CFBAR &= ~(LTDC_LxCFBAR_CFBADD);
			  LTDC_LAYER(LTDC, LayerIdx)->CFBAR = (Layercfg_FBStartAdress);

			  if (Layercfg_PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
			  {
			    tmp = 4U;
			  }
			  else if (Layercfg_PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
			  {
			    tmp = 3U;
			  }
			  else if ((Layercfg_PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
			           (Layercfg_PixelFormat == LTDC_PIXEL_FORMAT_RGB565)   || \
			           (Layercfg_PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
			           (Layercfg_PixelFormat == LTDC_PIXEL_FORMAT_AL88))
			  {
			    tmp = 2U;
			  }
			  else
			  {
			    tmp = 1U;
			  }

			  /* Configure the color frame buffer pitch in byte */
			  LTDC_LAYER(LTDC, LayerIdx)->CFBLR  &= ~(LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP);
			  LTDC_LAYER(LTDC, LayerIdx)->CFBLR  = (((Layercfg_ImageWidth * tmp) << 16U) | (((Layercfg_WindowX1 - Layercfg_WindowX0) * tmp)  + 3U));
			  /* Configure the frame buffer line number */
			  LTDC_LAYER(LTDC, LayerIdx)->CFBLNR  &= ~(LTDC_LxCFBLNR_CFBLNBR);
			  LTDC_LAYER(LTDC, LayerIdx)->CFBLNR  = (Layercfg_ImageHeight);

			  /* Enable LTDC_Layer by setting LEN bit */
			  LTDC_LAYER(LTDC, LayerIdx)->CR |= (uint32_t)LTDC_LxCR_LEN;


			  /* Set the Immediate Reload type */
			  LTDC->SRCR = LTDC_SRCR_IMR;
			  BSP_LCD_DisplayOn();
#endif
			  ///			  ///ENDS:BSP_LCD_LayerDefaultInit


	return 0;
}
void BSP_LCD_DisplayOn2(void){
    DSI_ShortWrite(OTM8009A_ORIENTATION_PORTRAIT,
                       DSI_DCS_SHORT_PKT_WRITE_P1,
                       OTM8009A_CMD_DISPON,
                       0x00);
}

int postInitDSI(orientation_t orientation){
#if 1//ndef INCLUDES_ALL
	BSP_SDRAM_Init();
	OTM8009A_Init(OTM8009A_FORMAT_RGB888, orientation==GDISP_ROTATE_PORTRAIT?OTM8009A_ORIENTATION_PORTRAIT:OTM8009A_ORIENTATION_LANDSCAPE);
//	BSP_LCD_DisplayOn();
#endif
	//9-HAL_DSI_Start(&hdsi_discovery);

//	  /* Initialize the font */
////	BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
//
//
//	OTM8009A_Init(OTM8009A_FORMAT_RGB888, OTM8009A_ORIENTATION_PORTRAIT);
//	//
////    BSP_TS_Init(800, 480);
//
//	BSP_LCD_DisplayOn();
	return 0;
}
uint8_t DSI_ShortWrite(uint32_t ChannelID, uint32_t Mode, uint32_t Param1, uint32_t Param2){
  WAIT_FOR_DSI_FLAG(DSI->GPSR & DSI_GPSR_CMDFE)
  /* Configure the packet to send a short DCS command with 0 or 1 parameter */
  /* Update the DSI packet header with new information */
  DSI->GHCR = (Mode | (ChannelID << 6U) | (Param1 << 8U) | (Param2 << 16U));

  return 0;
}
static void DSI_ConfigPacketHeader(DSI_TypeDef *DSIx,
                                   uint32_t ChannelID,
                                   uint32_t DataType,
                                   uint32_t Data0,
                                   uint32_t Data1)
{
  /* Update the DSI packet header with new information */
  DSIx->GHCR = (DataType | (ChannelID << 6U) | (Data0 << 8U) | (Data1 << 16U));
}
uint8_t DSI_LongWrite(uint32_t ChannelID, uint32_t Mode, uint32_t NbParams,  uint32_t Param1, uint8_t *ParametersTable){
  WAIT_FOR_DSI_FLAG(DSI->GPSR & DSI_GPSR_CMDFE)
  uint32_t uicounter, nbBytes, count;
  uint32_t tickstart;
  uint32_t fifoword;
  uint8_t *pparams = ParametersTable;

  /* Set the DCS code on payload byte 1, and the other parameters on the write FIFO command*/
   fifoword = Param1;
   nbBytes = (NbParams < 3U) ? NbParams : 3U;

   for (count = 0U; count < nbBytes; count++)
   {
     fifoword |= (((uint32_t)(*(pparams + count))) << (8U + (8U * count)));
   }
   DSI->GPDR = fifoword;

   uicounter = NbParams - nbBytes;
   pparams += nbBytes;
   /* Set the Next parameters on the write FIFO command*/
   while (uicounter != 0U)
   {
     nbBytes = (uicounter < 4U) ? uicounter : 4U;
     fifoword = 0U;
     for (count = 0U; count < nbBytes; count++)
     {
       fifoword |= (((uint32_t)(*(pparams + count))) << (8U * count));
     }
     DSI->GPDR = fifoword;

     uicounter -= nbBytes;
     pparams += nbBytes;
   }

   /* Configure the packet to send a long DCS command */
   DSI_ConfigPacketHeader(DSI,
                          ChannelID,
                          Mode,
                          ((NbParams + 1U) & 0x00FFU),
                          (((NbParams + 1U) & 0xFF00U) >> 8U));


   return 0;
}
void setBacklightPWM(uint8_t percent) {
   DSI_ShortWrite(LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_WRDISBV, (uint16_t)(percent * 255)/100);
}
void OTM8009A_IO_Delay(uint32_t Delay)
{
	chThdSleepMilliseconds(Delay);
}
void HAL_Delay(uint32_t Delay)
{
	chThdSleepMilliseconds(Delay);
}

uint32_t timeI2MS(uint32_t interval){
	return TIME_I2MS(interval);
}

void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams)
{
	uint8_t rc = 0;
  if(NbrParams <= 1)
  {
     rc = DSI_ShortWrite(LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]);
  }
  else
  {
    rc = DSI_LongWrite(LCD_OTM8009A_ID, DSI_DCS_LONG_PKT_WRITE, NbrParams, pParams[NbrParams], pParams);
  }

  if ( rc )
	  rc = 0;
  else
	  rc = 1;
}
typedef struct
{
  uint32_t             Mode;               /*!< Configures the DMA2D transfer mode.
                                                This parameter can be one value of @ref DMA2D_Mode. */

  uint32_t             ColorMode;          /*!< Configures the color format of the output image.
                                                This parameter can be one value of @ref DMA2D_Output_Color_Mode. */

  uint32_t             OutputOffset;       /*!< Specifies the Offset value.
                                                This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF. */
#if defined (DMA2D_ALPHA_INV_RB_SWAP_SUPPORT)
  uint32_t             AlphaInverted;     /*!< Select regular or inverted alpha value for the output pixel format converter.
                                               This parameter can be one value of @ref DMA2D_Alpha_Inverted. */

  uint32_t             RedBlueSwap;       /*!< Select regular mode (RGB or ARGB) or swap mode (BGR or ABGR)
                                               for the output pixel format converter.
                                               This parameter can be one value of @ref DMA2D_RB_Swap. */

#endif /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */




} DMA2D_InitTypeDef;
typedef struct
{
  uint32_t             InputOffset;       /*!< Configures the DMA2D foreground or background offset.
                                               This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF. */

  uint32_t             InputColorMode;    /*!< Configures the DMA2D foreground or background color mode.
                                               This parameter can be one value of @ref DMA2D_Input_Color_Mode. */

  uint32_t             AlphaMode;         /*!< Configures the DMA2D foreground or background alpha mode.
                                               This parameter can be one value of @ref DMA2D_Alpha_Mode. */

  uint32_t             InputAlpha;        /*!< Specifies the DMA2D foreground or background alpha value and color value in case of A8 or A4 color mode.
                                               This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF except for the color modes detailed below.
                                               @note In case of A8 or A4 color mode (ARGB), this parameter must be a number between
                                               Min_Data = 0x00000000 and Max_Data = 0xFFFFFFFF where
                                               - InputAlpha[24:31] is the alpha value ALPHA[0:7]
                                               - InputAlpha[16:23] is the red value RED[0:7]
                                               - InputAlpha[8:15] is the green value GREEN[0:7]
                                               - InputAlpha[0:7] is the blue value BLUE[0:7]. */
#if defined (DMA2D_ALPHA_INV_RB_SWAP_SUPPORT)
  uint32_t             AlphaInverted;     /*!< Select regular or inverted alpha value.
                                               This parameter can be one value of @ref DMA2D_Alpha_Inverted. */

  uint32_t             RedBlueSwap;       /*!< Select regular mode (RGB or ARGB) or swap mode (BGR or ABGR).
                                               This parameter can be one value of @ref DMA2D_RB_Swap. */
#endif /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT  */


} DMA2D_LayerCfgTypeDef;


#define MAX_DMA2D_LAYER  2U


typedef struct __DMA2D_HandleTypeDef
{
  DMA2D_TypeDef               *Instance;                                                    /*!< DMA2D register base address.               */

  DMA2D_InitTypeDef           Init;                                                         /*!< DMA2D communication parameters.            */

  DMA2D_LayerCfgTypeDef       LayerCfg[MAX_DMA2D_LAYER];                                    /*!< DMA2D Layers parameters           */

  __IO uint32_t               ErrorCode;                                                    /*!< DMA2D error code.                          */
} DMA2D_HandleTypeDef;
#define DMA2D_R2M                   DMA2D_CR_MODE            /*!< DMA2D register to memory transfer mode */
#define DMA2D_OUTPUT_ARGB8888       0x00000000U                           /*!< ARGB8888 DMA2D color mode */
#define HAL_OK 						0
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define DMA2D_INPUT_A8              0x00000009U  /*!< A8 color mode       */
#define DMA2D_INPUT_A4              0x0000000AU  /*!< A4 color mode       */
#define DMA2D_M2M                   0x00000000U                         /*!< DMA2D memory to memory transfer mode */
#define DMA2D_BACKGROUND_LAYER             0x00000000U   /*!< DMA2D Background Layer (layer 0) */
#define DMA2D_FOREGROUND_LAYER             0x00000001U   /*!< DMA2D Foreground Layer (layer 1) */
#define DMA2D_OUTPUT_ARGB8888       0x00000000U                           /*!< ARGB8888 DMA2D color mode */
#define DMA2D_OUTPUT_RGB888         DMA2D_OPFCCR_CM_0                     /*!< RGB888 DMA2D color mode   */
#define DMA2D_OUTPUT_RGB565         DMA2D_OPFCCR_CM_1                     /*!< RGB565 DMA2D color mode   */
#define DMA2D_OUTPUT_ARGB1555       (DMA2D_OPFCCR_CM_0|DMA2D_OPFCCR_CM_1) /*!< ARGB1555 DMA2D color mode */
#define DMA2D_OUTPUT_ARGB4444       DMA2D_OPFCCR_CM_2                     /*!< ARGB4444 DMA2D color mode */



uint8_t HAL_DMA2D_Init(DMA2D_HandleTypeDef *hdma2d)
{
	 MODIFY_REG(hdma2d->Instance->CR, DMA2D_CR_MODE, hdma2d->Init.Mode);

	  /* DMA2D OPFCCR register configuration ---------------------------------------*/
	  MODIFY_REG(hdma2d->Instance->OPFCCR, DMA2D_OPFCCR_CM, hdma2d->Init.ColorMode);

	  /* DMA2D OOR register configuration ------------------------------------------*/
	  MODIFY_REG(hdma2d->Instance->OOR, DMA2D_OOR_LO, hdma2d->Init.OutputOffset);
	#if defined (DMA2D_ALPHA_INV_RB_SWAP_SUPPORT)
	  /* DMA2D OPFCCR AI and RBS fields setting (Output Alpha Inversion)*/
	  MODIFY_REG(hdma2d->Instance->OPFCCR,(DMA2D_OPFCCR_AI|DMA2D_OPFCCR_RBS), ((hdma2d->Init.AlphaInverted << DMA2D_OPFCCR_AI_Pos) | (hdma2d->Init.RedBlueSwap << DMA2D_OPFCCR_RBS_Pos)));
	#endif /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */

	return HAL_OK;
}
uint8_t HAL_DMA2D_ConfigLayer(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx)
{
	  DMA2D_LayerCfgTypeDef *pLayerCfg;
	  uint32_t regMask, regValue;

	  if(hdma2d->Init.Mode != DMA2D_R2M)
	  {
	  }
	  pLayerCfg = &hdma2d->LayerCfg[LayerIdx];

	  /* Prepare the value to be written to the BGPFCCR or FGPFCCR register */
	#if defined (DMA2D_ALPHA_INV_RB_SWAP_SUPPORT)
	  regValue = pLayerCfg->InputColorMode | (pLayerCfg->AlphaMode << DMA2D_BGPFCCR_AM_Pos) |\
	             (pLayerCfg->AlphaInverted << DMA2D_BGPFCCR_AI_Pos) | (pLayerCfg->RedBlueSwap << DMA2D_BGPFCCR_RBS_Pos);
	  regMask  = (DMA2D_BGPFCCR_CM | DMA2D_BGPFCCR_AM | DMA2D_BGPFCCR_ALPHA | DMA2D_BGPFCCR_AI | DMA2D_BGPFCCR_RBS);
	#else
	  regValue = pLayerCfg->InputColorMode | (pLayerCfg->AlphaMode << DMA2D_BGPFCCR_AM_Pos);
	  regMask  = DMA2D_BGPFCCR_CM | DMA2D_BGPFCCR_AM | DMA2D_BGPFCCR_ALPHA;
	#endif /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */


	  if ((pLayerCfg->InputColorMode == DMA2D_INPUT_A4) || (pLayerCfg->InputColorMode == DMA2D_INPUT_A8))
	  {
	    regValue |= (pLayerCfg->InputAlpha & DMA2D_BGPFCCR_ALPHA);
	  }
	  else
	  {
	    regValue |=  (pLayerCfg->InputAlpha << DMA2D_BGPFCCR_ALPHA_Pos);
	  }

	  /* Configure the background DMA2D layer */
	  if(LayerIdx == DMA2D_BACKGROUND_LAYER)
	  {
	    /* Write DMA2D BGPFCCR register */
	    MODIFY_REG(hdma2d->Instance->BGPFCCR, regMask, regValue);

	    /* DMA2D BGOR register configuration -------------------------------------*/
	    WRITE_REG(hdma2d->Instance->BGOR, pLayerCfg->InputOffset);

	    /* DMA2D BGCOLR register configuration -------------------------------------*/
	    if ((pLayerCfg->InputColorMode == DMA2D_INPUT_A4) || (pLayerCfg->InputColorMode == DMA2D_INPUT_A8))
	    {
	      WRITE_REG(hdma2d->Instance->BGCOLR, pLayerCfg->InputAlpha & (DMA2D_BGCOLR_BLUE|DMA2D_BGCOLR_GREEN|DMA2D_BGCOLR_RED));
	    }
	  }
	  /* Configure the foreground DMA2D layer */
	  else
	  {


	     /* Write DMA2D FGPFCCR register */
	    MODIFY_REG(hdma2d->Instance->FGPFCCR, regMask, regValue);

	    /* DMA2D FGOR register configuration -------------------------------------*/
	    WRITE_REG(hdma2d->Instance->FGOR, pLayerCfg->InputOffset);

	    /* DMA2D FGCOLR register configuration -------------------------------------*/
	    if ((pLayerCfg->InputColorMode == DMA2D_INPUT_A4) || (pLayerCfg->InputColorMode == DMA2D_INPUT_A8))
	    {
	      WRITE_REG(hdma2d->Instance->FGCOLR, pLayerCfg->InputAlpha & (DMA2D_FGCOLR_BLUE|DMA2D_FGCOLR_GREEN|DMA2D_FGCOLR_RED));
	    }
	  }

	return HAL_OK;
}
uint8_t HAL_DMA2D_Start(DMA2D_HandleTypeDef *hdma2d, uint32_t pdata, uint32_t DstAddress, uint32_t Width,  uint32_t Height)
{
	 uint32_t tmp;
	  uint32_t tmp1;
	  uint32_t tmp2;
	  uint32_t tmp3;
	  uint32_t tmp4;

	  /* Configure DMA2D data size */
	  MODIFY_REG(hdma2d->Instance->NLR, (DMA2D_NLR_NL|DMA2D_NLR_PL), (Height| (Width << DMA2D_NLR_PL_Pos)));

	  /* Configure DMA2D destination address */
	  WRITE_REG(hdma2d->Instance->OMAR, DstAddress);

	  /* Register to memory DMA2D mode selected */
	  if (hdma2d->Init.Mode == DMA2D_R2M)
	  {
	    tmp1 = pdata & DMA2D_OCOLR_ALPHA_1;
	    tmp2 = pdata & DMA2D_OCOLR_RED_1;
	    tmp3 = pdata & DMA2D_OCOLR_GREEN_1;
	    tmp4 = pdata & DMA2D_OCOLR_BLUE_1;

	    /* Prepare the value to be written to the OCOLR register according to the color mode */
	    if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_ARGB8888)
	    {
	      tmp = (tmp3 | tmp2 | tmp1| tmp4);
	    }
	    else if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_RGB888)
	    {
	      tmp = (tmp3 | tmp2 | tmp4);
	    }
	    else if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_RGB565)
	    {
	      tmp2 = (tmp2 >> 19U);
	      tmp3 = (tmp3 >> 10U);
	      tmp4 = (tmp4 >> 3U );
	      tmp  = ((tmp3 << 5U) | (tmp2 << 11U) | tmp4);
	    }
	    else if (hdma2d->Init.ColorMode == DMA2D_OUTPUT_ARGB1555)
	    {
	      tmp1 = (tmp1 >> 31U);
	      tmp2 = (tmp2 >> 19U);
	      tmp3 = (tmp3 >> 11U);
	      tmp4 = (tmp4 >> 3U );
	      tmp  = ((tmp3 << 5U) | (tmp2 << 10U) | (tmp1 << 15U) | tmp4);
	    }
	    else /* Dhdma2d->Init.ColorMode = DMA2D_OUTPUT_ARGB4444 */
	    {
	      tmp1 = (tmp1 >> 28U);
	      tmp2 = (tmp2 >> 20U);
	      tmp3 = (tmp3 >> 12U);
	      tmp4 = (tmp4 >> 4U );
	      tmp  = ((tmp3 << 4U) | (tmp2 << 8U) | (tmp1 << 12U) | tmp4);
	    }
	    /* Write to DMA2D OCOLR register */
	    WRITE_REG(hdma2d->Instance->OCOLR, tmp);
	  }
	  else /* M2M, M2M_PFC or M2M_Blending DMA2D Mode */
	  {
	    /* Configure DMA2D source address */
	    WRITE_REG(hdma2d->Instance->FGMAR, pdata);
	  }

	return HAL_OK;
}
#define HAL_ERROR 				-1
#define __HAL_DMA2D_GET_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->ISR & (__FLAG__))
#define __HAL_DMA2D_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->IFCR = (__FLAG__))
#define __HAL_DMA2D_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->CR |= (__INTERRUPT__))
#define __HAL_DMA2D_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->CR &= ~(__INTERRUPT__))
#define __HAL_DMA2D_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->CR & (__INTERRUPT__))
#define DMA2D_FLAG_CE               DMA2D_ISR_CEIF           /*!< Configuration Error Interrupt Flag */
#define DMA2D_FLAG_CTC              DMA2D_ISR_CTCIF          /*!< CLUT Transfer Complete Interrupt Flag */
#define DMA2D_FLAG_CAE              DMA2D_ISR_CAEIF          /*!< CLUT Access Error Interrupt Flag */
#define DMA2D_FLAG_TW               DMA2D_ISR_TWIF           /*!< Transfer Watermark Interrupt Flag */
#define DMA2D_FLAG_TC               DMA2D_ISR_TCIF           /*!< Transfer Complete Interrupt Flag */
#define DMA2D_FLAG_TE               DMA2D_ISR_TEIF           /*!< Transfer Error Interrupt Flag */
#define DMA2D_IT_CE                 DMA2D_CR_CEIE            /*!< Configuration Error Interrupt */
#define DMA2D_IT_CTC                DMA2D_CR_CTCIE           /*!< CLUT Transfer Complete Interrupt */
#define DMA2D_IT_CAE                DMA2D_CR_CAEIE           /*!< CLUT Access Error Interrupt */
#define DMA2D_IT_TW                 DMA2D_CR_TWIE            /*!< Transfer Watermark Interrupt */
#define DMA2D_IT_TC                 DMA2D_CR_TCIE            /*!< Transfer Complete Interrupt */
#define DMA2D_IT_TE                 DMA2D_CR_TEIE            /*!< Transfer Error Interrupt */
#define HAL_DMA2D_ERROR_NONE        0x00000000U  /*!< No error             */
#define HAL_DMA2D_ERROR_TE          0x00000001U  /*!< Transfer error       */
#define HAL_DMA2D_ERROR_CE          0x00000002U  /*!< Configuration error  */
#define HAL_DMA2D_ERROR_CAE         0x00000004U  /*!< CLUT access error    */
#define HAL_DMA2D_ERROR_TIMEOUT     0x00000020U  /*!< Timeout error        */
#define HAL_MAX_DELAY      0xFFFFFFFFU
uint8_t HAL_DMA2D_PollForTransfer(DMA2D_HandleTypeDef *hdma2d, uint32_t Timeout)
{
	  uint32_t tickstart;
	  uint32_t layer_start;
	  __IO uint32_t isrflags = 0x0U;

	  /* Polling for DMA2D transfer */
	  if((hdma2d->Instance->CR & DMA2D_CR_START) != 0U)
	  {
	   /* Get tick */
	   tickstart = st_lld_get_counter();

	    while(__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_TC) == 0U)
	    {
	      isrflags = READ_REG(hdma2d->Instance->ISR);
	      if ((isrflags & (DMA2D_FLAG_CE|DMA2D_FLAG_TE)) != 0U)
	      {
	        if ((isrflags & DMA2D_FLAG_CE) != 0U)
	        {
	          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CE;
	        }
	        if ((isrflags & DMA2D_FLAG_TE) != 0U)
	        {
	          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TE;
	        }
	        /* Clear the transfer and configuration error flags */
	        __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CE | DMA2D_FLAG_TE);

	        return HAL_ERROR;
	      }
	      /* Check for the Timeout */
	      if(Timeout != HAL_MAX_DELAY)
	      {
	        if(( TIME_I2MS(st_lld_get_counter() - tickstart ) > Timeout)||(Timeout == 0U))
	        {
	          /* Update error code */
	          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;
 	          return HAL_TIMEOUT;
	        }
	      }
	    }
	  }
	  /* Polling for CLUT loading (foreground or background) */
	  layer_start = hdma2d->Instance->FGPFCCR & DMA2D_FGPFCCR_START;
	  layer_start |= hdma2d->Instance->BGPFCCR & DMA2D_BGPFCCR_START;
	  if (layer_start != 0U)
	  {
	    /* Get tick */
	    tickstart = st_lld_get_counter();

	    while(__HAL_DMA2D_GET_FLAG(hdma2d, DMA2D_FLAG_CTC) == 0U)
	    {
	      isrflags = READ_REG(hdma2d->Instance->ISR);
	      if ((isrflags & (DMA2D_FLAG_CAE|DMA2D_FLAG_CE|DMA2D_FLAG_TE)) != 0U)
	      {
	        if ((isrflags & DMA2D_FLAG_CAE) != 0U)
	        {
	          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CAE;
	        }
	        if ((isrflags & DMA2D_FLAG_CE) != 0U)
	        {
	          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_CE;
	        }
	        if ((isrflags & DMA2D_FLAG_TE) != 0U)
	        {
	          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TE;
	        }
	        /* Clear the CLUT Access Error, Configuration Error and Transfer Error flags */
	        __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_CAE | DMA2D_FLAG_CE | DMA2D_FLAG_TE);


	        return HAL_ERROR;
	      }
	      /* Check for the Timeout */
	      if(Timeout != HAL_MAX_DELAY)
	      {
	        if( (TIME_I2MS(st_lld_get_counter() - tickstart ) > Timeout)||(Timeout == 0U))
	        {
	          /* Update error code */
	          hdma2d->ErrorCode |= HAL_DMA2D_ERROR_TIMEOUT;


	          return HAL_TIMEOUT;
	        }
	      }
	    }
	  }

	  /* Clear the transfer complete and CLUT loading flags */
	  __HAL_DMA2D_CLEAR_FLAG(hdma2d, DMA2D_FLAG_TC|DMA2D_FLAG_CTC);
	return HAL_OK;
}
static DMA2D_HandleTypeDef hdma2d_discovery;
static void LL_FillBuffer(uint32_t LayerIndex, void *pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex)
{
  /* Register to memory mode with ARGB8888 as color Mode */
  hdma2d_discovery.Init.Mode         = DMA2D_R2M;
  hdma2d_discovery.Init.ColorMode    = DMA2D_OUTPUT_RGB888;
  hdma2d_discovery.Init.OutputOffset = OffLine;

  hdma2d_discovery.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d_discovery) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d_discovery, LayerIndex) == HAL_OK){
      if (HAL_DMA2D_Start(&hdma2d_discovery, ColorIndex, (uint32_t)pDst, xSize, ySize) == HAL_OK){
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d_discovery, 10);
      }
    }
  }
}
#define DMA2D_M2M_PFC               DMA2D_CR_MODE_0                     /*!< DMA2D memory to memory with pixel format conversion transfer mode */
#define DMA2D_NO_MODIF_ALPHA        0x00000000U  /*!< No modification of the alpha channel value */



void BSP_LCD_Clear(uint32_t LayerIndex, uint32_t Color, uint32_t *FBStartAdress){


  /* Clear the LCD */
  LL_FillBuffer(LayerIndex, FBStartAdress, 800, 480, 0, Color);
}

void HAL_DSI_MspDeInit(DSI_HandleTypeDef *hdsi){UNUSED(hdsi);
	  ///Start----->HAL_DSI_DeInit(&(hdsi_discovery));
	  __HAL_DSI_WRAPPER_DISABLE(DSI);

	  /* Disable the DSI host */
	  __HAL_DSI_DISABLE(DSI);

	  /* D-PHY clock and digital disable */
	  DSI->PCTLR &= ~(DSI_PCTLR_CKE | DSI_PCTLR_DEN);

	  /* Turn off the DSI PLL */
	  __HAL_DSI_PLL_DISABLE(DSI);

	  /* Disable the regulator */
	  __HAL_DSI_REG_DISABLE(DSI);

	  ///ENDs----->HAL_DSI_DeInit(&(hdsi_discovery));

}
HAL_StatusTypeDef HAL_DSI_DeInit(DSI_HandleTypeDef *hdsi)
{
  /* Check the DSI handle allocation */
  if (hdsi == NULL)
  {
    return HAL_ERROR;
  }

  /* Change DSI peripheral state */
  hdsi->State = HAL_DSI_STATE_BUSY;

  /* Disable the DSI wrapper */
  __HAL_DSI_WRAPPER_DISABLE(DSI);

  /* Disable the DSI host */
  __HAL_DSI_DISABLE(DSI);

  /* D-PHY clock and digital disable */
  hdsi->Instance->PCTLR &= ~(DSI_PCTLR_CKE | DSI_PCTLR_DEN);

  /* Turn off the DSI PLL */
  __HAL_DSI_PLL_DISABLE(DSI);

  /* Disable the regulator */
  __HAL_DSI_REG_DISABLE(DSI);

#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
  if (hdsi->MspDeInitCallback == NULL)
  {
    hdsi->MspDeInitCallback = HAL_DSI_MspDeInit;
  }
  /* DeInit the low level hardware */
  hdsi->MspDeInitCallback(hdsi);
#else
  /* DeInit the low level hardware */
  HAL_DSI_MspDeInit(hdsi);
#endif /* USE_HAL_DSI_REGISTER_CALLBACKS */

  /* Initialise the error code */
  hdsi->ErrorCode = HAL_DSI_ERROR_NONE;

  /* Initialize the DSI state*/
  hdsi->State = HAL_DSI_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hdsi);

  return HAL_OK;
}
#endif
