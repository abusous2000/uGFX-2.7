/*
 * STM32DSI.h
 *
 *  Created on: May 11, 2020
 *      Author: abusous2000
 */
#include "Strust4EmbeddedConf.h"
#if USE_LCD_TFT == 1
#ifndef CFG_STM32F769_DISCOVERY_STM32DSI_H_
#define CFG_STM32F769_DISCOVERY_STM32DSI_H_


typedef struct
{
  uint32_t PLLNDIV;                      /*!< PLL Loop Division Factor
                                              This parameter must be a value between 10 and 125   */

  uint32_t PLLIDF;                       /*!< PLL Input Division Factor
                                              This parameter can be any value of @ref DSI_PLL_IDF */

  uint32_t PLLODF;                       /*!< PLL Output Division Factor
                                              This parameter can be any value of @ref DSI_PLL_ODF */

} DSI_PLLInitTypeDef;

/**
  * @brief  DSI Video mode configuration
  */
typedef struct
{
  uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                 */

  uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding           */

  uint32_t LooselyPacked;                /*!< Enable or disable loosely packed stream (needed only when using
                                              18-bit configuration).
                                              This parameter can be any value of @ref DSI_LooselyPacked          */

  uint32_t Mode;                         /*!< Video mode type
                                              This parameter can be any value of @ref DSI_Video_Mode_Type        */

  uint32_t PacketSize;                   /*!< Video packet size                                                  */

  uint32_t NumberOfChunks;               /*!< Number of chunks                                                   */

  uint32_t NullPacketSize;               /*!< Null packet size                                                   */

  uint32_t HSPolarity;                   /*!< HSYNC pin polarity
                                              This parameter can be any value of @ref DSI_HSYNC_Polarity         */

  uint32_t VSPolarity;                   /*!< VSYNC pin polarity
                                              This parameter can be any value of @ref DSI_VSYNC_Active_Polarity  */

  uint32_t DEPolarity;                   /*!< Data Enable pin polarity
                                              This parameter can be any value of @ref DSI_DATA_ENABLE_Polarity   */

  uint32_t HorizontalSyncActive;         /*!< Horizontal synchronism active duration (in lane byte clock cycles) */

  uint32_t HorizontalBackPorch;          /*!< Horizontal back-porch duration (in lane byte clock cycles)         */

  uint32_t HorizontalLine;               /*!< Horizontal line duration (in lane byte clock cycles)               */

  uint32_t VerticalSyncActive;           /*!< Vertical synchronism active duration                               */

  uint32_t VerticalBackPorch;            /*!< Vertical back-porch duration                                       */

  uint32_t VerticalFrontPorch;           /*!< Vertical front-porch duration                                      */

  uint32_t VerticalActive;               /*!< Vertical active duration                                           */

  uint32_t LPCommandEnable;              /*!< Low-power command enable
                                              This parameter can be any value of @ref DSI_LP_Command             */

  uint32_t LPLargestPacketSize;          /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VSA, VBP and VFP regions                  */

  uint32_t LPVACTLargestPacketSize;      /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VACT region                               */

  uint32_t LPHorizontalFrontPorchEnable; /*!< Low-power horizontal front-porch enable
                                              This parameter can be any value of @ref DSI_LP_HFP                 */

  uint32_t LPHorizontalBackPorchEnable;  /*!< Low-power horizontal back-porch enable
                                              This parameter can be any value of @ref DSI_LP_HBP                 */

  uint32_t LPVerticalActiveEnable;       /*!< Low-power vertical active enable
                                              This parameter can be any value of @ref DSI_LP_VACT                */

  uint32_t LPVerticalFrontPorchEnable;   /*!< Low-power vertical front-porch enable
                                              This parameter can be any value of @ref DSI_LP_VFP                 */

  uint32_t LPVerticalBackPorchEnable;    /*!< Low-power vertical back-porch enable
                                              This parameter can be any value of @ref DSI_LP_VBP                 */

  uint32_t LPVerticalSyncActiveEnable;   /*!< Low-power vertical sync active enable
                                              This parameter can be any value of @ref DSI_LP_VSYNC               */

  uint32_t FrameBTAAcknowledgeEnable;    /*!< Frame bus-turn-around acknowledge enable
                                              This parameter can be any value of @ref DSI_FBTA_acknowledge       */

} DSI_VidCfgTypeDef;
typedef struct
{
  uint32_t AutomaticClockLaneControl;    /*!< Automatic clock lane control
                                              This parameter can be any value of @ref DSI_Automatic_Clk_Lane_Control */

  uint32_t TXEscapeCkdiv;                /*!< TX Escape clock division
                                              The values 0 and 1 stop the TX_ESC clock generation                    */

  uint32_t NumberOfLanes;                /*!< Number of lanes
                                              This parameter can be any value of @ref DSI_Number_Of_Lanes            */

} DSI_InitTypeDef;
typedef enum
{
  HAL_DSI_STATE_RESET   = 0x00U,
  HAL_DSI_STATE_READY   = 0x01U,
  HAL_DSI_STATE_ERROR   = 0x02U,
  HAL_DSI_STATE_BUSY    = 0x03U,
  HAL_DSI_STATE_TIMEOUT = 0x04U
} HAL_DSI_StateTypeDef;
typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
}HAL_LockTypeDef;
//typedef enum
//{
//  HAL_DSI_STATE_RESET   = 0x00U,
//  HAL_DSI_STATE_READY   = 0x01U,
//  HAL_DSI_STATE_ERROR   = 0x02U,
//  HAL_DSI_STATE_BUSY    = 0x03U,
//  HAL_DSI_STATE_TIMEOUT = 0x04U
//} HAL_DSI_StateTypeDef;
#define USE_HAL_DSI_REGISTER_CALLBACKS  0
typedef struct
{
  DSI_TypeDef               *Instance;    /*!< Register base address      */
  DSI_InitTypeDef           Init;         /*!< DSI required parameters    */
  HAL_LockTypeDef           Lock;         /*!< DSI peripheral status      */
  __IO HAL_DSI_StateTypeDef State;        /*!< DSI communication state    */
  __IO uint32_t             ErrorCode;    /*!< DSI Error code             */
  uint32_t                  ErrorMsk;     /*!< DSI Error monitoring mask  */

#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
  void (* TearingEffectCallback)(struct __DSI_HandleTypeDef *hdsi);   /*!< DSI Tearing Effect Callback */
  void (* EndOfRefreshCallback)(struct __DSI_HandleTypeDef *hdsi);    /*!< DSI End Of Refresh Callback */
  void (* ErrorCallback)(struct __DSI_HandleTypeDef *hdsi);           /*!< DSI Error Callback          */

  void (* MspInitCallback)(struct __DSI_HandleTypeDef *hdsi);         /*!< DSI Msp Init callback       */
  void (* MspDeInitCallback)(struct __DSI_HandleTypeDef *hdsi);       /*!< DSI Msp DeInit callback     */

#endif /* USE_HAL_DSI_REGISTER_CALLBACKS */

} DSI_HandleTypeDef;
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
#define HSE_VALUE                25000000U
#define HAL_DSI_ERROR_NONE              0U
#define HAL_DSI_ERROR_ACK               0x00000001U /*!< acknowledge errors          */
#define HAL_DSI_ERROR_PHY               0x00000002U /*!< PHY related errors          */
#define HAL_DSI_ERROR_TX                0x00000004U /*!< transmission error          */
#define HAL_DSI_ERROR_RX                0x00000008U /*!< reception error             */
#define HAL_DSI_ERROR_ECC               0x00000010U /*!< ECC errors                  */
#define HAL_DSI_ERROR_CRC               0x00000020U /*!< CRC error                   */
#define HAL_DSI_ERROR_PSE               0x00000040U /*!< Packet Size error           */
#define HAL_DSI_ERROR_EOT               0x00000080U /*!< End Of Transmission error   */
#define HAL_DSI_ERROR_OVF               0x00000100U /*!< FIFO overflow error         */
#define HAL_DSI_ERROR_GEN               0x00000200U /*!< Generic FIFO related errors */
#define __HAL_UNLOCK(__HANDLE__)                                          \
                                do{                                       \
                                    (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                  }while (0U)

#if defined(STM32F769xx)
#define UNUSED(X) 				   (void)X      /* To avoid gcc/g++ warnings */
#define __HAL_RCC_DSI_FORCE_RESET()      (RCC->APB2RSTR |= (RCC_APB2RSTR_DSIRST))
#define __HAL_RCC_DSI_RELEASE_RESET()    (RCC->APB2RSTR &= ~(RCC_APB2RSTR_DSIRST))
#define __HAL_RCC_LTDC_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_LTDCEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_LTDCEN);\
                                        UNUSED(tmpreg); \
                                      } while(0)

#define __HAL_RCC_LTDC_FORCE_RESET()     (RCC->APB2RSTR |= (RCC_APB2RSTR_LTDCRST)) //already done upstairs
#define __HAL_RCC_LTDC_RELEASE_RESET()   (RCC->APB2RSTR &= ~(RCC_APB2RSTR_LTDCRST))

#define __HAL_RCC_DMA2D_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2DEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2DEN);\
                                        UNUSED(tmpreg); \
                                      } while(0)
#define __HAL_RCC_DMA2D_FORCE_RESET()    (RCC->AHB1RSTR |= (RCC_AHB1RSTR_DMA2DRST))
#define __HAL_RCC_DMA2D_RELEASE_RESET()  (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_DMA2DRST))
#define __HAL_RCC_DSI_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_DSIEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_DSIEN);\
                                        UNUSED(tmpreg); \
                                      } while(0)
#define __HAL_RCC_DSI_FORCE_RESET()      (RCC->APB2RSTR |= (RCC_APB2RSTR_DSIRST))
#define __HAL_RCC_DSI_RELEASE_RESET()    (RCC->APB2RSTR &= ~(RCC_APB2RSTR_DSIRST))
#define DSI_PLL_OUT_DIV1            0x00000000U
#define DSI_PLL_IN_DIV5             0x00000005U
#define __HAL_DSI_PLL_ENABLE() do { \
                                             __IO uint32_t tmpreg = 0x00U; \
                                             SET_BIT(DSI->WRPCR, DSI_WRPCR_PLLEN);\
                                             /* Delay after an DSI PLL enabling */ \
                                             tmpreg = READ_BIT(DSI->WRPCR, DSI_WRPCR_PLLEN);\
                                             UNUSED(tmpreg); \
                                            } while(0U)
#define DSI_AUTO_CLK_LANE_CTRL_DISABLE 0x00000000U
#define DSI_ONE_DATA_LANE          0U
#define DSI_TWO_DATA_LANES         1U
#define  OTM8009A_480X800_WIDTH             ((uint16_t)480)     /* LCD PIXEL WIDTH   */
#define  OTM8009A_480X800_HEIGHT            ((uint16_t)800)     /* LCD PIXEL HEIGHT  */
#define   LCD_DSI_PIXEL_DATA_FMT_RBG888  DSI_RGB888 /*!< DSI packet pixel format chosen is RGB888 : 24 bpp */
#define   LCD_DSI_PIXEL_DATA_FMT_RBG565  DSI_RGB565 /*!< DSI packet pixel format chosen is RGB565 : 16 bpp */
#define LCD_OTM8009A_ID  				    ((uint32_t) 0)
#define DSI_RGB565                 		0x00000000U /*!< The values 0x00000001 and 0x00000002 can also be used for the RGB565 color mode configuration */
#define DSI_RGB666                 		0x00000003U /*!< The value 0x00000004 can also be used for the RGB666 color mode configuration                 */
#define DSI_RGB888                 		0x00000005U
#define   LCD_DSI_PIXEL_DATA_FMT_RBG888 DSI_RGB888 /*!< DSI packet pixel format chosen is RGB888 : 24 bpp */
#define   LCD_DSI_PIXEL_DATA_FMT_RBG565 DSI_RGB565 /*!< DSI packet pixel format chosen is RGB565 : 16 bpp */
#define DSI_VSYNC_ACTIVE_HIGH       	0x00000000U
#define DSI_VSYNC_ACTIVE_LOW        	DSI_LPCR_VSP
#define DSI_DATA_ENABLE_ACTIVE_HIGH 	0x00000000U
#define DSI_VID_MODE_NB_PULSES    		0U
#define DSI_VID_MODE_NB_EVENTS    		1U
#define DSI_VID_MODE_BURST        		2U
#define DSI_HSYNC_ACTIVE_HIGH       	0x00000000U
#define DSI_LP_COMMAND_DISABLE    		0x00000000U
#define DSI_LP_COMMAND_ENABLE     		DSI_VMCR_LPCE
#define DSI_LP_HFP_DISABLE        		0x00000000U
#define DSI_LP_HFP_ENABLE         		DSI_VMCR_LPHFPE
#define DSI_LP_HBP_DISABLE        		0x00000000U
#define DSI_LP_HBP_ENABLE         		DSI_VMCR_LPHBPE
#define DSI_LP_VACT_DISABLE       		0x00000000U
#define DSI_LP_VACT_ENABLE        		DSI_VMCR_LPVAE
#define DSI_LP_VFP_DISABLE       		0x00000000U
#define DSI_LP_VFP_ENABLE        		DSI_VMCR_LPVFPE
#define DSI_LP_VBP_DISABLE       		0x00000000U
#define DSI_LP_VBP_ENABLE       		DSI_VMCR_LPVBPE
#define DSI_LP_VSYNC_DISABLE     		0x00000000U
#define DSI_LP_VSYNC_ENABLE      		DSI_VMCR_LPVSAE
/* Specify for each region of the video frame, if the transmission of command in LP mode is allowed in this region */
#define DSI_LOOSELY_PACKED_ENABLE  		DSI_LCOLCR_LPE
#define DSI_LOOSELY_PACKED_DISABLE 		0x00000000U
#define DSI_FBTAA_DISABLE        		0x00000000U
#define DSI_FBTAA_ENABLE         		DSI_VMCR_FBTAAE
#define HAL_TIMEOUT                     -1
#define SET_BIT(REG, BIT)     			((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   			((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    			((REG) & (BIT))
#define DSI_TIMEOUT_VALUE                1000U
#define LCD_OTM8009A_ID  				((uint32_t) 0)
#define DSI_DCS_SHORT_PKT_WRITE_P1  	0x00000015U
#define DSI_DATA_ENABLE_ACTIVE_HIGH 	0x00000000U
#define DSI_VSYNC_ACTIVE_HIGH       	0x00000000U
#define DSI_HSYNC_ACTIVE_HIGH       	0x00000000U
#define LTDC_VSPOLARITY_AL              0x00000000U   /*!< Vertical Synchronization is active low. */
#define LTDC_VSPOLARITY_AH              LTDC_GCR_VSPOL            /*!< Vertical Synchronization is active high. */
#define LTDC_DEPOLARITY_AL              0x00000000U   /*!< Data Enable, is active low. */
#define LTDC_DEPOLARITY_AH              LTDC_GCR_DEPOL            /*!< Data Enable, is active high. */
#define LTDC_PCPOLARITY_IPC             0x00000000U   /*!< input pixel clock. */
#define LTDC_PCPOLARITY_IIPC            LTDC_GCR_PCPOL            /*!< inverted input pixel clock. */
#define LTDC_HSPOLARITY_AL                0x00000000U   /*!< Horizontal Synchronization is active low. */
#define LTDC_HSPOLARITY_AH                LTDC_GCR_HSPOL            /*!< Horizontal Synchronization is active high. */
#define LTDC_VSPOLARITY_AL                0x00000000U   /*!< Vertical Synchronization is active low. */
#define LTDC_VSPOLARITY_AH                LTDC_GCR_VSPOL
#define DSI_DCS_LONG_PKT_WRITE      0x00000039U /*!< DCS long write     */
#define DSI_GEN_LONG_PKT_WRITE      0x00000029U /*!< Generic long write */
#define __HAL_DSI_REG_ENABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              SET_BIT((__HANDLE__)->WRPCR, DSI_WRPCR_REGEN);\
                                              /* Delay after an DSI regulator enabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->WRPCR, DSI_WRPCR_REGEN);\
                                              UNUSED(tmpreg); \
                                            } while(0U)

#define WAIT_FOR_DSI_FLAG(__x)          {\
											uint32_t _tickstart = st_lld_get_counter();\
											while( (__x) == 0U){\
													if( (TIME_I2MS(st_lld_get_counter() - _tickstart)) > DSI_TIMEOUT_VALUE){\
														return HAL_TIMEOUT;\
													}\
												  chThdYield();\
											}\
										 }
#define __HAL_DSI_ENABLE(__HANDLE__) do { \
                                          __IO uint32_t tmpreg = 0x00U; \
                                          SET_BIT((__HANDLE__)->CR, DSI_CR_EN);\
                                          /* Delay after an DSI Host enabling */ \
                                          tmpreg = READ_BIT((__HANDLE__)->CR, DSI_CR_EN);\
                                          UNUSED(tmpreg); \
                                        } while(0U)

#define __HAL_DSI_WRAPPER_ENABLE(__HANDLE__) do { \
                                                 __IO uint32_t tmpreg = 0x00U; \
                                                 SET_BIT((__HANDLE__)->WCR, DSI_WCR_DSIEN);\
                                                 /* Delay after an DSI warpper enabling */ \
                                                 tmpreg = READ_BIT((__HANDLE__)->WCR, DSI_WCR_DSIEN);\
                                                 UNUSED(tmpreg); \
                                                } while(0U)

#define __HAL_DSI_WRAPPER_DISABLE(__HANDLE__) do { \
                                                  __IO uint32_t tmpreg = 0x00U; \
                                                  CLEAR_BIT((__HANDLE__)->WCR, DSI_WCR_DSIEN);\
                                                  /* Delay after an DSI warpper disabling*/ \
                                                  tmpreg = READ_BIT((__HANDLE__)->WCR, DSI_WCR_DSIEN);\
                                                  UNUSED(tmpreg); \
                                                 } while(0U)

#define __HAL_DSI_DISABLE(__HANDLE__) do { \
                                          __IO uint32_t tmpreg = 0x00U; \
                                          CLEAR_BIT((__HANDLE__)->CR, DSI_CR_EN);\
                                          /* Delay after an DSI Host disabling */ \
                                          tmpreg = READ_BIT((__HANDLE__)->CR, DSI_CR_EN);\
                                          UNUSED(tmpreg); \
                                         } while(0U)
#define __HAL_DSI_PLL_DISABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              CLEAR_BIT((__HANDLE__)->WRPCR, DSI_WRPCR_PLLEN);\
                                              /* Delay after an DSI PLL disabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->WRPCR, DSI_WRPCR_PLLEN);\
                                              UNUSED(tmpreg); \
                                             } while(0U)

#define __HAL_DSI_REG_DISABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              CLEAR_BIT((__HANDLE__)->WRPCR, DSI_WRPCR_REGEN);\
                                              /* Delay after an DSI regulator disabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->WRPCR, DSI_WRPCR_REGEN);\
                                              UNUSED(tmpreg); \
                                             } while(0U)

#define DSI_FLAG_PLLLS                    DSI_WISR_PLLLS
#define __HAL_LTDC_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->IER |= (__INTERRUPT__))
#define __HAL_LTDC_ENABLE(__HANDLE__)    ((__HANDLE__)->GCR |= LTDC_GCR_LTDCEN)


#define LTDC_IT_LI                        LTDC_IER_LIE              /*!< LTDC Line Interrupt            */
#define LTDC_IT_FU                        LTDC_IER_FUIE             /*!< LTDC FIFO Underrun Interrupt   */
#define LTDC_IT_TE                        LTDC_IER_TERRIE           /*!< LTDC Transfer Error Interrupt  */
#define LTDC_IT_RR                        LTDC_IER_RRIE             /*!< LTDC Register Reload Interrupt */
#define LTDC_PIXEL_FORMAT_ARGB8888        0x00000000U   /*!< ARGB8888 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_RGB888          0x00000001U   /*!< RGB888 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_RGB565          0x00000002U   /*!< RGB565 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_ARGB1555        0x00000003U   /*!< ARGB1555 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_ARGB4444        0x00000004U   /*!< ARGB4444 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_L8              0x00000005U   /*!< L8 LTDC pixel format       */
#define LTDC_PIXEL_FORMAT_AL44            0x00000006U   /*!< AL44 LTDC pixel format     */
#define LTDC_PIXEL_FORMAT_AL88            0x00000007U   /*!< AL88 LTDC pixel format     */
#define LTDC_BLENDING_FACTOR1_PAxCA       0x00000600U   /*!< Blending factor : Cte Alpha x Pixel Alpha*/
#define LTDC_BLENDING_FACTOR2_PAxCA       0x00000007U   /*!< Blending factor : Cte Alpha x Pixel Alpha*/
#define LCD_FB_START_ADDRESS              ((uint32_t*)0xC0000000)


void setBacklightPWM(uint8_t percent);
int initDSI(orientation_t orientation);
int postInitDSI(orientation_t orientation);
void BSP_LCD_DisplayOn(void);
void BSP_LCD_Reset(void);
uint8_t DSI_ShortWrite(uint32_t ChannelID, uint32_t Mode, uint32_t Param1, uint32_t Param2);
uint8_t DSI_LongWrite(uint32_t ChannelID, uint32_t Mode, uint32_t NbParams,  uint32_t Param1, uint8_t *ParametersTable);
void BSP_LCD_Clear(uint32_t LayerIndex,uint32_t Color, uint32_t *FBStartAdress);
#endif /*#if defined(STM32F769xx) */
#endif /* CFG_STM32F769_DISCOVERY_STM32DSI_H_ */


#endif
