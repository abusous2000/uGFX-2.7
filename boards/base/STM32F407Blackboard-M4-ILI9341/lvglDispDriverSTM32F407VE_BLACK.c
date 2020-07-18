
/* lvglDriver for Mbed
 * Copyright (c) 2019 Johannes Stratmann
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ili9341_fsmc.h"

/*
    use FSMC
*/

volatile uint16_t *ili9341_fsmcCommand;
volatile uint16_t *ili9341_fsmcData;

typedef struct
{
  uint32_t NSBank;                       /*!< Specifies the NORSRAM memory device that will be used.
                                              This parameter can be a value of @ref FSMC_NORSRAM_Bank                     */

  uint32_t DataAddressMux;               /*!< Specifies whether the address and data values are
                                              multiplexed on the data bus or not.
                                              This parameter can be a value of @ref FSMC_Data_Address_Bus_Multiplexing    */

  uint32_t MemoryType;                   /*!< Specifies the type of external memory attached to
                                              the corresponding memory device.
                                              This parameter can be a value of @ref FSMC_Memory_Type                      */

  uint32_t MemoryDataWidth;              /*!< Specifies the external memory device width.
                                              This parameter can be a value of @ref FSMC_NORSRAM_Data_Width               */

  uint32_t BurstAccessMode;              /*!< Enables or disables the burst access mode for Flash memory,
                                              valid only with synchronous burst Flash memories.
                                              This parameter can be a value of @ref FSMC_Burst_Access_Mode                */

  uint32_t WaitSignalPolarity;           /*!< Specifies the wait signal polarity, valid only when accessing
                                              the Flash memory in burst mode.
                                              This parameter can be a value of @ref FSMC_Wait_Signal_Polarity             */

  uint32_t WrapMode;                     /*!< Enables or disables the Wrapped burst access mode for Flash
                                              memory, valid only when accessing Flash memories in burst mode.
                                              This parameter can be a value of @ref FSMC_Wrap_Mode
                                              This mode is available only for the STM32F405/407/4015/417xx devices        */

  uint32_t WaitSignalActive;             /*!< Specifies if the wait signal is asserted by the memory one
                                              clock cycle before the wait state or during the wait state,
                                              valid only when accessing memories in burst mode.
                                              This parameter can be a value of @ref FSMC_Wait_Timing                      */

  uint32_t WriteOperation;               /*!< Enables or disables the write operation in the selected device by the FSMC.
                                              This parameter can be a value of @ref FSMC_Write_Operation                  */

  uint32_t WaitSignal;                   /*!< Enables or disables the wait state insertion via wait
                                              signal, valid for Flash memory access in burst mode.
                                              This parameter can be a value of @ref FSMC_Wait_Signal                      */

  uint32_t ExtendedMode;                 /*!< Enables or disables the extended mode.
                                              This parameter can be a value of @ref FSMC_Extended_Mode                    */

  uint32_t AsynchronousWait;             /*!< Enables or disables wait signal during asynchronous transfers,
                                              valid only with asynchronous Flash memories.
                                              This parameter can be a value of @ref FSMC_AsynchronousWait                 */

  uint32_t WriteBurst;                   /*!< Enables or disables the write burst operation.
                                              This parameter can be a value of @ref FSMC_Write_Burst                      */

  uint32_t ContinuousClock;              /*!< Enables or disables the FMC clock output to external memory devices.
                                              This parameter is only enabled through the FMC_BCR1 register, and don't care
                                              through FMC_BCR2..4 registers.
                                              This parameter can be a value of @ref FMC_Continous_Clock
                                              This mode is available only for the STM32F412Vx/Zx/Rx devices                 */

  uint32_t WriteFifo;                    /*!< Enables or disables the write FIFO used by the FMC controller.
                                              This parameter is only enabled through the FMC_BCR1 register, and don't care
                                              through FMC_BCR2..4 registers.
                                              This parameter can be a value of @ref FMC_Write_FIFO
                                              This mode is available only for the STM32F412Vx/Vx devices                    */

  uint32_t PageSize;                     /*!< Specifies the memory page size.
                                              This parameter can be a value of @ref FMC_Page_Size                   */
}FSMC_NORSRAM_InitTypeDef;

HAL_StatusTypeDef  FMC_NORSRAM_Init(FMC_NORSRAM_TypeDef *Device, FMC_NORSRAM_InitTypeDef* Init)
{
  uint32_t tmpr = 0U;

  /* Check the parameters */
  assert_param(IS_FMC_NORSRAM_DEVICE(Device));
  assert_param(IS_FMC_NORSRAM_BANK(Init->NSBank));
  assert_param(IS_FMC_MUX(Init->DataAddressMux));
  assert_param(IS_FMC_MEMORY(Init->MemoryType));
  assert_param(IS_FMC_NORSRAM_MEMORY_WIDTH(Init->MemoryDataWidth));
  assert_param(IS_FMC_BURSTMODE(Init->BurstAccessMode));
  assert_param(IS_FMC_WAIT_POLARITY(Init->WaitSignalPolarity));
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  assert_param(IS_FMC_WRAP_MODE(Init->WrapMode));
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */
  assert_param(IS_FMC_WAIT_SIGNAL_ACTIVE(Init->WaitSignalActive));
  assert_param(IS_FMC_WRITE_OPERATION(Init->WriteOperation));
  assert_param(IS_FMC_WAITE_SIGNAL(Init->WaitSignal));
  assert_param(IS_FMC_EXTENDED_MODE(Init->ExtendedMode));
  assert_param(IS_FMC_ASYNWAIT(Init->AsynchronousWait));
  assert_param(IS_FMC_WRITE_BURST(Init->WriteBurst));
  assert_param(IS_FMC_CONTINOUS_CLOCK(Init->ContinuousClock));
  assert_param(IS_FMC_PAGESIZE(Init->PageSize));
#if defined (STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
  assert_param(IS_FMC_WRITE_FIFO(Init->WriteFifo));
#endif /* STM32F446xx || STM32F469xx || STM32F479xx */

  /* Get the BTCR register value */
  tmpr = Device->BTCR[Init->NSBank];

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Clear MBKEN, MUXEN, MTYP, MWID, FACCEN, BURSTEN, WAITPOL, WRAPMOD, WAITCFG, WREN,
           WAITEN, EXTMOD, ASYNCWAIT, CPSIZE, CBURSTRW and CCLKEN bits */
  tmpr &= ((uint32_t)~(FMC_BCR1_MBKEN     | FMC_BCR1_MUXEN    | FMC_BCR1_MTYP     | \
                       FMC_BCR1_MWID      | FMC_BCR1_FACCEN   | FMC_BCR1_BURSTEN  | \
                       FMC_BCR1_WAITPOL   | FMC_BCR1_WRAPMOD  | FMC_BCR1_WAITCFG  | \
                       FMC_BCR1_WREN      | FMC_BCR1_WAITEN   | FMC_BCR1_EXTMOD   | \
                       FMC_BCR1_ASYNCWAIT | FMC_BCR1_CPSIZE   | FMC_BCR1_CBURSTRW | \
                       FMC_BCR1_CCLKEN));

  /* Set NORSRAM device control parameters */
  tmpr |= (uint32_t)(Init->DataAddressMux       |\
                    Init->MemoryType           |\
                    Init->MemoryDataWidth      |\
                    Init->BurstAccessMode      |\
                    Init->WaitSignalPolarity   |\
                    Init->WrapMode             |\
                    Init->WaitSignalActive     |\
                    Init->WriteOperation       |\
                    Init->WaitSignal           |\
                    Init->ExtendedMode         |\
                    Init->AsynchronousWait     |\
                    Init->PageSize             |\
                    Init->WriteBurst           |\
                    Init->ContinuousClock);
#else /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) */
  /* Clear MBKEN, MUXEN, MTYP, MWID, FACCEN, BURSTEN, WAITPOL, CPSIZE, WAITCFG, WREN,
           WAITEN, EXTMOD, ASYNCWAIT, CBURSTRW, CCLKEN and WFDIS bits */
  tmpr &= ((uint32_t)~(FMC_BCR1_MBKEN     | FMC_BCR1_MUXEN    | FMC_BCR1_MTYP     | \
                       FMC_BCR1_MWID      | FMC_BCR1_FACCEN   | FMC_BCR1_BURSTEN  | \
                       FMC_BCR1_WAITPOL   | FMC_BCR1_WAITCFG  | FMC_BCR1_CPSIZE   | \
                       FMC_BCR1_WREN      | FMC_BCR1_WAITEN   | FMC_BCR1_EXTMOD   | \
                       FMC_BCR1_ASYNCWAIT | FMC_BCR1_CBURSTRW | FMC_BCR1_CCLKEN   | \
                       FMC_BCR1_WFDIS));

  /* Set NORSRAM device control parameters */
  tmpr |= (uint32_t)(Init->DataAddressMux       |\
                    Init->MemoryType           |\
                    Init->MemoryDataWidth      |\
                    Init->BurstAccessMode      |\
                    Init->WaitSignalPolarity   |\
                    Init->WaitSignalActive     |\
                    Init->WriteOperation       |\
                    Init->WaitSignal           |\
                    Init->ExtendedMode         |\
                    Init->AsynchronousWait     |\
                    Init->WriteBurst           |\
                    Init->ContinuousClock      |\
                    Init->PageSize             |\
                    Init->WriteFifo);
#endif /*  defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) */

  if(Init->MemoryType == FMC_MEMORY_TYPE_NOR)
  {
    tmpr |= (uint32_t)FMC_NORSRAM_FLASH_ACCESS_ENABLE;
  }

  Device->BTCR[Init->NSBank] = tmpr;

  /* Configure synchronous mode when Continuous clock is enabled for bank2..4 */
  if((Init->ContinuousClock == FMC_CONTINUOUS_CLOCK_SYNC_ASYNC) && (Init->NSBank != FMC_NORSRAM_BANK1))
  {
    Device->BTCR[FMC_NORSRAM_BANK1] |= (uint32_t)(Init->ContinuousClock);
  }

#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
  if(Init->NSBank != FMC_NORSRAM_BANK1)
  {
    Device->BTCR[FMC_NORSRAM_BANK1] |= (uint32_t)(Init->WriteFifo);
  }
#endif /* STM32F446xx || STM32F469xx || STM32F479xx */

  return HAL_OK;
}
static int fsmc_lcd_init()
{
    // enable clocks
    // workaround, the HAL macros produce warnings
    volatile uint32_t dummy;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    dummy = RCC->AHB1ENR;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    dummy = RCC->AHB1ENR;
    RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
    dummy = RCC->AHB3ENR;
    dummy = dummy;

    // set GPIO Alternate Function FSMC
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
                             GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                             GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                             GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                             GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

    FSMC_NORSRAM_InitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMInitStructure.NSBank = FSMC_NORSRAM_BANK1;                       // ??
    FSMC_NORSRAMInitStructure.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    FSMC_NORSRAMInitStructure.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    FSMC_NORSRAMInitStructure.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    FSMC_NORSRAMInitStructure.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;    // disable
    FSMC_NORSRAMInitStructure.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    FSMC_NORSRAMInitStructure.WrapMode = FSMC_WRAP_MODE_DISABLE;
    FSMC_NORSRAMInitStructure.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    FSMC_NORSRAMInitStructure.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    FSMC_NORSRAMInitStructure.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    FSMC_NORSRAMInitStructure.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    FSMC_NORSRAMInitStructure.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    FSMC_NORSRAMInitStructure.WriteBurst = FSMC_WRITE_BURST_DISABLE;  // enable?
    FSMC_NORSRAMInitStructure.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
    FSMC_NORSRAMInitStructure.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
    FSMC_NORSRAMInitStructure.PageSize = 0;

    HAL_StatusTypeDef status = HAL_OK;
    status = FSMC_NORSRAM_Init(FSMC_Bank1, &FSMC_NORSRAMInitStructure);

    FSMC_NORSRAM_TimingTypeDef FSMC_NORSRAMTimingInitStructure;
    FSMC_NORSRAMTimingInitStructure.AddressSetupTime = 7; // 100 ns (ADDSET+1)*12.5ns = CS to RW
    FSMC_NORSRAMTimingInitStructure.AddressHoldTime = 0;
    FSMC_NORSRAMTimingInitStructure.DataSetupTime = 3;    // 50 ns (DATAST+1)*12.5ns = RW length
    FSMC_NORSRAMTimingInitStructure.BusTurnAroundDuration = 0;
    FSMC_NORSRAMTimingInitStructure.CLKDivision = 0;
    FSMC_NORSRAMTimingInitStructure.DataLatency = 0;
    FSMC_NORSRAMTimingInitStructure.AccessMode = FSMC_ACCESS_MODE_A;
    FSMC_NORSRAM_Init(FSMC_Bank1, &FSMC_NORSRAMInitStructure);

    status = FSMC_NORSRAM_Timing_Init(FSMC_Bank1,  &FSMC_NORSRAMTimingInitStructure, FSMC_NORSRAM_BANK1);

    /* Enable FSMC NOR/SRAM Bank1 */
    __FSMC_NORSRAM_ENABLE(FSMC_Bank1, FSMC_NORSRAM_BANK1);

    ili9341_fsmcCommand    = (volatile uint16_t *)NOR_MEMORY_ADRESS1;   // clears A18
    ili9341_fsmcData       = (ili9341_fsmcCommand + (1 << 18));                 // sets A18

    return status;
};

void LVGLDispSTM32F407VE_BLACKInit(uint32_t nBufferRows) :
{
    // low level hardware init
    fsmc_lcd_init();

    // tft controller init
    ili9341_fsmc_init();
    ili9341_fsmc_setRotation(1);

    init();
}

static void LVGLDispSTM32F407VE_BLACK__init()
{
    size_t bufferSize = LV_HOR_RES_MAX * _nBufferRows;

    // allocate memory for display buffer
    _buf1_1 = new lv_color_t[bufferSize];             /* a buffer for n rows */
    MBED_ASSERT(_buf1_1 != nullptr);
    memset(_buf1_1, 0, bufferSize*sizeof(lv_color_t));

    /*Used to copy the buffer's content to the display*/
    _disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    _disp_drv.buffer = &_disp_buf_1;

    lv_disp_buf_init(&_disp_buf_1, _buf1_1, NULL, bufferSize);   /* Initialize the display buffer */

    /*Finally register the driver*/
    _disp = lv_disp_drv_register(&_disp_drv);
}
