GFXINC  +=	$(GFXLIB)/boards/base/STM32F769i-Discovery \
			$(STMHAL)/Inc
GFXSRC  +=  $(GFXLIB)/boards/base/STM32F769i-Discovery/stm32f769i_discovery_sdram.c \
		    $(GFXLIB)/boards/base/STM32F769i-Discovery/STM32DSI.c \
         	$(GFXLIB)/boards/base/STM32F769i-Discovery/otm8009a.c \
			$(GFXLIB)/boards/base/STM32F769i-Discovery/stm32f7_i2c.c

include $(GFXLIB)/drivers/gdisp/STM32LTDC/driverWithDSIV1.mk
include $(GFXLIB)/drivers/ginput/touch/FT6x06/driver.mk