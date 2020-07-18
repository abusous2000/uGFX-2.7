GFXINC  += $(GFXLIB)/boards/base/STM32F407Blackboard-M4-ILI9341 \
		   $(STMHAL)/Inc
GFXSRC  +=
GFXDEFS += -DGFX_USE_OS_CHIBIOS=TRUE
#include $(GFXLIB)/drivers/ginput/touch/ADS7843/driver.mk
include $(GFXLIB)/drivers/gdisp/ILI9341/driver.mk
