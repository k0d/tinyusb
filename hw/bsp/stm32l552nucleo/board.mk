CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m33 \
  -mfloat-abi=hard \
  -mfpu=fpv5-d16 \
  -nostdlib -nostartfiles \
  -DSTM32L552xx \
  -DCFG_TUSB_MCU=OPT_MCU_STM32L5

# mcu driver causes annoying warnings
CFLAGS += -D__ARMCC_VERSION=0

# mcu driver cause following warnings
CFLAGS += -Wno-error=shadow -Wno-error=unused-parameter -Wno-error=pointer-sign

ST_HAL_DRIVER = hw/mcu/st/st_driver/STM32L5xx_HAL_Driver
ST_CMSIS = hw/mcu/st/st_driver/CMSIS/Device/ST/STM32L5xx

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/STM32L552ZETX_FLASH.ld

SRC_C += \
	$(ST_CMSIS)/Source/Templates/system_stm32l5xx.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_rcc_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_gpio.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_uart.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_uart_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_pwr.c \
	$(ST_HAL_DRIVER)/Src/stm32l5xx_hal_pwr_ex.c

SRC_S += \
	$(ST_CMSIS)/Source/Templates/gcc/startup_stm32l552xx.s

INC += \
	$(TOP)/hw/mcu/st/st_driver/CMSIS/Include \
	$(TOP)/$(ST_CMSIS)/Include \
	$(TOP)/$(ST_HAL_DRIVER)/Inc \
	$(TOP)/hw/bsp/$(BOARD)

# For TinyUSB port source
VENDOR = st
CHIP_FAMILY = stm32_fsdev

# For freeRTOS port source
FREERTOS_PORT = ARM_CM4/r0p1

# flash target using on-board stlink
flash: flash-stlink
