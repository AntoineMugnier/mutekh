
objs =

ifeq ($(CONFIG_BOARD_STM32_F401RE), defined)
objs += board_stm32_f401re.o
endif

ifeq ($(CONFIG_STM32_EARLY_CONSOLE_UART), defined)
objs += early_console.o
endif

meta = ldscript

subdirs = drivers

