
objs = arch_init.o

ifeq ($(CONFIG_EFM32_STK3200),defined)
objs += dev_stk3200.o
endif

ifeq ($(CONFIG_EFM32_STK3600),defined)
objs += dev_stk3600_3800.o
endif

ifeq ($(CONFIG_EFM32_STK3800),defined)
objs += dev_stk3600_3800.o
endif

ifeq ($(CONFIG_EFM32_G8XX_STK),defined)
objs += dev_g8xxstk.o
endif

ifeq ($(CONFIG_MUTEK_EARLY_CONSOLE),defined)
objs += early_console.o
endif

meta = ldscript

subdirs = drivers

