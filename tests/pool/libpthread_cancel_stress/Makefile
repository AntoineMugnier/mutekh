
objs = test.o

ifeq ($(CONFIG_MUTEK_MEM_REGION), defined)
ifeq ($(CONFIG_MUTEK_MEM_REGION_AUTO_INIT), undefined)
objs += custom_mem_init.o
endif
endif
