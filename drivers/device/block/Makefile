
objs = device_block.o

ifeq ($(CONFIG_DRIVER_BLOCK_ATA), defined)
 subdirs += ata
endif

ifeq ($(CONFIG_DRIVER_BLOCK_RAMDISK), defined)
 subdirs += ramdisk
endif

ifeq ($(CONFIG_DRIVER_BLOCK_PARTITION), defined)
 subdirs += partition
endif

ifeq ($(CONFIG_DRIVER_BLOCK_SOCLIB), defined)
 subdirs += soclib
endif

ifeq ($(CONFIG_DRIVER_BLOCK_EMU), defined)
 subdirs += file-emu
endif

