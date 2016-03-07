objs += fat-dirent.o fat-file.o fat-sector-cache.o fat.o

objs-$(CONFIG_DRIVER_FS_FAT16) += fat16.o
objs-$(CONFIG_DRIVER_FS_FAT32) += fat32.o

ifeq ($(CONFIG_DRIVER_FS_FAT_LFN), defined)
objs += fat-lfn.o
else
objs += fat-83.o
endif
