CPUTOOLS=x86_64-unknown-elf-

BCFLAGS+= -w 3

CPUCFLAGS=-mno-tls-direct-seg-refs

ifeq ($(CONFIG_COMPILE_SOFTFLOAT), defined)
CPUCFLAGS += -msoft-float
endif
