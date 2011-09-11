CPUCFLAGS=-fsigned-char -mflat -mno-app-regs
CPUTOOLS=sparc-unknown-elf-

ifeq ($(CONFIG_COMPILE_SOFTFLOAT), defined)
CPUCFLAGS += -msoft-float
endif

