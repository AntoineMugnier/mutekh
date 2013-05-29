CPUTOOLS=arm-unknown-elf-

CPUCFLAGS= -fsigned-char
CPULDFLAGS=
CPUASFLAGS=

ifeq ($(CONFIG_COMPILE_DEBUG), defined)
CPUCFLAGS += -fno-dwarf2-cfi-asm
endif

ifeq ($(CONFIG_CPU_ARM_THUMB), defined)
CPUCFLAGS += -mthumb -mthumb-interwork
endif

ifeq ($(CONFIG_COMPILE_SOFTFLOAT),defined)
CPUCFLAGS+=-msoft-float -mfpu=vfp
endif

ifeq ($(CONFIG_CPU_ARM_LITTLE), defined)
CPUCFLAGS+= -mlittle-endian
CPULDFLAGS+= -EL
else
CPUCFLAGS+= -mbig-endian
CPULDFLAGS+= -EB
endif

ifeq ($(CONFIG_CPU_ARM_BIG_BE8), defined)
LINK_LDFLAGS+= --be8
endif

ifeq ($(CONFIG_ARCH_SIMPLE), defined)
TARGET_EXT=bin
endif
