CPUCFLAGS= -fsigned-char
CPULDFLAGS=
CPUASFLAGS=

ifneq ($(CONFIG_CPU_ARM_CPU),undefined)
CPUCFLAGS+= -mcpu=$(CONFIG_CPU_ARM_CPU)
endif

ifneq ($(CONFIG_CPU_ARM_ARCH),undefined)
CPUCFLAGS+= -march=$(CONFIG_CPU_ARM_ARCH)
endif

ifeq ($(CONFIG_CPU_ARM_FPU),soft)
CPUCFLAGS+=-msoft-float -mfloat-abi=softfp -mfpu=vfp
CPUASFLAGS+= -mfpu=vfp
endif

CPUTOOLS=arm-unknown-elf-

ifeq ($(CONFIG_CPU_ENDIAN_LITTLE), defined)
CPUCFLAGS+= -mlittle-endian
CPULDFLAGS+= -EL
endif

ifeq ($(CONFIG_CPU_ENDIAN_BIG), defined)
CPUCFLAGS+= -mbig-endian
CPULDFLAGS+= -EB
endif

ifeq ($(CONFIG_ARCH_SIMPLE), defined)
TARGET_EXT=bin
endif
