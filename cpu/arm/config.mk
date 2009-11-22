CPUCFLAGS= -fsigned-char
CPULDFLAGS=
CPUASFLAGS=

CPUTOOLS_GCC_VERSION:=$(shell \
	(echo '((' ; $(CC) -dumpversion | sed -e 's:\.)\?:)*100+:g' ) \
	)
CPUTOOLS_GCC_VERSION:=$(shell echo "$(CPUTOOLS_GCC_VERSION)" | bc -q)

# Dont forget the shell's 'true' is 0...
CPUTOOLS_GCC_4_4_PLUS:=$(shell test "$(CPUTOOLS_GCC_VERSION)" -lt "40400" ; echo "$$?" )

ifneq ($(CONFIG_CPU_ARM_CPU),undefined)
CPUCFLAGS+= -mcpu=$(CONFIG_CPU_ARM_CPU)
endif

ifneq ($(CONFIG_CPU_ARM_ARCH),undefined)
CPUCFLAGS+= -march=$(CONFIG_CPU_ARM_ARCH)
endif

ifeq ($(CONFIG_COMPILE_DEBUG), defined)
ifeq ($(CPUTOOLS_GCC_4_4_PLUS), 1)
CPUCFLAGS += -fno-dwarf2-cfi-asm
endif
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
