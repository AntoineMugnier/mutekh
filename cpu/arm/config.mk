#ifeq ($(CONFIG_CPU_ARM_VERSION), 7)
CPUCFLAGS= -mlittle-endian -march=armv6k -msoft-float -fsigned-char -mfloat-abi=softfp -mfpu=vfp
CPULDFLAGS= -EL
CPUASFLAGS= -mfpu=vfp
#endif

CPUTOOLS=arm-unknown-elf-

ifeq ($(CONFIG_CPU_ENDIAN_LITTLE), defined)
CPUCFLAGS+=
CPULDFLAGS+=
endif

ifeq ($(CONFIG_CPU_ENDIAN_BIG), defined)
CPUCFLAGS+=
CPULDFLAGS+=
endif

CPUCFLAGS+=

