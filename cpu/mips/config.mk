CPUCFLAGS=-mips2 -mno-branch-likely

ifeq ($(CONFIG_CPU_ENDIAN_LITTLE), defined)
CPUTOOLS=mipsel-unknown-elf-
endif

ifeq ($(CONFIG_CPU_ENDIAN_BIG), defined)
CPUTOOLS=mips-unknown-elf-
endif

