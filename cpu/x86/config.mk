CPUTOOLS=i686-unknown-elf-

BCFLAGS+= -w 2

CPUCFLAGS=-mno-tls-direct-seg-refs -m32 -malign-double -Xassembler --divide
CPULDFLAGS= -melf_i386

ifeq ($(CONFIG_COMPILE_SOFTFLOAT), defined)
CPUCFLAGS += -msoft-float
endif
