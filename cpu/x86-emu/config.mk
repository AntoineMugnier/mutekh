
CPUCFLAGS=-mno-tls-direct-seg-refs
ifeq ($(CONFIG_ARCH_EMU_DARWIN), defined)
CPUTOOLS=i686-unknown-elf-
DEPCC=gcc
else
CPUTOOLS=
endif
