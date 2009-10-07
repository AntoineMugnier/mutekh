
ifeq ($(CONFIG_ARCH_EMU_DARWIN), defined)
HOSTCPPFLAGS= -I/System/Library/Frameworks/Kernel.framework/Headers
LD_NO_Q = 1
endif

ARCHCFLAGS= -static
ARCHLDFLAGS= -static


