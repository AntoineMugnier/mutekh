
subdirs = libc drivers

ifeq ($(CONFIG_CPU_ARM_ARCH_PROFILE_A), defined)
objs = context_switch.o cpu_context.o boot.o exception.o driver.o
ifeq ($(CONFIG_CPU_ARM_THUMB), defined)
objs += thumb_helpers.o
endif
endif

ifeq ($(CONFIG_CPU_ARM_ARCH_PROFILE_M), defined)
objs = context_switch_m.o cpu_context_m.o boot_m.o exception_m.o driver_m.o
endif

copy = ldscript
