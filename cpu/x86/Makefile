
objs = boot.o cpu_init.o driver.o cpu_context.o interrupts.o context_switch.o

objs-$(CONFIG_HEXO_MMU) += mmu.o

subdirs = libc

subdirs-$(CONFIG_LIBELF) += libelf
subdirs-$(CONFIG_NETWORK) += libnetwork
subdirs-$(CONFIG_LIBUNIX) += libunix

copy = ldscript

