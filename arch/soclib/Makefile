
objs = 											\
arch_init.o										\
mem_init.o

ifeq ($(CONFIG_HEXO_MMU), defined)
objs += mmu.o
endif

ifeq ($(CONFIG_SOCLIB_MEMCHECK), defined)
 objs += mem_checker.o
endif

ifeq ($(LINKING),1)

meta = ldscript

endif

ldscript: $(CURRENT_DIR)/soclib_addresses.ldscript.m4