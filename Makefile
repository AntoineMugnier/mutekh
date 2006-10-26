BASE_PWD:=$(PWD)
BUILD_DIR:=$(BASE_PWD)
SRC_DIR=$(BASE_PWD)

export SRC_DIR
export BUILD_DIR

all: default

$(SRC_DIR)/config.h:
	touch $@

$(SRC_DIR)/config.mk: $(SRC_DIR)/config.h
	perl scripts/config.pl

include $(SRC_DIR)/config.mk

arch/current: $(SRC_DIR)/config.mk
	ln -sfn $(CONFIG_ARCH_NAME) $@

cpu/current: $(SRC_DIR)/config.mk
	ln -sfn $(CONFIG_CPU_NAME) $@

subdirs = arch cpu drivers hexo libc

ifeq ($(CONFIG_NETWORK), defined)
 subdirs += libnetwork
endif

ifeq ($(CONFIG_PTHREAD), defined)
 subdirs += libpthread
endif

objs =

include $(SRC_DIR)/scripts/common.mk

target = kernel-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME).out

default: arch/current cpu/current $(target)

$(target): $(SRC_DIR)/config.h $(objs) $(subdirs-lists) $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME)/ldscript $(LIBAPP)
	@echo '    LD      $@'
	@$(LD) -q $$(cat /dev/null $(filter %.list,$^)) \
	$(filter %.o,$^) $(filter %.a,$^) \
	-T $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME)/ldscript \
	-o $@



