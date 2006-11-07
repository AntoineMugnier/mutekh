BASE_PWD:=$(PWD)
BUILD_DIR:=$(BASE_PWD)
SRC_DIR=$(BASE_PWD)
LDFLAGS=
CONF=myconfig

export SRC_DIR
export BUILD_DIR

all: default

config: $(SRC_DIR)/config.mk $(SRC_DIR)/config.h

include $(SRC_DIR)/config.mk

$(SRC_DIR)/$(CONF):
	echo -e "Missing user configuration file \`$(CONF)'."
	echo -e "Please refer to the documentation."
	false

$(SRC_DIR)/config.mk $(SRC_DIR)/config.h: $(SRC_DIR)/$(CONF)
	perl $(SRC_DIR)/scripts/config.pl	\
		--input=$(SRC_DIR)/$(CONF)	\
		--header=$(SRC_DIR)/config.h	\
		--makefile=$(SRC_DIR)/config.mk

arch/current: $(SRC_DIR)/config.mk
	ln -sfn $(CONFIG_ARCH_NAME) $@

cpu/current: $(SRC_DIR)/config.mk
	ln -sfn $(CONFIG_CPU_NAME) $@

subdirs = arch cpu drivers hexo mutek libc

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
	@$(LD) $(LDFLAGS) -q $$(cat /dev/null $(filter %.list,$^)) \
	$(filter %.o,$^) $(filter %.a,$^) \
	-T $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME)/ldscript \
	-o $@

