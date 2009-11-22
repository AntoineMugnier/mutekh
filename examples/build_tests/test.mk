MUTEK_SRC_DIR?=$(abspath $(shell pwd)/../..)
CONFIGS = $(wildcard config_*)
KERNELS:=
BUILD_DIRS:=

all: kernels

define declare_config

KERNELS+=build_$(1)/kernel
BUILD_DIRS+=build_$(1)

build_$(1):
	mkdir $$@

build_$(1)/kernel: build_$(1) $(1) FORCE
	$(MAKE) -f $(MUTEK_SRC_DIR)/Makefile \
		MUTEK_SRC_DIR=$(MUTEK_SRC_DIR) \
		CONF=$$$${PWD}/$(1) \
		BUILD_DIR=$$$${PWD}/$$<

endef

$(eval $(foreach conf,$(CONFIGS),$(call declare_config,$(conf))))

kernels: $(KERNELS)

clean:
	rm -rf $(BUILD_DIRS)

FORCE:

.PHONY: FORCE
