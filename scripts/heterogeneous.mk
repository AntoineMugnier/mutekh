
HETLINK=$(MUTEK_SRC_DIR)/tools/hlink/hetlink
CONFS:=$(subst :, ,$(CONF))
COUPLES:=

define decl_conf

COUPLE:=$(shell $(MAKE) -s -f $(MUTEK_SRC_DIR)/scripts/config.mk couple CONF=$(1))
COUPLES+=$$(COUPLE)
$$(COUPLE)_CONF:=$(1)

endef

$(eval $(foreach c,$(CONFS),$(call decl_conf,$(c))))

PRE_OBJS=$(foreach couple,$(COUPLES),$(BUILD_DIR)/kernel-$(couple).pre.o)
HET_OBJS=$(foreach couple,$(COUPLES),$(BUILD_DIR)/kernel-$(couple).pre.o.het.o)
HET_KERNELS=$(foreach couple,$(COUPLES),$(BUILD_DIR)/kernel-$(couple).het.out)

kernel-het: $(HET_KERNELS)
	echo "TARGETS: $(TARGETS)"
	echo "COUPLES: $(COUPLES)"
	echo "HET_KERNELS: $(HET_KERNELS)"
	echo "PRE_OBJS: $(PRE_OBJS)"

$(BUILD_DIR)/kernel-%.pre.o: FORCE
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk \
		 MAKEFLAGS=$(MAKEFLAGS) CONF=$($(*)_CONF) \
		 BUILD_DIR=$(BUILD_DIR) TARGET_EXT=pre.o \
		 CONF_DIR=$(BUILD_DIR)/obj-$* \
		 kernel

$(HET_OBJS): $(PRE_OBJS) $(HETLINK_CONF) FORCE $(HETLINK)
	$(HETLINK) $(PRE_OBJS)

$(BUILD_DIR)/kernel-%.het.out : $(BUILD_DIR)/kernel-%.pre.o.het.o
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk \
		 MAKEFLAGS=$(MAKEFLAGS) CONF=$($(*)_CONF) \
		 BUILD_DIR=$(BUILD_DIR) \
		 CONF_DIR=$(BUILD_DIR)/obj-$* \
		 FINAL_LINK_TARGET=$@ \
		 FINAL_LINK_SOURCE=$< \
		 final_link

clean:
	rm -f $(PRE_OBJS) $(HET_OBJS) $(HET_KERNELS)
	rm -r $(foreach c,$(COUPLES),obj-$(c))

.PHONY: FORCE

FORCE:

