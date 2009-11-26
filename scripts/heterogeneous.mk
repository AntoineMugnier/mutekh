
HETLINK=$(MUTEK_SRC_DIR)/tools/hlink/hetlink
CONFS:=$(subst :, ,$(CONF))
COUPLES:=

define decl_conf

# Ugly Make things to make this work when using make -[pnd]
COUPLE:=$(shell unset MAKEFLAGS ; $(MAKE_COMMAND) -s -f $(MUTEK_SRC_DIR)/scripts/config.mk couple CONF=$(1))
COUPLES+=$$(COUPLE)
$$(COUPLE)_CONF:=$(1)

endef

$(eval $(foreach c,$(CONFS),$(call decl_conf,$(c))))

PRE_OBJS=$(foreach couple,$(COUPLES),$(BUILD_DIR)/kernel-$(couple).pre.o)
HET_OBJS=$(foreach couple,$(COUPLES),$(BUILD_DIR)/kernel-$(couple).pre.o.het.o)
HET_KERNELS=$(foreach couple,$(COUPLES),$(BUILD_DIR)/kernel-$(couple).het.out)

export HETLINK

kernel-het: $(HET_KERNELS)
	echo "TARGETS: $(TARGETS)"
	echo "COUPLES: $(COUPLES)"
	echo "HET_KERNELS: $(HET_KERNELS)"
	echo "PRE_OBJS: $(PRE_OBJS)"
	echo "HET_OBJS: $(HET_OBJS)"

$(BUILD_DIR)/kernel-%.pre.o: FORCE
	@echo "PRE $@"
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk \
		 MAKEFLAGS=$(MAKEFLAGS) CONF=$($(*)_CONF) \
		 BUILD_DIR=$(BUILD_DIR) TARGET_EXT=pre.o \
		 CONF_DIR=$(BUILD_DIR)/obj-$* \
		 kernel

# We have to go through an unique target or the hetlink will be done
# twice...

$(HET_OBJS): __do_hetlink FORCE

.NOPARALLEL: __do_hetlink

__do_hetlink : $(PRE_OBJS) $(HETLINK_CONF) FORCE
	echo '    HETLINK ' $(notdir $@)
	$(HETLINK) -v 4 -c $(MUTEK_SRC_DIR)/scripts/hetlink.conf $(PRE_OBJS)

$(BUILD_DIR)/kernel-%.het.out : $(BUILD_DIR)/kernel-%.pre.o.het.o FORCE
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

