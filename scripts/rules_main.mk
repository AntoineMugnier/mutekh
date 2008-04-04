
LDFLAGS=
target = kernel-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)
TARGET_EXT=out

TARGET_SECTIONS=.text .data .boot .contextdata

KERNEL_FILE=$(target).$(TARGET_EXT)

LINKING=1
ifeq ($(TARGET_EXT),o)
LINKING=0
endif
export LINKING

BASE_MODULES = libc hexo mutek gpct drivers

-include $(CONF_DIR)/.config.mk

MODULES = $(CONFIG_MODULES) $(foreach mod,$(BASE_MODULES),$(mod):$(MUTEK_SRC_DIR)/$(mod))

# filter module names
MODULE_NAMES := $(foreach modwd,$(MODULES),$(shell echo $(modwd) | cut -d: -f1))

export MODULE_NAMES

# for all modules looking like module_name:module_src_dir, export
# module_name_SRC_DIR := module_src_dir
# module_name_OBJ_DIR := BUILD_DIR/module_src_dir/obj-arch-cpu

define declare_module_dir

$(1)_SRC_DIR:=$(2)
$(1)_OBJ_DIR:=$(BUILD_DIR)/$(1)/obj-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)

export $(1)_SRC_DIR
export $(1)_OBJ_DIR

$(BUILD_DIR)/$(1)/obj-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)/obj.list: LIST_SRC_DIR=$(realpath $(2))
$(BUILD_DIR)/$(1)/obj-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)/obj.list: LIST_OBJ_DIR=$(BUILD_DIR)/$(1)/obj-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)

endef

$(eval \
$(foreach modwd,$(MODULES),\
$(call declare_module_dir,$(shell echo $(modwd) | cut -d: -f1),$(shell echo $(modwd) | cut -d: -f2))))

LISTS=$(foreach mn,$(MODULE_NAMES),$($(mn)_OBJ_DIR)/obj.list)

# Ok, ended with horrible things

include $(MUTEK_SRC_DIR)/scripts/cflags.mk

showpaths:
	@echo MUTEK_SRC_DIR $(MUTEK_SRC_DIR)
	@echo BUILD_DIR $(BUILD_DIR)
	@echo CONF_DIR $(CONF_DIR)
	@echo Modules: $(MODULES)
	@echo Module names: $(MODULE_NAMES)
	@echo Module src pahs:
	@$(foreach mn,$(MODULE_NAMES),echo " " $(mn): $($(mn)_SRC_DIR); )
	@echo Module build pahs:
	@$(foreach mn,$(MODULE_NAMES),echo " " $(mn): $($(mn)_OBJ_DIR); )

.PHONY: FORCE

FORCE:

kernel: $(KERNEL_FILE)

clean:
	rm -f $(KERNEL_FILE)
	rm -rf $(foreach mn,$(MODULE_NAMES),$($(mn)_OBJ_DIR))

%/obj.list: FORCE
	mkdir -p "$(LIST_OBJ_DIR)"
	$(MAKE) \
		-C $(LIST_OBJ_DIR) \
		SRC_DIR=$(LIST_SRC_DIR) \
		OBJ_DIR=$(LIST_OBJ_DIR) \
		-f $(MUTEK_SRC_DIR)/scripts/local.mk \
		obj.list

$(target).out: $(CONF_DIR)/.config.m4 $(LISTS) \
		$(arch_OBJ_DIR)/ldscript \
		$(cpu_OBJ_DIR)/ldscript
	echo '    LD      $@'
	$(LD) $(LINK_LDFLAGS) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $$(cat /dev/null $(filter %.list,$^)) \
		$(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $(BUILD_DIR)/$@

$(target).o: $(CONF_DIR)/.config.m4 $(LISTS)
	echo '    LD      $@'
	$(LD) -r \
		$(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $$(cat /dev/null $(filter %.list,$^)) \
		$(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $(BUILD_DIR)/$@

$(target).hex: $(KERNEL_FILE)
	echo 'OBJCOPY HEX $@'
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

$(target).bin: $(KERNEL_FILE)
	echo 'OBJCOPY BIN $@'
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O binary $(BUILD_DIR)/$< $(BUILD_DIR)/$@
