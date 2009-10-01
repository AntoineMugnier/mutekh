TARGET_COUPLE:=$(shell \
	cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(MUTEK_SRC_DIR):$(CURRENT_DIR) \
		--input=$(CONF) --arch-cpu)

LDFLAGS=
target = kernel-$(CONFIG_ARCH_NAME)-$(CONFIG_CPU_NAME)
TARGET_EXT ?= out

TARGET_SECTIONS=.text .data .boot .contextdata

KERNEL_FILE=$(target).$(TARGET_EXT)

LINKING=1
ifeq ($(TARGET_EXT),o)
LINKING=0
endif
export LINKING

BASE_MODULES = libc hexo mutek gpct drivers

CONF_DIR=$(BUILD_DIR)
#/obj-$(TARGET_COUPLE)

include $(MUTEK_SRC_DIR)/scripts/config.mk

include $(CONF_DIR)/.config.mk

MODULES = $(CONFIG_MODULES) $(foreach mod,$(BASE_MODULES),$(mod):$(MUTEK_SRC_DIR)/$(mod))

# filter module names
MODULE_NAMES := $(foreach modwd,$(MODULES),$(shell echo $(modwd) | cut -d: -f1))

TARGET_OBJECT_LIST:=
DEP_FILE_LIST:=
CLEAN_FILE_LIST:=

export MODULE_NAMES

# for all modules looking like module_name:module_src_dir, export
# module_name_SRC_DIR := module_src_dir
# module_name_OBJ_DIR := BUILD_DIR/module_src_dir/obj-arch-cpu

define declare_module_dir

$(1)_SRC_DIR:=$(2)
$(1)_OBJ_DIR:=$(BUILD_DIR)/obj-$(TARGET_COUPLE)/$(1)

endef

$(eval \
$(foreach modwd,$(MODULES),\
$(call declare_module_dir,$(shell echo $(modwd) | cut -d: -f1),$(shell echo $(modwd) | cut -d: -f2))))

include $(MUTEK_SRC_DIR)/scripts/cflags.mk
include $(MUTEK_SRC_DIR)/scripts/local.mk

$(eval \
$(foreach modwd,$(MODULE_NAMES),\
$(call scan_local_makefile,$($(modwd)_SRC_DIR),$($(modwd)_OBJ_DIR))))

ifneq ($(CLEANING),1)
define do_inc_dep
ifeq ($(wildcard $(1)),$(1))
include $(1)
endif

endef

$(eval \
$(foreach depfile,$(DEP_FILE_LIST),\
$(call do_inc_dep,$(depfile))))
endif

all: kernel

objs:
	echo "TARGET_OBJECT_LIST = $(TARGET_OBJECT_LIST)"
	echo "DEP_FILE_LIST = $(DEP_FILE_LIST)"
	echo "CLEAN_FILE_LIST = $(CLEAN_FILE_LIST)"

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
	rm -f $(KERNEL_FILE) $(TARGET_OBJECT_LIST)
	rm -rf $(foreach mn,$(MODULE_NAMES),$($(mn)_OBJ_DIR))
	rm -f $(CONFIG_FILES)

ifeq ($(CONFIG_ARCH_SIMPLE),defined)
WL=-Wl,
$(target).out: $(CONF_DIR)/.config.m4 $(TARGET_OBJECT_LIST) \
		$(arch_OBJ_DIR)/ldscript \
		$(cpu_OBJ_DIR)/ldscript \
	    FORCE
	echo '    LDL     $@'
	$(CC) $(addprefix $(WL),$(LINK_LDFLAGS) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS)) \
		$(CFLAGS) $(CPUCFLAGS) \
		$(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $(BUILD_DIR)/$@ `$(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name`
else
$(target).out: $(CONF_DIR)/.config.m4 $(TARGET_OBJECT_LIST) \
		$(arch_OBJ_DIR)/ldscript \
		$(cpu_OBJ_DIR)/ldscript \
	    FORCE
	echo '    LD      $@'
	$(LD) $(LINK_LDFLAGS) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $(BUILD_DIR)/$@ `$(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name`
endif

$(target).o: $(CONF_DIR)/.config.m4 $(TARGET_OBJECT_LIST) \
	    FORCE
	echo '    LD      $@'
	$(LD) -r \
		$(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $(BUILD_DIR)/$@ `$(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name`


$(target).hex: $(target).out
	echo 'OBJCOPY HEX $@'
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

$(target).bin: $(target).out
	echo 'OBJCOPY BIN $@'
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O binary $(BUILD_DIR)/$< $(BUILD_DIR)/$@
