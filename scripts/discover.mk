
BASE_MODULES += libc hexo mutek gpct drivers

CONF_DIR=$(BUILD_DIR)
#/obj-$(TARGET_COUPLE)

MODULES += $(CONFIG_MODULES) $(foreach mod,$(BASE_MODULES),$(mod):$(MUTEK_SRC_DIR)/$(mod))

# filter module names
MODULE_NAMES := $(foreach modwd,$(MODULES),$(shell echo $(modwd) | cut -d: -f1))

META_OBJECT_LIST:=
COPY_OBJECT_LIST:=
TARGET_OBJECT_LIST:=
DEP_FILE_LIST:=
CLEAN_FILE_LIST:=

export MODULE_NAMES

# for all modules looking like module_name:module_src_dir, export
# module_name_SRC_DIR := module_src_dir
# module_name_OBJ_DIR := BUILD_DIR/module_src_dir/obj-arch-cpu

define declare_module_dir

MKDOC_ARGS += -I $(2)/include
$(1)_SRC_DIR:=$(2)
$(1)_OBJ_DIR:=$(BUILD_DIR)/obj-$(TARGET_COUPLE)/$(1)

endef

GLOBAL_DOC_HEADERS:=

$(eval \
$(foreach modwd,$(MODULES),\
$(call declare_module_dir,$(shell echo $(modwd) | cut -d: -f1),$(shell echo $(modwd) | cut -d: -f2))))

include $(MUTEK_SRC_DIR)/scripts/local.mk

$(eval \
$(foreach modwd,$(MODULE_NAMES),\
$(call scan_local_makefile,$($(modwd)_SRC_DIR),$($(modwd)_OBJ_DIR))))
