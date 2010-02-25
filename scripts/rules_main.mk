POST_TARGET=__foo.out

LDFLAGS=
TARGET_EXT ?= out

TARGET_SECTIONS=.text .data .boot .contextdata

LINKING=1
ifeq ($(TARGET_EXT),o)
LINKING=0
endif
export LINKING

OUT_NAME := $(shell cd $(MUTEK_SRC_DIR) ; perl $(MUTEK_SRC_DIR)/scripts/config.pl	\
		--path=$(CONF_PATH):.            \
		--input=$(CONF)					 \
		--build-path=$(BUILD_DIR)/obj-   \
		--build=$(BUILD) --config $(CONFIG_FLAGS) )

ifeq ($(OUT_NAME),)
$(error Configure script failed)
endif

OBJ_DIR := $(BUILD_DIR)/obj-$(OUT_NAME)
target = $(subst /,-,$(OUT_NAME))
KERNEL_FILE=$(target).$(TARGET_EXT)

include $(OBJ_DIR)/config.mk

DEP_FILE_LIST:=

include $(MUTEK_SRC_DIR)/scripts/config.mk
include $(MUTEK_SRC_DIR)/scripts/discover.mk

ifneq ($(CLEANING),1)
define do_inc_dep

ifeq ($(wildcard $(1)),$(1))
include $(1)
# else
# $# $# (info $(1) not found)
endif

endef

$(eval \
$(foreach depfile,$(DEP_FILE_LIST),\
$(call do_inc_dep,$(depfile))))

$(eval $(call do_inc_dep,$(OBJ_DIR)/config.deps))

endif

TARGET_OBJECT_LIST:=$(filter %.o,$(TARGET_OBJECT_LIST))
META_OBJECT_LIST:=$(filter-out %ldscript,$(META_OBJECT_LIST))
COPY_OBJECT_LIST:=$(filter-out %ldscript,$(COPY_OBJECT_LIST))

all: kernel

$(OBJ_DIR)/.done_pre_header_list: $(PRE_HEADER_LIST)
	@touch $@

objs:
	echo "TARGET_OBJECT_LIST = $(TARGET_OBJECT_LIST)"
	echo "DEP_FILE_LIST = $(DEP_FILE_LIST)"
	echo "CLEAN_FILE_LIST = $(CLEAN_FILE_LIST)"
	echo "PRE_HEADER_LIST = $(PRE_HEADER_LIST)"

showpaths:
	@echo MUTEK_SRC_DIR $(MUTEK_SRC_DIR)
	@echo BUILD_DIR $(BUILD_DIR)
	@echo OBJ_DIR $(OBJ_DIR)
	@echo CONF $(CONF)
	@echo target $(target)
	@echo Modules: $(MODULES)
	@echo Module names: $(MODULE_NAMES)
	@echo Module src pahs:
	@$(foreach mn,$(MODULE_NAMES),echo " " $(mn): $($(mn)_SRC_DIR); )
	@echo Module build pahs:
	@$(foreach mn,$(MODULE_NAMES),echo " " $(mn): $($(mn)_OBJ_DIR); )

.PHONY: FORCE

FORCE:

kernel: $(OBJ_DIR)/$(KERNEL_FILE)
	cp $< $(BUILD_DIR)
	@echo 'BUILD DIR   ' $(OBJ_DIR)
	@echo 'KERNEL      ' $(notdir $<)

clean:
	rm -f $(OBJ_DIR)/$(KERNEL_FILE) $(TARGET_OBJECT_LIST)
	rm -rf $(foreach mn,$(MODULE_NAMES),$($(mn)_OBJ_DIR))
	rm -f $(CONFIG_FILES)
	rm -f $(OBJ_DIR)/$(target).o
	rm -f $(OBJ_DIR)/$(target).out
	rm -f $(OBJ_DIR)/config.*
	rm -f $(OBJ_DIR)/.done_pre_header_list
	rm -f $(BUILD_DIR)/$(target).out

FINAL_LINK_TARGET?=$(OBJ_DIR)/$(target).out
FINAL_LINK_SOURCE?=$(OBJ_DIR)/$(target).o

ifeq ($(LD_NO_Q),1)

ifeq ($(CONFIG_ARCH_EMU_DARWIN),defined)
$(OBJ_DIR)/$(target).out: $(OBJ_DIR)/config.m4 \
		$(COPY_OBJECT_LIST) \
		$(META_OBJECT_LIST) \
		$(TARGET_OBJECT_LIST) \
	    FORCE
	echo '    LDL     $@'
	$(LD) $(LINK_LDFLAGS) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		$(filter %_before.o,$(filter %.o,$^)) \
		$(filter-out %_before.o %_after.o,$(filter %.o,$^)) \
		$(filter %.a,$^) \
		$(filter %_after.o,$(filter %.o,$^)) \
		-o $@ `$(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name` \
	-flat_namespace \
	-e _arch_init \
	-undefined warning

else
WL=-Wl,
$(OBJ_DIR)/$(target).out: $(OBJ_DIR)/config.m4 \
		$(COPY_OBJECT_LIST) \
		$(META_OBJECT_LIST) \
		$(TARGET_OBJECT_LIST) \
		$(arch_OBJ_DIR)/ldscript \
		$(cpu_OBJ_DIR)/ldscript \
	    FORCE
	@echo '    LDL     ' $(notdir $@)
	$(CC) $(addprefix $(WL),$(LINK_LDFLAGS) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS)) \
		$(CFLAGS) $(CPUCFLAGS) \
		$(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $@ `$(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name`
endif
else
$(FINAL_LINK_TARGET): $(FINAL_LINK_SOURCE) FORCE \
		$(arch_OBJ_DIR)/ldscript \
		$(cpu_OBJ_DIR)/ldscript
	@echo '    LD out   ' $(notdir $@)
	$(LD) $(LINK_LDFLAGS) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		$< \
		-T $(arch_OBJ_DIR)/ldscript \
		-T $(cpu_OBJ_DIR)/ldscript \
		-o $@
endif

final_link: $(FINAL_LINK_TARGET)

$(OBJ_DIR)/$(target).o: $(OBJ_DIR)/config.m4 \
		$(COPY_OBJECT_LIST) \
		$(META_OBJECT_LIST) \
        $(TARGET_OBJECT_LIST) \
	    FORCE
	@echo '    LD o     ' $(notdir $@)
	$(LD) -r \
		$(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $(filter %.o,$^) $(filter %.a,$^) \
		$(addprefix -T ,$(filter %ldscript,$^)) \
		-o $@ `$(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name`

$(OBJ_DIR)/$(target).pre.o: $(OBJ_DIR)/config.m4 $(TARGET_OBJECT_LIST) \
	    FORCE $(arch_SRC_DIR)/ldscript_obj
	@echo '    LD o     ' $(notdir $@)
	$(LD) -r \
		$(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS) \
		-q $(filter %.o,$^) $(filter %.a,$^) \
		-T $(arch_SRC_DIR)/ldscript_obj \
		-o $@ `$(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name`

kernel-postlink: $(POST_TARGET)

$(POST_TARGET): $(OBJ_DIR)/$(target).o $(POST_LDSCRIPT)
	@echo '    LD post ' $(notdir $@)
	$(LD) -o $@ --gc-sections -T $(POST_LDSCRIPT) $<

$(OBJ_DIR)/$(target).hex: $(OBJ_DIR)/$(target).out
	echo 'OBJCOPY HEX ' $(notdir $@)
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O ihex $< $@

$(OBJ_DIR)/$(target).bin: $(OBJ_DIR)/$(target).out
	echo 'OBJCOPY BIN ' $(notdir $@)
	$(OBJCOPY) $(addprefix -j ,$(TARGET_SECTIONS)) -O binary $< $@

