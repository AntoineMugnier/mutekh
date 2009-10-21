
## declare_copy: file_name, src_dir, obj_dir

define declare_copy

#$( # info  ======== declare_copy, $(1), $(2), $(3))

$(3)/$(1): $(2)/$(1)
	echo '    COPY    $$@ $$<'
	test -d $(3) || mkdir -p $(3)
	cp $$< $$@

endef



## declare_obj: file_name, src_dir, obj_dir

define declare_obj

DEP_FILE_LIST+=$(3)/$(1:.o=.deps)

#$( # info  ======== declare_obj, $(1), $(2), $(3))

ifeq ($(wildcard $(2)/$(1:.o=.S.m4)),$(2)/$(1:.o=.S.m4))

#$$( # info  ======== declare_obj, $(1), $(2), $(3), found to be M4+ASM file)

DEP_FILE_LIST+=$(3)/$(1:.o=.m4.deps)

$(3)/$(1): $(2)/$(1:.o=.S.m4) $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 $(MUTEK_SRC_DIR)/scripts/compute_m4_deps.py
	@echo '   M4+AS    $$@'
	test -d $(3) || mkdir -p $(3)
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$$< | m4 -s $$(filter -I%,$$(INCS)) -P | \
		python $(MUTEK_SRC_DIR)/scripts/compute_m4_deps.py \
		$$@ $$(filter -I%,$$(INCS)) > $$(@:.o=.m4.deps)
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$$< | m4 $$(filter -I%,$$(INCS)) -P > $$(@:.o=.S)
	cd $(3) ; \
	$(DEPCC) $$(CFLAGS) $$(DEPINC) -M -MT $(3)/$(1) -MF $$(@:.o=.deps) $$(@:.o=.S)
	cd $(3) ; \
	$(CPP) $$(INCS) $$(@:.o=.S) | $(AS) $$(CPUASFLAGS) -o $$@

else
ifeq ($(wildcard $(2)/$(1:.o=.S)),$(2)/$(1:.o=.S))

#$$( # info  ======== declare_obj, $(1), $(2), $(3), found to be ASM file)

$(3)/$(1): $(2)/$(1:.o=.S)
	@echo '    AS      $$@'
	test -d $(3) || mkdir -p $(3)
	cd $(3) ; \
	$(DEPCC) $$(CFLAGS) $$(DEPINC) -M -MT $(3)/$(1) -MF $$(@:.o=.deps) $$<
	cd $(3) ; \
	$(CPP) $$(INCS) $$< | $(AS) $$(CPUASFLAGS) -o $$@

else
ifeq ($(wildcard $(2)/$(1:.o=.dts)),$(2)/$(1:.o=.dts))

#$$( # info  ======== declare_obj, $(1), $(2), $(3), found to be a device-tree file)

$(3)/$(1): $(2)/$(1:.o=.dts)
	@echo ' DTC->C+CC  $$@'
	test -d $(3) || mkdir -p $(3)
	cd $(3) ; $(DTC) -O dtb -o - $$< | \
		python $(MUTEK_SRC_DIR)/scripts/blob2c.py \
		-o $(3)/$(1:.o=.c) -n dt_blob_start -o $(3)/$(1:.o=.c) -
	cd $(3) ; \
	$(CC) $$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) $$(INCS) $(DIR_CFLAGS) -c \
		$(3)/$(1:.o=.c) -o $$@

else

#$$( # info  ======== declare_obj, $(1), $(2), $(3), found to be C file)

$(3)/$(1): $(2)/$(1:.o=.c) $(CONF_DIR)/.config.h
	@echo '    CC      $$@'
	test -d $(3) || mkdir -p $(3)
	cd $(3) ; \
	$(DEPCC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		$($(1)_CFLAGS) $(DIR_CFLAGS) \
		-M -MT $(3)/$(1) -MF $$(@:.o=.deps) $$<
	cd $(3) ; \
	$(CC) $$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) $$(INCS) $($(1)_CFLAGS) $(DIR_CFLAGS) -c \
		$$< -o $$@

endif
endif
endif

endef


## declare_meta_h: file_name, src_dir, obj_dir

define declare_meta_h

#$( # info  ======== declare_meta_h, $(1), $(2), $(3))

# Extract HOST defined macros and inject values in a new header file.
# This is used by emultaion platform to get correct syscall numbers and args
$(3)/$(1): $(2)/$(1:.h=.def)
	@echo ' HOST CPP   $$@'
	test -d $(3) || mkdir -p $(3)
	cat $(CONF_DIR)/.config.h $(2)/$(1:.h=.def) | \
		$(HOSTCPP) $(HOSTCPPFLAGS) > $(3)/$(1).tmp
	grep '#define' < $(3)/$(1).tmp > $(3)/$(1)

endef

## declare_meta_cpp: file_name, src_dir, obj_dir

define declare_meta_cpp

#$( # info  ======== declare_meta_cpp, $(1), $(2), $(3))

DEP_FILE_LIST+=$(3)/$(1).deps

ifeq ($(wildcard $(2)/$(1).cpp),$(2)/$(1).cpp)

# cpp preprocessed files
$(3)/$(1): $(2)/$(1).cpp $(CONF_DIR)/.config.h
	@echo '    CPP     $$@'
	test -d $(3) || mkdir -p $(3)
	$(DEPCC) -E -M -MF $$@.deps -MT $$@ $$(INCS) -P -x c $$<
	$(CC) -E $$(INCS) -P -x c - < $$< > $$@

else

# m4 preprocessed files
$(3)/$(1): $(2)/$(1).m4 $(CONF_DIR)/.config.m4 $(MUTEK_SRC_DIR)/scripts/global.m4 $(MUTEK_SRC_DIR)/scripts/compute_m4_deps.py
	@echo '    M4      $$@'
	test -d $(3) || mkdir -p $(3)
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$$< | m4 -s $$(filter -I%,$$(INCS)) -P | \
		python $(MUTEK_SRC_DIR)/scripts/compute_m4_deps.py \
		$$@ $$(filter -I%,$$(INCS)) > $$@.deps
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$$< | m4 $$(filter -I%,$$(INCS)) -P > $$@

endif

endef


## scan_local_makefile: src_dir, obj_dir

define scan_local_makefile

LOCAL_SRC_DIR:=$(1)
srcdir:=$(1)
LOCAL_OBJ_DIR:=$(2)
LOCAL_PPRINT_DIR:=$$(subst $(PWD)/,,$$(LOCAL_OBJ_DIR))

# $$( # info  ======== \
# 	scan_local_makefile \
# 	"LOCAL_SRC_DIR=$$(LOCAL_SRC_DIR)" \
# 	"LOCAL_OBJ_DIR=$$(LOCAL_OBJ_DIR)" \
# 	"LOCAL_PPRINT_DIR=$$(LOCAL_PPRINT_DIR)")

objs:=
meta:=
copy:=
subdirs:=

-include $$(LOCAL_SRC_DIR)/Makefile

#$$( # info  OBJS=$$(objs))

TARGET_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(objs))
COPY_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(copy))
META_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(meta))
CLEAN_FILE_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(objs) $$(copy) $$(meta))

$$(LOCAL_OBJ_DIR):
	mkdir -p $$@

$$(eval $$(foreach obj,$$(objs),$$(call declare_obj,$$(obj),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tocopy,$$(copy),$$(call declare_copy,$$(tocopy),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tometa,$$(filter %.h,$$(meta)),$$(call declare_meta_h,$$(tometa),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tometa,$$(filter-out %.h,$$(meta)),$$(call declare_meta_cpp,$$(tometa),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval \
$$(foreach m,$$(subdirs),\
$$(call scan_local_makefile,$$(LOCAL_SRC_DIR)/$$(m),$$(LOCAL_OBJ_DIR)/$$(m))))

endef

