
## declare_copy: file_name, src_dir, obj_dir

define declare_copy

#$( # info  ======== declare_copy, $(1), $(2), $(3))

$(3)/$(1): $(2)/$(1)
	echo '   COPY     ' $$(notdir $$@)
	test -d $(dir $(3)/$(1)) || mkdir -p $(dir $(3)/$(1))
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
	@echo '   M4+AS    ' $$(notdir $$@)
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

$(3)/$(1): $(2)/$(1:.o=.S) $(BUILD_DIR)/.done_pre_header_list
	@echo '    AS      ' $$(notdir $$@)
	test -d $(3) || mkdir -p $(3)
	cd $(3) ; \
	$(DEPCC) $$(CFLAGS) $$(DEPINC) -M -MT $(3)/$(1) -MF $$(@:.o=.deps) $$<
	cd $(3) ; \
	$(CC) $$(INCS) -c -x assembler-with-cpp $$< $$(CPUCFLAGS) -o $$@

else
ifeq ($(wildcard $(2)/$(1:.o=.dts)),$(2)/$(1:.o=.dts))

#$$( # info  ======== declare_obj, $(1), $(2), $(3), found to be a device-tree file)

$(3)/$(1): $(2)/$(1:.o=.dts) $(CONF_DIR)/.config.h
	@echo ' DTC->C+CC  ' $$(notdir $$@)
	test -d $(3) || mkdir -p $(3)
	cd $(3) ; $(DTC) -O dtb -o $(3)/$(1:.o=.blob) $$<
	cd $(3) ; python $(MUTEK_SRC_DIR)/scripts/blob2c.py \
		-o $(3)/$(1:.o=.c) -n dt_blob_start $(3)/$(1:.o=.blob)
	cd $(3) ; \
	$(CC) $$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) $$(INCS) $(DIR_CFLAGS) -c \
		$(3)/$(1:.o=.c) -o $$@

else
ifeq ($(wildcard $(2)/$(1:.o=.dict)),$(2)/$(1:.o=.dict))

#$$( # info  ======== declare_obj, $(1), $(2), $(3), found to be a forth dictionary)

$(3)/$(1): $(2)/$(1:.o=.dict)
	@echo ' DICT->C+CC ' $$(notdir $$@)
	test -d $(3) || mkdir -p $(3)
	cd $(3) ; python $(MUTEK_SRC_DIR)/scripts/blob2c.py \
		-o $(3)/$(1:.o=.c) -S -n forth_dictionary $$<
	cd $(3) ; \
	$(CC) $$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) $$(INCS) $(DIR_CFLAGS) -c \
		$(3)/$(1:.o=.c) -o $$@

else

#$$( # info  ======== declare_obj, $(1), $(2), $(3), found to be C file)

$(3)/$(1): $(2)/$(1:.o=.c) $(CONF_DIR)/.config.h $(BUILD_DIR)/.done_pre_header_list
	@echo '    CC      ' $$(notdir $$@)
	test -d $(3) || mkdir -p $(3)
	cd $(3) ; \
	$(DEPCC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		$($(1)_CFLAGS) $(DIR_CFLAGS) \
		-M -MT $(3)/$(1) -MF $$(@:.o=.deps) $$<
	cd $(3) ; \
	$(CC) $$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) $$(INCS) $($(1)_CFLAGS) $(DIR_CFLAGS) -c \
		$$< -o $$@
ifdef HETLINK
	md5=$$(shell md5sum $$< | cut -c 1-8) ; \
		rm -f $$@.static ; \
		$(CPUTOOLS)nm $$@ | grep ' t ' | cut -c 12- | sort -u | while read i ; do echo "$$$${i} _$$$${md5}_$$$${i}" >> $$@.static ; done
	if test -e $$@.static ; then \
		echo '             renaming static symbols' ; \
		$(CPUTOOLS)objcopy --redefine-syms=$$@.static $$@ ; \
	fi
endif
endif
endif
endif
endif

endef

define declare_gpct_header

$(3)/$(1): $(2)/$(1:.h=.t)
	@echo '    \\      ' $$(notdir $$@)
	@mkdir -p $(dir $(3)/$(1).h)
	cp $$< $$@
	perl $(MUTEK_SRC_DIR)/gpct/gpct/build/backslash.pl < $$< > $$@ 2> $$@.log
	sed -e 's!^warning:\([0-9]*\):!$$<:\1:warning:!g' < $$@.log 1>&2

endef

## declare_meta_h: file_name, src_dir, obj_dir

define declare_meta_h

#$( # info  ======== declare_meta_h, $(1), $(2), $(3))

# Extract HOST defined macros and inject values in a new header file.
# This is used by emultaion platform to get correct syscall numbers and args
$(3)/$(1): $(2)/$(1:.h=.def) $(CONF_DIR)/.config.h
	@echo ' HOST CPP   ' $$(notdir $$@)
	test -d $(3) || mkdir -p $(3)
	cat $(CONF_DIR)/.config.h $(2)/$(1:.h=.def) | \
		$(HOSTCC) $$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) -E - | grep '#define' > $(3)/$(1)

endef

## declare_meta_cpp: file_name, src_dir, obj_dir

define declare_meta_cpp

#$( # info  ======== declare_meta_cpp, $(1), $(2), $(3))

DEP_FILE_LIST+=$(3)/$(1).deps

ifeq ($(wildcard $(2)/$(1).cpp),$(2)/$(1).cpp)

# cpp preprocessed files
$(3)/$(1): $(2)/$(1).cpp $(CONF_DIR)/.config.h
	@echo '   CPP      ' $$(notdir $$@)
	test -d $(3) || mkdir -p $(3)
	$(DEPCC) -E -M -MF $$@.deps -MT $$@ $$(INCS) -P -x c $$<
	$(CC) -E $$(INCS) -P -x c - < $$< > $$@

else

# m4 preprocessed files
$(3)/$(1): $(2)/$(1).m4 $(CONF_DIR)/.config.m4 $(MUTEK_SRC_DIR)/scripts/global.m4 $(MUTEK_SRC_DIR)/scripts/compute_m4_deps.py
	@echo '    M4      ' $$(notdir $$@)
	test -d $(3) || mkdir -p $(3)
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$$< | m4 -s $$(filter -I%,$$(INCS)) -P | \
		python $(MUTEK_SRC_DIR)/scripts/compute_m4_deps.py \
		$$@ $$(filter -I%,$$(INCS)) > $$@.deps
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$$< | m4 $$(filter -I%,$$(INCS)) -P > $$@

endif

endef

define declare_doc_header

MKDOC_ARGS += $(1)

endef

define declare_doc_files

MKDOC_ARGS += $(2)/$(1)

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
pre_headers:=
doc_headers:=
doc_files:=

include $$(LOCAL_SRC_DIR)/Makefile

#$$( # info  OBJS=$$(objs))

TARGET_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(objs))
COPY_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(copy))
META_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(meta))
PRE_HEADER_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(pre_headers))
CLEAN_FILE_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(objs) $$(copy) $$(meta))

PRE_HEADER_LIST+=$$(filter %.h,$$(COPY_OBJECT_LIST))

$$(LOCAL_OBJ_DIR):
	mkdir -p $$@

$$(eval $$(foreach obj,$$(objs),$$(call declare_obj,$$(obj),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tocopy,$$(copy),$$(call declare_copy,$$(tocopy),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tometa,$$(filter %.h,$$(meta)),$$(call declare_meta_h,$$(tometa),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tometa,$$(filter-out %.h,$$(meta)),$$(call declare_meta_cpp,$$(tometa),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach ph,$$(pre_headers),$$(call declare_gpct_header,$$(ph),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval \
$$(foreach h,$$(doc_headers),\
$$(call declare_doc_header,$$(h))))

$$(eval \
$$(foreach f,$$(doc_files),\
$$(call declare_doc_files,$$(f),$$(LOCAL_SRC_DIR))))

# Beware this must be left last in calls

$$(eval \
$$(foreach m,$$(subdirs),\
$$(call scan_local_makefile,$$(LOCAL_SRC_DIR)/$$(m),$$(LOCAL_OBJ_DIR)/$$(m))))

endef

