
ifdef HETLINK
define do_hetlink_mangling
	md5=$(shell md5sum $< | cut -c 1-8) ; \
		rm -f $@.static ; \
		$(CPUTOOLS)nm $@ | grep ' t ' | cut -c 12- | sort -u | while read i ; do echo "$${i} _$${md5}_$${i}" >> $@.static ; done
	if test -e $@.static ; then \
		echo '             renaming static symbols' $(LOG_REDIR) ; \
		$(CPUTOOLS)objcopy --redefine-syms=$@.static $@ $(LOG_REDIR) ; \
	fi
endef
endif

# prepare_command msg file
# creates the output directory parent of file
define echo_command
	@echo '$(1)	$$(notdir $(value 2))' $$(LOG_REDIR)
endef

define mkdir_command
	(test -d $$(dir $(value 1)) || mkdir -p $$(dir $(value 1))) $$(LOG_REDIR)
endef

define prepare_command
	$(call echo_command,$(1),$(2))
	$(call mkdir_command,$(2))
endef

# run_command dest_file cmd
# runs a command in destination directory, logging the output
define run_command
	( cd $$(dir $(value 1)) ; \
	    $(value 2) \
	) $(LOG_REDIR)
endef

# compute_depfile_c depfile target input [flags]
# runs gcc -M to compute dependancy makefile
# prepend obj dir to deps which are not found yet (generated files)
define compute_depfile_c
	( cd $$(dir $(value 1)) ; \
		$(DEPCC) \
			$$(CFLAGS) $$(DEPINC) $(value 4) \
			-M -MG -MT $(value 2) -x c $(value 3) \
                        | perl -ne 's:\s(\w): $$(dir $(value 1))\1:g; print;' > $(value 1) \
	) $(LOG_REDIR)
endef

# compile compiler target input [flags]
# runs compiler to build object
define compile
	( cd $$(dir $(value 2)) ; \
		$(value 1) -c  \
			$$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) $$(INCS) -I $$(dir $(value 1)) \
			$(value 4) $(value 3) -o $(value 2) \
	) $(LOG_REDIR)
endef

# blob2c c_file blob symbol_name
# runs blob2c.pl to build c file
define blob2c
	( cd $$(dir $(value 1)) ; \
		perl $(MUTEK_SRC_DIR)/scripts/blob2c.pl \
	    -a 4 -o $(value 1) -S -n $(value 3) $(value 2) \
	) $(LOG_REDIR)
endef

## declare_copy: file_name, src_dir, obj_dir

define declare_copy

#$( # info  ======== declare_copy, $(1), $(2), $(3))

$(3)/$(1): $(2)/$(1)
	$(call prepare_command,COPY,$$@)
	$(call run_command,$$@,cp $$< $$@)

endef


define declare_external_hg

$(info Updating external repository $(3) ...)
$(shell cd $(4) ; test -d $(1)_$(2) || hg clone $(3) $(1)_$(2) -u $(2))
$(shell cd $(4) ; ln -sf $(1)_$(2) $(1))

endef


## declare_obj: file_name, src_dir, obj_dir

define declare_obj

#$( # info  ======== declare_obj, $(1), $(2), $(3))

ifneq ($(wildcard $(2)/$(1:.o=.S)),) ################################################# Assembly file

$(3)/$(1:.o=.deps): $(2)/$(1:.o=.S) $(OBJ_DIR)/.done_pre_header_list $(OBJ_DIR)/config.h
	$(call mkdir_command,$$@)
	$(call compute_depfile_c,$$(@:.o=.deps),$(3)/$(1),\
		-x assembler-with-cpp $$<,\
		$(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		$($(1)_CFLAGS) $(DIR_CFLAGS) )

DEPS_LIST +=  $(3)/$(1:.o=.deps)

$(3)/$(1): $(2)/$(1:.o=.S)
	$(call echo_command,AS,$$@)
	$(call run_command,$$@, $(CC) -E \
                $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -I$(3) \
                $($(1)_CFLAGS) $(DIR_CFLAGS) $$< -o $$@.i )
	$(call run_command,$$@, perl $(MUTEK_SRC_DIR)/scripts/decl_filter.pl --filter-gnuasm --parse-decl $(CC) \
                $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -I$(3) \
                $($(1)_CFLAGS) $(DIR_CFLAGS) < $$@.i > $$@.s)
	$(call compile,$(CC),$$@, -x assembler-with-cpp $$@.s, $($(1)_CFLAGS) $(DIR_CFLAGS))

else ifneq ($(wildcard $(2)/$(1:.o=.dts)),) ######################################### device-tree file)

$(3)/$(1): $(2)/$(1:.o=.dts) $(OBJ_DIR)/config.h
	$(call prepare_command,DTC,$$@)
	$(call run_command,$$@,m4 -P $(MUTEK_SRC_DIR)/scripts/global.m4 $(OBJ_DIR)/config.m4 $$< | $(DTC) -O dtb -o $(3)/$(1:.o=.blob))
	$(call blob2c,$$(@:.o=.c),$$(@:.o=.blob),dt_blob_start)
	$(call compile,$(CC),$$@,$$(@:.o=.c))

else ifneq ($(wildcard $(2)/$(1:.o=.dict)),) ######################################## forth dictionary)

$(3)/$(1): $(2)/$(1:.o=.dict)
	$(call prepare_command,DICT,$$@)
	$(call blob2c,$$(@:.o=.c),$$<,forth_dictionary)
	$(call compile,$(CC),$$@,$$(@:.o=.c))

else ifneq ($(wildcard $(2)/$(1:.o=.cc))$(wildcard $(2)/$(1:.o=.cpp)),) ############## C++ file

$(3)/$(1:.o=.deps): $(wildcard $(2)/$(1:.o=.cc))$(wildcard $(2)/$(1:.o=.cpp)) $(OBJ_DIR)/config.h $(OBJ_DIR)/.done_pre_header_list
	$(call mkdir_command,$$@)
	$(call compute_depfile_c,$$(@:.o=.deps),$(3)/$(1),$$<,$(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		$($(1)_CXXFLAGS) $(DIR_CXXFLAGS))

DEPS_LIST +=  $(3)/$(1:.o=.deps)

$(3)/$(1): $(wildcard $(2)/$(1:.o=.cc))$(wildcard $(2)/$(1:.o=.cpp))
	$(call echo_command,C++,$$@)
	$(call compile,$(CXX),$$@,$$<,$($(1)_CXXFLAGS) $(DIR_CXXFLAGS) -DMUTEK_CFILE='"$$(<F)"')
	$(value do_hetlink_mangling)

else ifneq ($(wildcard $(2)/$(1:.o=.bc)),) ########################################### bytecode file

$(3)/$(1:.o=.deps): $(2)/$(1:.o=.bc) $(OBJ_DIR)/config.h $(OBJ_DIR)/.done_pre_header_list
	$(call mkdir_command,$$@)
	$(call compute_depfile_c,$$(@:.o=.deps),$(3)/$(1),$$<,$(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		$($(1)_CFLAGS) $(DIR_CFLAGS))

DEPS_LIST +=  $(3)/$(1:.o=.deps)

$(3)/$(1): $(2)/$(1:.o=.bc)
	$(call echo_command,BC,$$@)
	$(call run_command,$$@, $(CC) -E -x c \
                $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -I$(3) \
                $($(1)_CFLAGS) $(DIR_CFLAGS) -DMUTEK_CFILE='"$$(<F)"' $$< -o $$@.i )
	$(call run_command,$$@, perl $(MUTEK_SRC_DIR)/scripts/decl_filter.pl --parse-decl $(CC) \
                $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -I$(3) \
                $($(1)_CFLAGS) $(DIR_CFLAGS) < $$@.i > $$@.bc)
	$(call run_command,$$@, perl $(MUTEK_SRC_DIR)/scripts/bc_asm.pl $(BCPATH) $(BCFLAGS) -h $$@.h -o $$@.s < $$@.bc )
	$(call compile,$(CC),$$@,$$@.s,$($(1)_CFLAGS) $(DIR_CFLAGS))
	$(value do_hetlink_mangling)

$(3)/$(1).h: $(3)/$(1)
	touch $$@

else ifneq ($(wildcard $(2)/$(1:.o=.c)),) ########################################### C file

$(3)/$(1:.o=.deps): $(2)/$(1:.o=.c) $(OBJ_DIR)/config.h $(OBJ_DIR)/.done_pre_header_list
	$(call mkdir_command,$$@)
	$(call compute_depfile_c,$$@,$(3)/$(1),$$<,$(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		$($(1)_CFLAGS) $(DIR_CFLAGS))

DEPS_LIST +=  $(3)/$(1:.o=.deps)

$(3)/$(1): $(2)/$(1:.o=.c)
	$(call echo_command,CC,$$@)
	$(call compile,$(CC),$$@,$$<,$($(1)_CFLAGS) $(DIR_CFLAGS) -DMUTEK_CFILE='"$$(<F)"')
	$(value do_hetlink_mangling)

else ifneq ($(wildcard $(2)/$(1:.o=.t)),) ########################################### template C file

$(3)/$(1:.o=.deps): $(2)/$(1:.o=.t) $(OBJ_DIR)/config.h $(OBJ_DIR)/.done_pre_header_list
	$(call mkdir_command,$$@)
	$(call compute_depfile_c,$$@,$(3)/$(1),$$<,$(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		$($(1)_CFLAGS) $(DIR_CFLAGS))

include $(3)/$(1:.o=.deps)

$(3)/$(1): $(2)/$(1:.o=.t)
	$(call echo_command,TC,$$@)
	$(call run_command,$$@, perl $(MUTEK_SRC_DIR)/gct/gct/build/backslash.pl $$< $$@.c )
	$(call compile,$(CC),$$@,$$@.c,$($(1)_CFLAGS) $(DIR_CFLAGS) -DMUTEK_CFILE='"$$(<F)"')
	$(value do_hetlink_mangling)

endif

endef

define declare_gct_header

$(3)/$(1): $(2)/$(1:.h=.t)
	$(call prepare_command,\\,$$@)
	cp $$< $$@ $(LOG_REDIR)
	perl $(MUTEK_SRC_DIR)/gct/gct/build/backslash.pl $$< $$@ 2>> $(LOG_FILE)
#	sed -e 's!^warning:\([0-9]*\):!$$<:\1:warning:!g' < $(LOG_FILE) 1>&2

endef

## declare_meta_cpp: file_name, src_dir, obj_dir

define declare_meta_cpp

#$( # info  ======== declare_meta_cpp, $(1), $(2), $(3))

ifeq ($(wildcard $(2)/$(1).cpp),$(2)/$(1).cpp)

# cpp preprocessed files
$(3)/$(1).deps: $(2)/$(1).cpp $(OBJ_DIR)/config.h
	$(call mkdir_command,$$@)
	$(DEPCC) -E -M -MG -MF $$@ -MT $(3)/$(1) $$(INCS) -P -x c $$<

DEPS_LIST +=  $(3)/$(1).deps

$(3)/$(1): $(2)/$(1).cpp
	$(call echo_command,CPP,$$@)
	$(CC) $$(CFLAGS) $$(CPUCFLAGS) $$(ARCHCFLAGS) -E $$(INCS) -P -x c $$< -o $$@

else

# m4 preprocessed files
$(3)/$(1).deps: $(2)/$(1).m4 $(OBJ_DIR)/config.m4 $(MUTEK_SRC_DIR)/scripts/global.m4
	$(call mkdir_command,$$@)
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(OBJ_DIR)/config.m4 \
		$$< | m4 -s $$(filter -I%,$$(INCS)) -P | \
		perl $(MUTEK_SRC_DIR)/scripts/compute_m4_deps.pl \
		$$@ $$(filter -I%,$$(INCS)) > $$@

DEPS_LIST +=  $(3)/$(1).deps

$(3)/$(1): $(2)/$(1).m4
	$(call echo_command,M4,$$@)
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(OBJ_DIR)/config.m4 \
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
pre_headers:=
enum_headers:=
objs-defined:=
meta-defined:=
copy-defined:=
subdirs-defined:=
pre_headers-defined:=
enum_headers-defined:=
ext_hg:=

include $$(LOCAL_SRC_DIR)/Makefile

$$(eval $$(if $$(findstring -,$$(objs)), $$(warning '-' is not allowed in source file names: $$(objs))))

TARGET_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(objs) $$(objs-defined))
COPY_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(copy) $$(copy-defined))
META_OBJECT_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(meta) $$(meta-defined))
PRE_HEADER_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(pre_headers) $$(pre_headers-defined))
ENUM_HEADER_LIST+=$$(addprefix $$(LOCAL_SRC_DIR)/,$$(enum_headers) $$(enum_headers-defined))
CLEAN_FILE_LIST+=$$(addprefix $$(LOCAL_OBJ_DIR)/,$$(objs) $$(copy) $$(meta) $$(objs-defined) $$(copy-defined) $$(meta-defined))

PRE_HEADER_LIST+=$$(filter %.h,$$(COPY_OBJECT_LIST))

$$(LOCAL_OBJ_DIR):
	mkdir -p $$@

$$(eval $$(if $$(ext_hg),$$(call declare_external_hg,$$(word 1, $$(ext_hg)),$$(word 2, $$(ext_hg)),$$(wordlist 3,99,$$(ext_hg)),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach obj,$$(objs) $$(objs-defined),$$(call declare_obj,$$(obj),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tocopy,$$(copy) $$(copy-defined),$$(call declare_copy,$$(tocopy),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tometa,$$(filter %.h,$$(meta) $$(meta-defined)),$$(call declare_meta_h,$$(tometa),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach tometa,$$(filter-out %.h,$$(meta) $$(meta-defined)),$$(call declare_meta_cpp,$$(tometa),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

$$(eval $$(foreach ph,$$(pre_headers) $$(pre_headers-defined),$$(call declare_gct_header,$$(ph),$$(LOCAL_SRC_DIR),$$(LOCAL_OBJ_DIR))))

# Beware this must be left last in calls

$$(eval \
$$(foreach m,$$(subdirs) $$(subdirs-defined),\
$$(call scan_local_makefile,$$(LOCAL_SRC_DIR)/$$(m),$$(LOCAL_OBJ_DIR)/$$(m))))

endef

