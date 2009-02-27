include $(CONF_DIR)/.config.mk
-include $(SRC_DIR)/Makefile

$(SRC_DIR)/Makefile:
	@true

DEPS=$(filter %.deps,$(objs:.o=.deps))

VPATH= . $(SRC_DIR)

PPRINT_DIR=$(subst $(PWD)/,,$(OBJ_DIR))

to_copy=$(addprefix $(OBJ_DIR)/,$(copy))

obj.list: prepare $(objs) FORCE
#	@echo
#	@echo '  SUBDIR    $(SRC_DIR)'
	( cat $(foreach sd,$(subdirs),$(sd)/obj.list) /dev/null ; \
	  echo $(addprefix $(OBJ_DIR)/,$(objs)) \
	) > $@

prepare: $(meta) $(to_copy) $(foreach sd,$(subdirs),$(sd)/obj.list)

define declare_subdir

$(1)/obj.list: FORCE
	mkdir -p $(1)
	$(MAKE) \
		-C $(1) \
		SRC_DIR=$(SRC_DIR)/$(1) \
		OBJ_DIR=$(OBJ_DIR)/$(1) \
		-f $(MUTEK_SRC_DIR)/scripts/local.mk \
		obj.list

.PHONY: $(1)/obj.list

endef

$(eval $(foreach d,$(subdirs),$(call declare_subdir,$(d))))

define copy_one

$(OBJ_DIR)/$(1): $(SRC_DIR)/$(1)
	echo '    COPY    $(1)'
	cp $$< $$@

endef

$(eval $(foreach c,$(copy),$(call copy_one,$(c))))

clean:
	rm -f $(objs)

.PHONY: obj.list do_copy FORCE

FORCE:

include $(MUTEK_SRC_DIR)/scripts/cflags.mk
-include depend.mk

%.o: %.S
	@echo '    AS      $(PPRINT_DIR)/$@'
	$(CPP) $(INCS) $< | $(AS) $(CPUASFLAGS) -o $@

%.o: %.c
	@echo '    CC      $(PPRINT_DIR)/$@'
	$(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) $($@_CFLAGS) $(DIR_CFLAGS) -c \
		$< -o $@

# Extract HOST defined macros and inject values in a new header file.
# This is used by emultaion platform to get correct syscall numbers and args
%.h: %.def
	@echo ' HOST CPP   $(PPRINT_DIR)/$@'
	$(HOSTCPP) $< | grep '#define' > $@

# m4 preprocessed files
%:: %.m4 $(CONF_DIR)/.config.m4
	@echo '    M4      $(PPRINT_DIR)/$@'
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$< | m4 $(filter -I%,$(INCS)) -P > $@

# cpp preprocessed files
%:: %.cpp $(CONF_DIR)/.config.h
	@echo '    CPP     $(PPRINT_DIR)/$@'
	$(CPP) $(INCS) $< -P -o $@

depend.mk: $(DEPS)
	@echo '    DEP     $(PPRINT_DIR)/$@'
	cat /dev/null $^ > $@

%.deps: %.S
	@echo '    DEP     $(PPRINT_DIR)/$@'
	$(CC) $(CFLAGS) $(DEPINC) -M -MT $*.o -MF $@ $<

%.deps: %.c
	@echo '    DEP     $(PPRINT_DIR)/$@'
	$(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) \
		-M -MT $*.o -MF $@ $<

