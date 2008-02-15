include $(CONF_DIR)/.config.mk
-include $(SRC_DIR)/Makefile

$(SRC_DIR)/Makefile:
	@true

DEPS=$(objs:.o=.deps)

VPATH= . $(SRC_DIR)

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
	echo '   COPY     $(1)'
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
	@echo '    AS      $(OBJ_DIR)/$@'
	$(CPP) $(INCS) $< | $(AS) $(CPUASFLAGS) -o $@

%.o: %.c
	@echo '    CC      $(OBJ_DIR)/$@'
	$(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -c \
		$< -o $@

# Extract HOST defined macros and inject values in a new header file.
# This is used by emultaion platform to get correct syscall numbers and args
%.h: %.def
	@echo ' HOST CPP   $(OBJ_DIR)/$@'
	$(HOSTCPP) $< | grep '#define' > $@

# m4 preprocessed files
%:: %.m4 $(CONF_DIR)/.config.m4
	@echo '    M4      $(OBJ_DIR)/$@'
	cat $(MUTEK_SRC_DIR)/scripts/global.m4 $(CONF_DIR)/.config.m4 \
		$< | m4 $(filter -I%,$(INCS)) -P > $@

# cpp preprocessed files
%:: %.cpp $(CONF_DIR)/.config.h
	@echo '    CPP     $(OBJ_DIR)/$@'
	$(CPP) $(INCS) $< -P -o $@

depend.mk: $(DEPS)
	@echo '    DEP     $(OBJ_DIR)/$@'
	cat /dev/null $^ > $@

%.deps: %.S
	@echo '    DEP     $(OBJ_DIR)/$@'
	$(CC) $(CFLAGS) $(DEPINC) -MM -MT $*.o -MF $@ $<

%.deps: %.c
	@echo '    DEP     $(OBJ_DIR)/$@'
	$(CC) $(CFLAGS) $(DEPINC) -MM -MT $*.o -MF $@ $<

