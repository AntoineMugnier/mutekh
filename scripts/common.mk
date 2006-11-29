#
#     This file is part of MutekH.
#
#     MutekH is free software; you can redistribute it and/or modify it
#     under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.
#
#     MutekH is distributed in the hope that it will be useful, but
#     WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with MutekH; if not, write to the Free Software Foundation,
#     Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
#

export SRC_DIR
export BUILD_DIR

VPATH = $(SRC_DIR) $(BUILD_DIR)

CFLAGS=	-fno-builtin -Wall

ifeq ($(CONFIG_COMPILE_SAVETEMPS), defined)
CFLAGS += -save-temps
endif

ifeq ($(CONFIG_COMPILE_DEBUG), defined)
CFLAGS += -O0 -gstabs #-ggdb
else
CFLAGS += -O2
endif

ifeq ($(CONFIG_COMPILE_FRAMEPTR), undefined)
CFLAGS += -fomit-frame-pointer
endif

ifeq ($(CONFIG_COMPILE_COLLECT), defined)
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS += --gc-sections
endif

ifeq ($(CONFIG_COMPILE_INSTRUMENT), defined)
CFLAGS += -finstrument-functions
endif

INCS=-nostdinc -D__MUTEK__ \
	-I$(SRC_DIR)/include \
	-I$(BUILD_DIR)/include \
	-I$(BUILD_DIR) \
	-include $(BUILD_DIR)/.config.h

%.o: %.S
	@echo '    AS      $@'
	mkdir -p $(BUILD_DIR)/$(H)
	$(CPP) $(INCS) $(SRC_DIR)/$(H)/$(<F) | $(AS) $(CPUASFLAGS) -o $(BUILD_DIR)/$(H)/$@

%.o: %.c
	@echo '    CC      $@'
	mkdir -p $(BUILD_DIR)/$(H)
	cd $(BUILD_DIR)/$(H) ; $(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -c \
		$(SRC_DIR)/$(H)/$(<F) -o $(BUILD_DIR)/$(H)/$@

%.hdef: %.def
	@echo ' HOST CPP   $(@F)'
	mkdir -p $(BUILD_DIR)/$(@D)
	$(HOSTCPP) $(SRC_DIR)/$< | grep '#define' > $(BUILD_DIR)/$@

%: %.m4
	@echo '    M4      $(@F)'
	cat $(BUILD_DIR)/.config.m4 $(SRC_DIR)/$< | m4 -P > $(BUILD_DIR)/$@

%: %.cpp
	@echo '    CPP     $(@F)'
	$(CPP) $(INCS) $(SRC_DIR)/$< -P -o $(BUILD_DIR)/$@

subdirs-lists = $(foreach name,$(subdirs),$(patsubst %,$(BUILD_DIR)$(H)/%.list,$(name)/.$(name)))
CC=$(CPUTOOLS)gcc
CPP=$(CPUTOOLS)cpp
HOSTCPP=$(CPP)
LD=$(CPUTOOLS)ld
AR=$(CPUTOOLS)ar
AS=$(CPUTOOLS)as
OBJCOPY=$(CPUTOOLS)objcopy
OBJDUMP=$(CPUTOOLS)objdump

#MAKEFLAGS += -s

print_dir:
	test -z '$(objs)' || echo $$'\n --------  $(H)  --------'

.PHONY: $(subdirs-lists) $(target) print_dir clean_sub clean kernel

$(BUILD_DIR)$(H)/.$(DIR).list: print_dir $(objs) $(subdirs-lists) $(SRC_DIR)/$(H)/Makefile
	cat /dev/null $(filter %.list,$^) > $@
	for obj in $(objs) ; do \
		echo $(BUILD_DIR)$(H)/$${obj} >> $@ ; \
	done

define recurse

$(BUILD_DIR)$(H)/$(1)/.$(1).list:
	mkdir -p $(BUILD_DIR)$(H)/$(1)
	rm -f $$@
	$$(MAKE) -C $(SRC_DIR) -f $$(SRC_DIR)/scripts/rules_subdir.mk $$@ DIR=$(1) H="$$(H)/$(1)"

endef

clean_sub:
	echo " CLEAN      $(H)"
	cd $(BUILD_DIR)$(H)/ && rm -f depend.mk .*.deps *.i *.s *.o .*.list $(clean)
	for i in $(subdirs) ; do \
		$(MAKE) -i -C $$i -f $(SRC_DIR)/scripts/rules_clean.mk H="$(H)/$$i" clean_sub; \
	done

$(eval $(foreach dirname,$(subdirs),$(call recurse,$(dirname))))

