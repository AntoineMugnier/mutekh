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

CFLAGS=	-fno-builtin -Wall

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
	$(CPP) $(INCS) $(SRC_DIR)/$(H)/$(<F) | $(AS) -o $(BUILD_DIR)/$(H)/$@

%.o: %.c
	@echo '    CC      $@'
	mkdir -p $(BUILD_DIR)/$(H)
	$(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -c \
		$(SRC_DIR)/$(H)/$(<F) -o $(BUILD_DIR)/$(H)/$@

# cpp -MG do not produce the same dep (w and w/o full path) if
# file is present or not a compile time
%.hdef: %.def
	@echo '    CPP     $@'
	mkdir -p $(BUILD_DIR)/$(subst $(BUILD_DIR),,$(@D))
	$(CPP) $(SRC_DIR)/$(subst $(BUILD_DIR),,$<) | grep '#define' > $(BUILD_DIR)/$(subst $(BUILD_DIR),,$@)

% : %.m4
	m4 $<

subdirs-lists = $(foreach name,$(subdirs),$(patsubst %,$(BUILD_DIR)$(H)/%.list,$(name)/.$(name)))
CC=$(CPUTOOLS)gcc
CPP=$(CPUTOOLS)cpp
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

$(BUILD_DIR)$(H)/$(1)/.$(1).list: $$(SRC_DIR)/$$(H)/$(1)/Makefile
	mkdir -p $(BUILD_DIR)$(H)/$(1)
	rm -f $$@
	$$(MAKE) -C $(1) -f $$(SRC_DIR)/scripts/rules_subdir.mk $$@ DIR=$(1) H="$$(H)/$(1)"

endef

clean_sub:
	echo " CLEAN      $(H)"
	cd $(BUILD_DIR)$(H)/ && rm -f depend.mk .*.deps $(objs) $(subdirs-lists)
	for i in $(subdirs) ; do \
		$(MAKE) -i -C $$i -f $(SRC_DIR)/scripts/rules_clean.mk H="$(H)/$$i" clean_sub clean; \
	done

$(eval $(foreach dirname,$(subdirs),$(call recurse,$(dirname))))

