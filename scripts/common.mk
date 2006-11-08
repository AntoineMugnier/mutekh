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

CFLAGS=	-fno-builtin -Wall

ifeq ($(CONFIG_COMPILE_DEBUG), defined)
CFLAGS += -O0 -ggdb
else
CFLAGS += -O2 -fomit-frame-pointer
endif

ifeq ($(CONFIG_COMPILE_COLLECT), defined)
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS += --gc-sections
endif

ifeq ($(CONFIG_COMPILE_INSTRUMENT), defined)
CFLAGS += -finstrument-functions
endif

include $(SRC_DIR)/arch/current/config.mk
include $(SRC_DIR)/cpu/current/config.mk

INCS=-nostdinc -D__TEST__ -D__MUTEK__ -I$(SRC_DIR)/include -include $(SRC_DIR)/config.h

%.o: %.S
	@echo '    AS      $@'
	@$(CPP) $(INCS) $< | $(AS) -o $@

%.o: %.c
	@echo '    CC      $@'
	@$(CC) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -c $< -o $@

subdirs-lists = $(foreach name,$(subdirs),$(patsubst %,.%.list,$(name)))
CC=$(CPUTOOLS)gcc
CPP=$(CPUTOOLS)cpp
LD=$(CPUTOOLS)ld
AR=$(CPUTOOLS)ar
AS=$(CPUTOOLS)as
OBJCOPY=$(CPUTOOLS)objcopy
OBJDUMP=$(CPUTOOLS)objdump

MAKEFLAGS += -s

clean:
	@echo " CLEAN      $(H)"
	@rm -f $target *~ depend.mk $(objs) $(subdirs-lists) *.map
	@rm -rf .depends
	@for i in $(subdirs) ; do \
		make -C $$i -f $(SRC_DIR)/scripts/rules.mk H="$(H)/$$i" clean; \
	done

print_dir:
	@ test -z '$(objs)' || echo $$'\n --------  $(H)  --------'

.PHONY: $(subdirs-lists) $(target)

$(HH)/.$(DIR).list: print_dir $(objs) $(subdirs-lists) $(SRC_DIR)$(H)/Makefile
	@rm -f $@
	@for obj in $(filter %.o,$^) \
			$$(cat /dev/null $(filter %.list,$^)) ; do \
		echo $(DIR)/$${obj} >> $@ ; \
	done

define recurse

.$(1).list: $$(SRC_DIR)$$(H)/$(1)/Makefile
	@test -d $(1) || mkdir -p $(1)
	@rm -f $$@
	@$$(MAKE) -C $(1) -f $$(SRC_DIR)/scripts/rules.mk $$(SRC_DIR)$$(H)/$$@ DIR=$(1) H="$$(H)/$(1)" HH="$$(SRC_DIR)$$(H)"

endef

$(eval $(foreach dirname,$(subdirs),$(call recurse,$(dirname))))

re: clean default

