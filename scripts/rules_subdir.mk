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

include $(BUILD_DIR)/.config.mk
include $(SRC_DIR)/$(H)Makefile

objs_ = $(addprefix $(H), $(objs))

-include $(BUILD_DIR)/$(H)depend.mk

deps = $(foreach i, $(objs_), $(dir $(i)).$(notdir $(basename $(i))).deps)

include $(SRC_DIR)/arch/$(CONFIG_ARCH_NAME)/config.mk
include $(SRC_DIR)/cpu/$(CONFIG_CPU_NAME)/config.mk
include $(SRC_DIR)/scripts/common.mk

VPATH += $(SRC_DIR)/$(H)

DEPINC=	-nostdinc -D__MUTEK__ \
	-I include \
	$(foreach mod,$(CONFIG_MODULES), -I $(mod)/include) \
	-I arch/$(CONFIG_ARCH_NAME)/include \
	-I cpu/$(CONFIG_CPU_NAME)/include \
	-include $(BUILD_DIR)/.config.h

%depend.mk: $(deps)
	@echo '    DEP     depend.mk'
	cat /dev/null $(addprefix $(BUILD_DIR)/,$^) > $@

.%.deps: %.S
	@echo '    DEP     $(<F)'
	mkdir -p $(BUILD_DIR)/$(H)
	$(CPP) $(CFLAGS) $(DEPINC) -M -MG -MT $(basename $<).o -MF $(BUILD_DIR)/$@ $<

.%.deps: %.c
	@echo '    DEP     $(<F)'
	mkdir -p $(BUILD_DIR)/$(H)
	$(CPP) $(CFLAGS) $(DEPINC) -M -MG -MT $(basename $<).o -MF $(BUILD_DIR)/$@ $<

