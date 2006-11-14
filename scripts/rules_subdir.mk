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

include $(BUILD_DIR)/config.mk
include $(SRC_DIR)$(H)/Makefile
-include depend.mk

VPATH = $(SRC_DIR)$(H)

deps = $(patsubst %.o,.%.deps,$(objs))

include $(BUILD_DIR)/arch/current/config.mk
include $(BUILD_DIR)/cpu/current/config.mk
include $(SRC_DIR)/scripts/common.mk

depend.mk: $(deps)
	@echo '    DEP     depend.mk'
	cat /dev/null $(patsubst .%.deps, $(BUILD_DIR)/$(H)/.%.deps,$^) > $(BUILD_DIR)/$(H)/$@

.%.deps: %.S
	@echo '    DEP     $(<F)'
	mkdir -p $(BUILD_DIR)/$(H)
	$(CPP) $(CFLAGS) $(INCS) -M -MG -MF $(BUILD_DIR)/$(H)/$@ $<

.%.deps: %.c
	@echo '    DEP     $(<F)'
	mkdir -p $(BUILD_DIR)/$(H)
	$(CPP) $(CFLAGS) $(INCS) -M -MG -MF $(BUILD_DIR)/$(H)/$@ $<

