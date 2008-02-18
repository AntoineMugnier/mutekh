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

BUILD_DIR?=$(PWD)
CONF_DIR?=$(BUILD_DIR)
MUTEK_SRC_DIR=$(SRC_DIR)
ifeq ($(MUTEK_SRC_DIR),)
MUTEK_SRC_DIR:=$(PWD)
endif
CURRENT_DIR:=$(PWD)
CONF=myconfig

export MUTEK_SRC_DIR
export BUILD_DIR
export CONF_DIR
export CURRENT_DIR
export MODULES
MAKEFLAGS = -s

kernel: config $(BUILD_DIR) $(CONF_DIR)
	echo "Using '$(CONF)' configuration file."
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk kernel

include $(MUTEK_SRC_DIR)/scripts/config.mk

ifneq ($(BUILD_DIR),$(CONF_DIR))

$(BUILD_DIR):
	mkdir -p $@

endif

cflags:
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk $@

showpaths: config
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk $@

clean:
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk $@
	rm -f $(CONF_DIR)/.config.*

re: clean kernel

.PHONY : kernel re clean clean_sub \
	helpconfig showconfig listconfig listallconfig config

