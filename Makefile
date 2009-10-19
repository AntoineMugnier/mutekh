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

ifndef BUILD_DIR
BUILD_DIR:=$(shell pwd)
endif
ifdef SRC_DIR
MUTEK_SRC_DIR:=$(SRC_DIR)
else
MUTEK_SRC_DIR:=$(shell pwd)
endif
CURRENT_DIR:=$(shell pwd)
CONF=myconfig

export MUTEK_SRC_DIR
export BUILD_DIR
export CONF_DIR
export CURRENT_DIR
export MODULES

ifneq ($(VERBOSE),1)
MAKEFLAGS = -s
endif

CONF_DIR:=$(shell mktemp /tmp/mutekh_config.XXXXXX)

all: kernel

.PHONY: FORCE

mkmf: config
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk $@ TARGET_MK=$(TARGET_MK) MAKEFLAGS=$(MAKEFLAGS)

helpconfig listconfig showconfig listallconfig:
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/config.mk $@

clean:
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk $@ CLEANING=1 MAKEFLAGS=$(MAKEFLAGS)
	-rm -f $(CONF_DIR)/.config.* 2>/dev/null

FORCE::
	@true

config showpaths kernel cflags objs: FORCE
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk $@ MAKEFLAGS=$(MAKEFLAGS)

kernel-postlink:  FORCE
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/rules_main.mk $@ MAKEFLAGS=$(MAKEFLAGS) POST_LDSCRIPT=$(POST_LDSCRIPT) POST_TARGET=$(POST_TARGET)

kernel-het: 
	$(MAKE) -f $(MUTEK_SRC_DIR)/scripts/heterogeneous.mk $@ MAKEFLAGS=$(MAKEFLAGS) CONF=$(CONF)
