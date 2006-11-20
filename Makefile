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

BASE_PWD:=$(PWD)
BUILD_DIR=$(BASE_PWD)
SRC_DIR=$(BASE_PWD)
CONF=myconfig
LIBAPP=default.o

export SRC_DIR
export BUILD_DIR
MAKEFLAGS = -s

kernel: config $(BUILD_DIR)
	echo "Using '$(CONF)' configuration file."
	echo "Using '$(LIBAPP)' application library or object file."
	$(MAKE) -C $(BUILD_DIR) -f $(SRC_DIR)/scripts/rules_main.mk kernel

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/.config.mk $(BUILD_DIR)/.config.m4 $(BUILD_DIR)/.config.h: $(CONF)
	perl $(SRC_DIR)/scripts/config.pl	\
		--input=$(CONF)			\
		--m4=$(BUILD_DIR)/.config.m4	\
		--header=$(BUILD_DIR)/.config.h	\
		--makefile=$(BUILD_DIR)/.config.mk

config: $(BUILD_DIR) $(BUILD_DIR)/.config.mk $(BUILD_DIR)/.config.m4 $(BUILD_DIR)/.config.h
	$(MAKE) -f $(SRC_DIR)/scripts/rules_links.mk

checkconfig:
	perl $(SRC_DIR)/scripts/config.pl	\
		--input=$(CONF) --check

listconfig:
	perl $(SRC_DIR)/scripts/config.pl	\
		--input=$(CONF) --list

listallconfig:
	perl $(SRC_DIR)/scripts/config.pl	\
		--input=$(CONF) --list=all

showconfig:
	perl $(SRC_DIR)/scripts/config.pl	\
		--input=$(CONF) --info=$(TOKEN)

$(CONF):
	test -f $(CONF) || ( make helpconfig ; false )

helpconfig:
	echo -e "The \`$(CONF)' source configuration file is missing."
	echo -e "Please set the CONF variable to use an alternative"
	echo -e "file or provide the missing file.\n"
	echo -e "Available configuration options can be displayed"
	echo -e "with 'make listconfig' and 'make listallconfig'.\n"
	echo -e "Informations about a specific configuration token"
	echo -e "can be displayed with 'make showconfig TOKEN=...'.\n"

clean_sub:
	$(MAKE) -f $(SRC_DIR)/scripts/rules_main.mk clean clean_sub

clean: clean_sub
	rm -f $(BUILD_DIR)/cpu/current $(BUILD_DIR)/arch/current
	rm -f $(BUILD_DIR)/.config.*

re: clean kernel

.PHONY : kernel re clean clean_sub \
	helpconfig showconfig listconfig listallconfig config

