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

-include $(arch_SRC_DIR)/config.mk
-include $(cpu_SRC_DIR)/config.mk

CC=$(CPUTOOLS)gcc
CPP=$(CPUTOOLS)cpp
HOSTCPP=$(CPP)
LD=$(CPUTOOLS)ld
AR=$(CPUTOOLS)ar
AS=$(CPUTOOLS)as
OBJCOPY=$(CPUTOOLS)objcopy
OBJDUMP=$(CPUTOOLS)objdump

CFLAGS=	-fno-builtin -Wall -fno-stack-protector

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
	-I$(MUTEK_SRC_DIR)/include \
	$(foreach mod,$(MODULE_NAMES),-I$($(mod)_SRC_DIR)/include) \
	$(foreach mod,$(MODULE_NAMES),-I$($(mod)_OBJ_DIR)/include) \
	-I$(CURRENT_DIR) \
	-I$(BUILD_DIR) \
	-I$(MUTEK_SRC_DIR) \
	-include $(CONF_DIR)/.config.h

cflags:
	echo $(INCS)

.SUFFIXES:
