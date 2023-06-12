#
#     This file is part of MutekH.
#     
#     MutekH is free software; you can redistribute it and/or modify it
#     under the terms of the GNU Lesser General Public License as
#     published by the Free Software Foundation; version 2.1 of the
#     License.
#     
#     MutekH is distributed in the hope that it will be useful, but
#     WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     Lesser General Public License for more details.
#     
#     You should have received a copy of the GNU Lesser General Public
#     License along with MutekH; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
#     02110-1301 USA.
#

CFLAGS=	-std=gnu99 -nostdlib -fno-builtin -foptimize-sibling-calls \
        -Wno-multichar \
        -Wall -Wmissing-prototypes -Wno-main -Wno-unused-label -O$(CONFIG_COMPILE_OPTIMIZE)

-include $(arch_SRC_DIR)/config.mk
-include $(cpu_SRC_DIR)/config.mk

CC=$(CCACHE) $(CPUTOOLS)gcc
CXX=$(CCACHE) $(CPUTOOLS)g++
DECL_FILTER_CC ?= $(CC)
HOSTCC=gcc
DEPCC?=$(CC)
CPP=$(CPUTOOLS)cpp
LD=$(CPUTOOLS)ld
AR=$(CPUTOOLS)ar
AS=$(CPUTOOLS)as
OBJCOPY=$(CPUTOOLS)objcopy
OBJDUMP=$(CPUTOOLS)objdump
LIBGCC_PATH=$(if $(LIBGCC_DIR), \
    $(dir $(shell $(CC) -print-libgcc-file-name))/$(subst $(_empty_before_space_) ,,$(LIBGCC_DIR))/libgcc.a, \
    $(shell $(CC) $(CFLAGS) $(CPUCFLAGS) -print-libgcc-file-name) )

DTC=dtc

ifeq ($(CONFIG_COMPILE_LTO), defined)
CFLAGS += -flto
LINK_LDFLAGS += -flto
endif

ifeq ($(CONFIG_OPENMP), defined)
CFLAGS += -fopenmp
endif

ifeq ($(CONFIG_GCT_CHECKING), defined)
CFLAGS += -DGCT_CONFIG_CHECKING
endif

ifeq ($(CONFIG_HET_BUILD), defined)
CFLAGS += -fno-section-anchors -ffunction-sections -fdata-sections
endif

ifeq ($(CONFIG_COMPILE_SAVETEMPS), defined)
CFLAGS += -save-temps
endif

ifeq ($(CONFIG_COMPILE_DEBUG), defined)
CFLAGS += -ggdb
endif

ifneq ($(CONFIG_COMPILE_MCPU), undefined)
CFLAGS += -mcpu=$(CONFIG_COMPILE_MCPU)
endif

ifneq ($(CONFIG_COMPILE_MARCH), undefined)
CFLAGS += -march=$(CONFIG_COMPILE_MARCH)
endif

ifeq ($(CONFIG_COMPILE_FRAMEPTR), defined)
CFLAGS += -fno-omit-frame-pointer
else
CFLAGS += -fomit-frame-pointer
endif

ifeq ($(CONFIG_COMPILE_COLLECT), defined)
CFLAGS += -ffunction-sections -fdata-sections
LINK_LDFLAGS += --gc-sections
endif

ifeq ($(CONFIG_COMPILE_MERGE), defined)
CFLAGS += -fmerge-all-constants -fmerge-constants
endif

ifeq ($(CONFIG_COMPILE_INSTRUMENT), defined)
CFLAGS += -finstrument-functions
endif

ifeq ($(CONFIG_LOAD_NO_ZERO_IN_BSS), defined)
CFLAGS += -fno-zero-initialized-in-bss
endif

INCS=-nostdinc -D__MUTEK__ \
	-I$(MUTEK_SRC_DIR)/include -I $(OBJ_DIR) \
	$(foreach mod,$(MODULE_NAMES),-I$($(mod)_SRC_DIR)/include) \
	$(foreach mod,$(MODULE_NAMES),-I$($(mod)_OBJ_DIR)/include) \
	-I$(CURRENT_DIR) \
	-I$(BUILD_DIR) \
	-I$(MUTEK_SRC_DIR) \
	-include $(OBJ_DIR)/config.h

cflags:
	@echo $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS)

ifeq ($(TARGET_MK),)
TARGET_MK=flags.mk
endif

mkmf: $(TARGET_MK)

$(TARGET_MK): $(OBJ_DIR)/config.mk
	@> $@
	@echo 'CC=$(CC)' >> $@
	@echo 'LD=$(LD)' >> $@
	@echo 'AR=$(AR)' >> $@
	@echo 'AS=$(AS)' >> $@
	@echo 'OBJCOPY=$(OBJCOPY)' >> $@
	@echo 'OBJDUMP=$(OBJDUMP)' >> $@
	@echo 'CFLAGS=$(INCS) $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS)' >> $@
	@echo 'INCS=$(INCS)' >> $@
	@echo 'LDFLAGS=$(LINK_LDFLAGS) $(LDFLAGS) $(ARCHLDFLAGS) $(CPULDFLAGS)' >> $@

.SUFFIXES:
