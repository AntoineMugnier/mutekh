#!/usr/bin/make -f
#
#    Cross compiler generation script
#
#    This file is part of MutekH.
#    
#    MutekH is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Lesser General Public License as
#    published by the Free Software Foundation; version 2.1 of the
#    License.
#    
#    MutekH is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Lesser General Public License for more details.
#    
#    You should have received a copy of the GNU Lesser General Public
#    License along with MutekH; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
#    02110-1301 USA.
#
#    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2009-2011
#

#### LINE 25 IS HERE ####

# Target architecture
TARGET=mipsel

# Install PATH
PREFIX=/opt/mutekh

# Temp directory
WORKDIR=/tmp/crossgen

# GNU Binutils
binutils_VER_mipsel  = 2.20.1
binutils_VER_powerpc = 2.20.1
binutils_VER_arm     = 2.20.1
binutils_VER_i686    = 2.20.1
binutils_VER_x86_64  = 2.20.1
binutils_VER_nios2   = 2.20.1

binutils_VER=$(binutils_VER_$(TARGET))
binutils_CONF=

# GNU Compiler
gcc_VER_mipsel  = 4.5.2
gcc_VER_powerpc = 4.5.2
gcc_VER_arm     = 4.5.2
gcc_VER_i686    = 4.5.2
gcc_VER_x86_64  = 4.5.2
gcc_VER_nios2   = 4.4.4

gcc_VER=$(gcc_VER_$(TARGET))
gcc_CONF=--enable-languages=c --disable-libssp --enable-multilib

# GCC requirements
mpfr_VER=2.4.2
gmp_VER=4.3.2
mpc_VER=0.9

# GNU Debugger
gdb_VER_mipsel  = 7.2
gdb_VER_powerpc = 7.2
gdb_VER_arm     = 7.2
gdb_VER_i686    = 7.2
gdb_VER_x86_64  = 7.2
gdb_VER_nios2   = 7.0
gdb_VER=$(gdb_VER_$(TARGET))
gdb_CONF=

# Device Tree Compiler
dtc_VER=1.2.0




#### LINE 79 IS HERE ####

# packages configurations

binutils_URL=ftp://ftp.gnu.org/gnu/binutils/binutils-$(binutils_VER).tar.bz2
binutils_TGZ=$(WORKDIR)/binutils-$(binutils_VER).tar.bz2
binutils_TESTBIN=bin/$(TARGET)-unknown-elf-as

gcc_URL=ftp://ftp.gnu.org/gnu/gcc/gcc-$(gcc_VER)/gcc-$(gcc_VER).tar.bz2
gcc_TGZ=$(WORKDIR)/gcc-$(gcc_VER).tar.bz2
gcc_TESTBIN=bin/$(TARGET)-unknown-elf-gcc
gcc_DEPS=binutils mpfr gmp mpc
gcc_CONF+=--with-mpfr=$(PREFIX) --with-gmp=$(PREFIX) --with-mpc=$(PREFIX)

gdb_URL=ftp://ftp.gnu.org/gnu/gdb/gdb-$(gdb_VER).tar.bz2
gdb_TGZ=$(WORKDIR)/gdb-$(gdb_VER).tar.bz2
gdb_TESTBIN=bin/$(TARGET)-unknown-elf-gdb

mpfr_URL=ftp://ftp.gnu.org/gnu/mpfr/mpfr-$(mpfr_VER).tar.bz2
mpfr_TGZ=$(WORKDIR)/mpfr-$(mpfr_VER).tar.bz2
mpfr_TESTBIN=lib/libmpfr.a

gmp_URL=ftp://ftp.gnu.org/gnu/gmp/gmp-$(gmp_VER).tar.bz2
gmp_TGZ=$(WORKDIR)/gmp-$(gmp_VER).tar.bz2
gmp_TESTBIN=lib/libgmp.a

mpc_URL=http://www.multiprecision.org/mpc/download/mpc-$(mpc_VER).tar.gz
mpc_TGZ=$(WORKDIR)/mpc-$(mpc_VER).tar.gz
mpc_TESTBIN=lib/libmpc.a
mpc_DEPS=mpfr gmp
mpc_CONF+=--with-mpfr=$(PREFIX) --with-gmp=$(PREFIX)

dtc_URL=https://www.mutekh.org/www/tools/dtc-$(dtc_VER).tar.gz
dtc_TGZ=$(WORKDIR)/dtc-$(dtc_VER).tar.gz
dtc_TESTBIN=bin/dtc

WGET_OPTS=-c -t 5 -w 5 --no-check-certificate

$(shell mkdir -p $(WORKDIR))

# main rules

help:
	@echo Available targets are: all, gcc, binutils, gdb, dtc
	@echo Default configuration is:
	@head $(MAKEFILE_LIST) -n 78 | tail -n 53

all: gcc binutils gdb dtc

% : %.tar.bz2
	( mkdir -p $@ ; cd $@/.. ; tar xjf $< )
	touch $@

% : %.tar.gz
	( mkdir -p $@ ; cd $@/.. ; tar xzf $< )
	touch $@

# template rules for tools which depend on target processor

define TGTTOOL_template

.PHONY: $(1)
.PRECIOUS: $$($(1)_TGZ)
.DELETE_ON_ERROR: $$($(1)_STAMP)-wget $$($(1)_STAMP)-$$(TARGET)-conf $$($(1)_STAMP)-$$(TARGET)-build $$($(1)_STAMP)-$$(TARGET)-patch

$(1)_DIR=$$(WORKDIR)/$(1)-$$($(1)_VER)
$(1)_BDIR=$$(WORKDIR)/$(1)-bld-$$(TARGET)-$$($(1)_VER)
$(1)_STAMP=$$(WORKDIR)/$(1)-$$($(1)_VER)-stamp
$(1)_PATCH=$$(WORKDIR)/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff

$$($(1)_STAMP)-wget:
	wget $$(WGET_OPTS) $$($(1)_URL) -O $$($(1)_TGZ)
	touch $$@

$$($(1)_TGZ): $$($(1)_STAMP)-wget
	touch $$@

$$($(1)_STAMP)-$$(TARGET)-patch: $$($(1)_DIR)
        # try to fetch a patch
	wget $$(WGET_OPTS) https://www.mutekh.org/www/tools/patchs/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff.gz -O $$($(1)_PATCH).gz || rm -f $$($(1)_PATCH).gz
        # test is a patch is available and apply
	test ! -f $$($(1)_PATCH).gz || ( cd $$($(1)_DIR) ; cat $$($(1)_PATCH).gz | gunzip | patch -p 0 )
	touch $$@

$$($(1)_STAMP)-$$(TARGET)-conf: $$($(1)_DIR) $$($(1)_STAMP)-$$(TARGET)-patch $$($(1)_DEPS)
	mkdir -p $$($(1)_BDIR)
	( cd $$($(1)_BDIR) ; LD_LIBRARY_PATH=$$(PREFIX)/lib $$($(1)_DIR)/configure --disable-nls --prefix=$$(PREFIX) --target=$$(TARGET)-unknown-elf --disable-checking --disable-werror $$($(1)_CONF) ) && touch $$@

$$($(1)_STAMP)-$$(TARGET)-build: $$($(1)_STAMP)-$$(TARGET)-conf
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) && touch $$@

$$(PREFIX)/$$($(1)_TESTBIN): $$($(1)_STAMP)-$$(TARGET)-build
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) install && touch $$@

$(1): $$(shell test -f $$(PREFIX)/$$($(1)_TESTBIN) || echo $$(PREFIX)/$$($(1)_TESTBIN))

endef

# template rules for other non target dependent tools

define TOOL_template

.PHONY: $(1)
.PRECIOUS: $$($(1)_TGZ)
.DELETE_ON_ERROR: $$($(1)_STAMP)-wget $$($(1)_STAMP)-conf $$($(1)_STAMP)-build

$(1)_DIR=$(WORKDIR)/$(1)-$($(1)_VER)
$(1)_BDIR=$(WORKDIR)/$(1)-bld-$($(1)_VER)
$(1)_STAMP=$(WORKDIR)/$(1)-$($(1)_VER)-stamp

$$($(1)_STAMP)-wget:
	wget $$(WGET_OPTS) $$($(1)_URL) -O $$($(1)_TGZ)
	touch $$@

$$($(1)_TGZ): $$($(1)_STAMP)-wget
	touch $$@

$$($(1)_STAMP)-conf: $$($(1)_DIR) $$($(1)_DEPS)
	mkdir -p $$($(1)_BDIR)
	( cd $$($(1)_BDIR) ; LD_LIBRARY_PATH=$$(PREFIX)/lib $$($(1)_DIR)/configure --prefix=$$(PREFIX) $$($(1)_CONF) ) && touch $$@

$$($(1)_STAMP)-build: $$($(1)_STAMP)-conf
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) && touch $$@

$$(PREFIX)/$$($(1)_TESTBIN): $$($(1)_STAMP)-build
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) install && touch $$@

$(1): $$(shell test -f $$(PREFIX)/$$($(1)_TESTBIN) || echo $$(PREFIX)/$$($(1)_TESTBIN))

endef

# template rules instantiation

$(eval $(call TOOL_template,mpfr))
$(eval $(call TOOL_template,gmp))
$(eval $(call TOOL_template,mpc))
$(eval $(call TGTTOOL_template,gdb))
$(eval $(call TGTTOOL_template,binutils))
$(eval $(call TGTTOOL_template,gcc))
$(eval $(call TOOL_template,dtc))

