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
#    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2009
#

#### LINE 25 IS HERE ####

# GNU Binutils
binutils_VER=2.20.1
binutils_CONF=

# GNU Compiler
gcc_VER=4.5.2
gcc_CONF=--enable-languages=c --disable-libssp

# GCC requirements
mpfr_VER=2.4.2
gmp_VER=4.3.2
mpc_VER=0.9

# GNU Debugger
gdb_VER=7.2
gdb_CONF=

# Device Tree Compiler
dtc_VER=1.2.0

# Target processor: mipsel mipseb powerpc arm i686 sparc nios2 ...
TARGET=mipsel

# Install PATH
PREFIX=/opt/mutekh

# Temp directory
WORKDIR=/tmp/crossgen

# Some targets may require precise version of the tools to fetch
# patchs from https://www.mutekh.org/www/tools/patchs/

#### LINE 45 IS HERE ####

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

help:
	@echo Available targets are: all, gcc, binutils, gdb, dtc
	@echo Default configuration is:
	@head $(MAKEFILE_LIST) -n 58 | tail -n 33

all: gcc binutils gdb dtc

.PRECIOUS: $(binutils_TGZ) $(gcc_TGZ) $(gdb_TGZ) $(mpfr_TGZ) $(gmp_TGZ) $(mpc_TGZ)
.DELETE_ON_ERROR: \
	$(binutils_STAMP)-wget $(binutils_STAMP)-$(TARGET)-conf $(binutils_STAMP)-$(TARGET)-build \
	$(gcc_STAMP)-wget $(gcc_STAMP)-$(TARGET)-conf $(gcc_STAMP)-$(TARGET)-build \
	$(gdb_STAMP)-wget $(gdb_STAMP)-$(TARGET)-conf $(gdb_STAMP)-$(TARGET)-build \
	$(mpfr_STAMP)-wget $(mpfr_STAMP)-$(TARGET)-conf $(mpfr_STAMP)-$(TARGET)-build \
	$(gmp_STAMP)-wget $(gmp_STAMP)-$(TARGET)-conf $(gmp_STAMP)-$(TARGET)-build \
	$(mpc_STAMP)-wget $(mpc_STAMP)-$(TARGET)-conf $(mpc_STAMP)-$(TARGET)-build

% : %.tar.bz2
	( mkdir -p $@ ; cd $@/.. ; tar xjf $< )
	touch $@

% : %.tar.gz
	( mkdir -p $@ ; cd $@/.. ; tar xzf $< )
	touch $@

define TGTTOOL_template

$(1)_DIR=$$(WORKDIR)/$(1)-$$($(1)_VER)
$(1)_BDIR=$$(WORKDIR)/$(1)-bld-$$(TARGET)-$$($(1)_VER)
$(1)_STAMP=$$(WORKDIR)/$(1)-$$($(1)_VER)-stamp
$(1)_PATCH=$$(WORKDIR)/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff

$$($(1)_STAMP)-wget:
	touch $$@
	wget $$(WGET_OPTS) $$($(1)_URL) -O $$($(1)_TGZ)
$$($(1)_TGZ): $$($(1)_STAMP)-wget
	touch $$@

$$($(1)_STAMP)-$$(TARGET)-patch: $$($(1)_DIR)
	wget $$(WGET_OPTS) https://www.mutekh.org/www/tools/patchs/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff.gz -O $$($(1)_PATCH).gz || rm -f $$($(1)_PATCH).gz
	test ! -f $$($(1)_PATCH).gz || ( cd $$($(1)_DIR) ; cat $$($(1)_PATCH).gz | gunzip | patch -p 0 )
	touch $$@

$$($(1)_STAMP)-$$(TARGET)-conf: $$($(1)_DIR) $$($(1)_STAMP)-$$(TARGET)-patch $$($(1)_DEPS)
	mkdir -p $$($(1)_BDIR)
	( cd $$($(1)_BDIR) ; $$($(1)_DIR)/configure --disable-nls --prefix=$$(PREFIX) --target=$$(TARGET)-unknown-elf --disable-checking --disable-werror $$($(1)_CONF) ) && touch $$@

$$($(1)_STAMP)-$$(TARGET)-build: $$($(1)_STAMP)-$$(TARGET)-conf
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) && touch $$@

$$(PREFIX)/$$($(1)_TESTBIN): $$($(1)_STAMP)-$$(TARGET)-build
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) install && touch $$@

$(1): $$(PREFIX)/$$($(1)_TESTBIN)

endef

define TOOL_template

$(1)_DIR=$(WORKDIR)/$(1)-$($(1)_VER)
$(1)_BDIR=$(WORKDIR)/$(1)-bld-$($(1)_VER)
$(1)_STAMP=$(WORKDIR)/$(1)-$($(1)_VER)-stamp

$$($(1)_STAMP)-wget:
	touch $$@
	wget $$(WGET_OPTS) $$($(1)_URL) -O $$($(1)_TGZ)
$$($(1)_TGZ): $$($(1)_STAMP)-wget
	touch $$@

$$($(1)_STAMP)-conf: $$($(1)_DIR) $$($(1)_DEPS)
	mkdir -p $$($(1)_BDIR)
	( cd $$($(1)_BDIR) ; $$($(1)_DIR)/configure --prefix=$$(PREFIX) $$($(1)_CONF) ) && touch $$@

$$($(1)_STAMP)-build: $$($(1)_STAMP)-conf
	make -C $$($(1)_BDIR) && touch $$@

$$(PREFIX)/$$($(1)_TESTBIN): $$($(1)_STAMP)-build
	make -C $$($(1)_BDIR) install && touch $$@

$(1): $$(PREFIX)/$$($(1)_TESTBIN)

endef

$(eval $(call TOOL_template,mpfr))
$(eval $(call TOOL_template,gmp))
$(eval $(call TOOL_template,mpc))
$(eval $(call TGTTOOL_template,gdb))
$(eval $(call TGTTOOL_template,binutils))
$(eval $(call TGTTOOL_template,gcc))
$(eval $(call TOOL_template,dtc))
