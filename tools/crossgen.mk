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

SERVER=ftp://ftp.gnu.org/gnu

binutils_VER=2.20.1
binutils_CONF=

gcc_VER=4.5.2
gcc_CONF=--enable-languages=c --disable-libssp

gdb_VER=7.2
gdb_CONF=

# TARGET=mipseb powerpc arm i686 sparc nios2 ...
TARGET=mipsel

PREFIX=/opt/mutekh
WORKDIR=/tmp/crossgen


#### LINE 45 IS HERE ####

binutils_URL=$(SERVER)/binutils/binutils-$(binutils_VER).tar.bz2
binutils_TGZ=$(WORKDIR)/binutils-$(binutils_VER).tar.bz2
binutils_DIR=$(WORKDIR)/binutils-$(binutils_VER)
binutils_BDIR=$(WORKDIR)/binutils-bld-$(TARGET)-$(binutils_VER)
binutils_STAMP=$(WORKDIR)/binutils-$(binutils_VER)-stamp
binutils_TESTBIN=as

gcc_URL=$(SERVER)/gcc/gcc-$(gcc_VER)/gcc-$(gcc_VER).tar.bz2
gcc_TGZ=$(WORKDIR)/gcc-$(gcc_VER).tar.bz2
gcc_DIR=$(WORKDIR)/gcc-$(gcc_VER)
gcc_BDIR=$(WORKDIR)/gcc-bld-$(TARGET)-$(gcc_VER)
gcc_STAMP=$(WORKDIR)/gcc-$(gcc_VER)-stamp
gcc_TESTBIN=gcc
gcc_DEPS=binutils

gdb_URL=$(SERVER)/gdb/gdb-$(gdb_VER).tar.bz2
gdb_TGZ=$(WORKDIR)/gdb-$(gdb_VER).tar.bz2
gdb_DIR=$(WORKDIR)/gdb-$(gdb_VER)
gdb_BDIR=$(WORKDIR)/gdb-bld-$(TARGET)-$(gdb_VER)
gdb_STAMP=$(WORKDIR)/gdb-$(gdb_VER)-stamp
gdb_TESTBIN=gdb

$(shell mkdir -p $(WORKDIR))

help:
	@echo Available targets are: all, gcc, binutils, gdb
	@echo Default configuration is:
	@head $(MAKEFILE_LIST) -n 44 | tail -n 19

all: gcc binutils gdb

.PRECIOUS: $(binutils_TGZ) $(gcc_TGZ) $(gdb_TGZ)
.DELETE_ON_ERROR: $(binutils_STAMP)-wget $(gcc_STAMP)-wget $(gdb_STAMP)-wget \
	$(binutils_STAMP)-$(TARGET)-conf $(binutils_STAMP)-$(TARGET)-build \
	$(gcc_STAMP)-$(TARGET)-conf $(gcc_STAMP)-$(TARGET)-build \
	$(gdb_STAMP)-$(TARGET)-conf $(gdb_STAMP)-$(TARGET)-build

% : %.tar.bz2
	( mkdir -p $@ ; cd $@/.. ; tar xjf $< )
	touch $@

define TOOL_template

$$($(1)_STAMP)-wget:
	touch $$@
	wget -c $$($(1)_URL) -O $$($(1)_TGZ)
$$($(1)_TGZ): $$($(1)_STAMP)-wget
	touch $$@

$$(WORKDIR)/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff:
	wget --no-check-certificate https://www.mutekh.org/www/tools/patchs/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff.gz -O $$@.gz || test $$$$? = 8
	gunzip $$(WORKDIR)/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff.gz || true

$$($(1)_STAMP)-$$(TARGET)-patch: $$(WORKDIR)/$(1)-$$($(1)_VER)-$$(TARGET)-latest.diff $$($(1)_DIR)
	test ! -f $$< || ( cd $$($(1)_DIR) ; cat $$< | patch -p 0 ) && touch $$@

$$($(1)_STAMP)-$$(TARGET)-conf: $$($(1)_DIR) $$($(1)_STAMP)-$$(TARGET)-patch $$($(1)_DEPS)
	mkdir -p $$($(1)_BDIR)
	( cd $$($(1)_BDIR) ; $$($(1)_DIR)/configure --disable-nls --prefix=$$(PREFIX) --target=$$(TARGET)-unknown-elf --disable-checking --disable-werror $$($(1)_CONF) ) && touch $$@

$$($(1)_STAMP)-$$(TARGET)-build: $$($(1)_STAMP)-$$(TARGET)-conf
	make -C $$($(1)_BDIR) && touch $$@

$$(PREFIX)/bin/$$(TARGET)-unknown-elf-$$($(1)_TESTBIN): $$($(1)_STAMP)-$$(TARGET)-build
	make -C $$($(1)_BDIR) install && touch $$@

$(1): $$(PREFIX)/bin/$$(TARGET)-unknown-elf-$$($(1)_TESTBIN)

endef

$(eval $(call TOOL_template,gcc))
$(eval $(call TOOL_template,gdb))
$(eval $(call TOOL_template,binutils))

