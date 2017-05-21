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

# Build make invocation options
BLDMAKE_OPTS= -j8

# Install PATH
PREFIX=/opt/mutekh

# Temp directory
WORKDIR=/tmp/crossgen

common_CONF=
binutils_VER_common  = 2.25

# GNU Binutils
binutils_VER_mipsel  = $(binutils_VER_common)
binutils_VER_powerpc = $(binutils_VER_common)
binutils_VER_arm     = $(binutils_VER_common)
binutils_VER_i686    = $(binutils_VER_common)
binutils_VER_x86_64  = $(binutils_VER_common)
binutils_VER_nios2   = $(binutils_VER_common)
binutils_VER_sparc   = $(binutils_VER_common)
binutils_VER_lm32    = $(binutils_VER_common)
binutils_VER_microblaze = $(binutils_VER_common)
binutils_VER_m68k    = $(binutils_VER_common)
binutils_VER_avr32   = 2.22

binutils_VER=$(binutils_VER_$(TARGET))
binutils_CONF=$(common_CONF) --enable-plugins=no

gcc_VER_common  = 4.9.3

# GNU Compiler
gcc_VER_mipsel  = $(gcc_VER_common)
SUFFIX_mipsel   = unknown-elf

gcc_VER_powerpc = $(gcc_VER_common)
SUFFIX_powerpc  = unknown-elf

gcc_VER_arm     = $(gcc_VER_common)
gcc_CONF_arm    = --with-arch=armv4t --with-fpu=vfp --with-float=soft
SUFFIX_arm      = mutekh-eabi

gcc_VER_i686    = $(gcc_VER_common)
SUFFIX_i686     = unknown-elf

gcc_VER_x86_64  = $(gcc_VER_common)
SUFFIX_x86_64   = unknown-elf

gcc_VER_nios2   = $(gcc_VER_common)
SUFFIX_nios2    = unknown-elf

gcc_VER_sparc   = $(gcc_VER_common)
SUFFIX_sparc    = unknown-elf

gcc_VER_lm32    = $(gcc_VER_common)
SUFFIX_lm32     = unknown-elf

gcc_VER_microblaze = $(gcc_VER_common)
SUFFIX_microblaze  = unknown-elf

gcc_VER_m68k    = $(gcc_VER_common)
SUFFIX_m68k     = unknown-elf

gcc_VER_avr32   = 4.4.3
SUFFIX_avr32    = unknown-elf

gcc_VER=$(gcc_VER_$(TARGET))
gcc_CONF=$(common_CONF) --enable-languages=c --disable-libssp --enable-multilib --disable-lto --disable-libquadmath --enable-checking=release --with-system-zlib

# GCC requirements
mpfr_VER=2.4.2
gmp_VER=4.3.2
mpc_VER=0.9

# GNU Debugger
gdb_VER_mipsel  = 7.8.2
gdb_VER_powerpc = 7.8.2
gdb_VER_arm     = 7.8.2
gdb_VER_i686    = 7.8.2
gdb_VER_x86_64  = 7.8.2
gdb_VER_nios2   = 7.8.2
gdb_VER_sparc   = 7.8.2
gdb_VER_lm32    = 7.8.2
gdb_VER_microblaze = 7.8.2
gdb_VER_m68k   = 7.8.2
gdb_VER_avr32   = 6.7.1

gdb_VER=$(gdb_VER_$(TARGET))
gdb_CONF=$(common_CONF) --with-python=no --disable-sim

# Device Tree Compiler
dtc_VER=1.2.0

# Bocsh x86 emulator
bochs_VER=2.4.6
bochs_CONF= --enable-x86-64 --enable-smp --enable-acpi --enable-pci --enable-disasm --enable-fpu --enable-alignment-check --enable-cdrom --enable-iodebug --with-nogui --with-term
          # --enable-debugger --enable-gdb-stub

# Qemu emulator
qemu_VER=0.14.0
qemu_CONF=--disable-docs --disable-kvm --disable-linux-user --disable-vnc-tls --extra-ldflags=-lrt

# Testsuite simulation wrapper
testwrap_VER=1.1

HELP_END=134 #### LINE 134 IS HERE ####

unexport MAKEFLAGS
unexport MFLAGS
unexport MAKELEVEL

SUFFIX=$(SUFFIX_$(TARGET))

# packages configurations

binutils_ARCHIVE=binutils-$(binutils_VER).tar.bz2
binutils_URL=ftp://ftp.gnu.org/gnu/binutils/$(binutils_ARCHIVE)
binutils_TESTBIN=bin/$(TARGET)-$(SUFFIX)-as

gcc_ARCHIVE=gcc-$(gcc_VER).tar.bz2
gcc_URL=ftp://ftp.gnu.org/gnu/gcc/gcc-$(gcc_VER)/$(gcc_ARCHIVE)
gcc_TESTBIN=bin/$(TARGET)-$(SUFFIX)-gcc
gcc_DEPS=binutils mpfr gmp mpc
gcc_CONF+=--with-mpfr=$(PREFIX) --with-gmp=$(PREFIX) --with-mpc=$(PREFIX)

gdb_ARCHIVE=gdb-$(gdb_VER).tar.gz
gdb_URL=ftp://ftp.gnu.org/gnu/gdb/gdb-$(gdb_VER).tar.gz
gdb_TESTBIN=bin/$(TARGET)-$(SUFFIX)-gdb

mpfr_ARCHIVE=mpfr-$(mpfr_VER).tar.bz2
mpfr_URL=ftp://ftp.gnu.org/gnu/mpfr/$(mpfr_ARCHIVE)
mpfr_TESTBIN=lib/libmpfr.a
mpfr_DEPS=gmp
mpfr_CONF+=--with-gmp=$(PREFIX)

gmp_ARCHIVE=gmp-$(gmp_VER).tar.bz2
gmp_URL=ftp://ftp.gnu.org/gnu/gmp/$(gmp_ARCHIVE)
gmp_TESTBIN=lib/libgmp.a

mpc_ARCHIVE=mpc-$(mpc_VER).tar.gz
mpc_URL=http://www.multiprecision.org/mpc/download/$(mpc_ARCHIVE)
mpc_TESTBIN=lib/libmpc.a
mpc_DEPS=mpfr gmp
mpc_CONF+=--with-mpfr=$(PREFIX) --with-gmp=$(PREFIX)

dtc_ARCHIVE=dtc-$(dtc_VER).tar.gz
dtc_URL=https://www.mutekh.org/tools/$(dtc_ARCHIVE)
dtc_TESTBIN=bin/dtc

testwrap_ARCHIVE=testwrap-$(testwrap_VER).tar.gz
testwrap_URL=https://www.mutekh.org/tools/$(testwrap_ARCHIVE)
testwrap_TESTBIN=bin/testwrap

bochs_ARCHIVE=bochs-$(bochs_VER).tar.gz
bochs_URL=http://freefr.dl.sourceforge.net/project/bochs/bochs/$(bochs_VER)/$(bochs_ARCHIVE)
bochs_TESTBIN=bin/bochs

qemu_ARCHIVE=qemu-$(qemu_VER).tar.gz
qemu_URL=http://download.savannah.gnu.org/releases/qemu/$(qemu_ARCHIVE)
qemu_TESTBIN=bin/qemu
qemu_INTREE_BUILD=1

PATCH_URL=https://www.mutekh.org/tools/diffs/

WGET_OPTS=-c -t 5 -w 5 --no-check-certificate

$(shell mkdir -p $(WORKDIR))

# main rules

help:
	@echo "usage ./crossgen.mk [CONFIG_VAR=..., ...] target"
	@echo ""
	@echo "Main targets:"
	@echo "  config    - display configuration"
	@echo "  toolchain - download, configure, build and install gcc, binutils, gdb"
	@echo "  testtools - download, configure, build and install testwrap, bochs, qemu"
	@echo "  all       - download, configure, build and install all packages"
	@echo "  cleanup   - remove all build files, keep downloaded archives"
	@echo ""
	@echo "Package targets:"
	@echo "  gcc, binutils, gdb, dtc, testwrap, bochs, qemu"

config:
	@head -n $$(($(HELP_END)-1)) $(MAKEFILE_LIST) | tail -n $$(($(HELP_END)-26))

toolchain: gcc binutils gdb

testtools: testwrap bochs qemu

all:       gcc binutils gdb dtc bochs qemu

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
.DELETE_ON_ERROR: $$($(1)_STAMP)-wget $$($(1)_STAMP)-$$(TARGET)-conf $$($(1)_STAMP)-$$(TARGET)-build $$($(1)_STAMP)-patch

$(1)_DIR=$$(WORKDIR)/$(1)-$$($(1)_VER)
$(1)_BDIR=$$(WORKDIR)/$(1)-bld-$$(TARGET)-$$($(1)_VER)
$(1)_STAMP=$$(WORKDIR)/$(1)-$$($(1)_VER)-stamp
$(1)_PATCH=$$(WORKDIR)/$(1)-$$($(1)_VER)-latest.diff
$(1)_TGZ=$$(WORKDIR)/$$($(1)_ARCHIVE)
CLEANUP_FILES+=$$($(1)_BDIR) $$($(1)_STAMP)-$$(TARGET)-conf $$($(1)_STAMP)-$$(TARGET)-build $$($(1)_STAMP)-patch $$($(1)_PATCH)*

$$($(1)_STAMP)-wget:
	wget $$(WGET_OPTS) $$($(1)_URL) -O $$($(1)_TGZ)
	touch $$@

$$($(1)_TGZ): $$($(1)_STAMP)-wget
	touch $$@

$$($(1)_STAMP)-patch: $$($(1)_DIR)
        # try to fetch a patch
	wget $$(WGET_OPTS) $$(PATCH_URL)/$(1)-$$($(1)_VER)-latest.diff.gz -O $$($(1)_PATCH).gz || rm -f $$($(1)_PATCH).gz
        # test if a patch is available and apply
	( cd $$($(1)_DIR) ; cat $$($(1)_PATCH).gz | gunzip | patch -p 0 )
	touch $$@

$$($(1)_STAMP)-$$(TARGET)-conf: $$($(1)_DIR) $$($(1)_STAMP)-patch $$($(1)_DEPS)
	mkdir -p $$($(1)_BDIR)
	( cd $$($(1)_BDIR) ; LD_LIBRARY_PATH=$$(PREFIX)/lib $$($(1)_DIR)/configure MAKEINFO="makeinfo --version" --disable-nls --prefix=$$(PREFIX) --target=$$(TARGET)-$$(SUFFIX) --disable-werror $$($(1)_CONF) $$($(1)_CONF_$$(TARGET))  ) && touch $$@

$$($(1)_STAMP)-$$(TARGET)-build: $$($(1)_STAMP)-$$(TARGET)-conf
	LD_LIBRARY_PATH=$$(PREFIX)/lib make $$(BLDMAKE_OPTS) -C $$($(1)_BDIR) && touch $$@

$$(PREFIX)/$$($(1)_TESTBIN): $$($(1)_STAMP)-$$(TARGET)-build
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) -j1 install && touch $$@

$(1): $$(shell test -f $$(PREFIX)/$$($(1)_TESTBIN) || echo $$(PREFIX)/$$($(1)_TESTBIN))

endef

# template rules for other non target dependent tools

define TOOL_template

.PHONY: $(1)
.PRECIOUS: $$($(1)_TGZ)
.DELETE_ON_ERROR: $$($(1)_STAMP)-wget $$($(1)_STAMP)-conf $$($(1)_STAMP)-build

$(1)_DIR=$$(WORKDIR)/$(1)-$$($(1)_VER)
$(1)_BDIR=$$(if $$($(1)_INTREE_BUILD), $$(WORKDIR)/$(1)-$$($(1)_VER), $$(WORKDIR)/$(1)-bld-$$($(1)_VER))
$(1)_STAMP=$$(WORKDIR)/$(1)-$$($(1)_VER)-stamp
$(1)_PATCH=$$(WORKDIR)/$(1)-$$($(1)_VER)-latest.diff
$(1)_TGZ=$$(WORKDIR)/$$($(1)_ARCHIVE)
CLEANUP_FILES+=$$($(1)_BDIR) $$($(1)_STAMP)-$$(TARGET)-conf $$($(1)_STAMP)-$$(TARGET)-build

$$($(1)_STAMP)-wget:
	wget $$(WGET_OPTS) $$($(1)_URL) -O $$($(1)_TGZ)
	touch $$@

$$($(1)_STAMP)-patch: $$($(1)_DIR)
        # try to fetch a patch
	wget $$(WGET_OPTS) $$(PATCH_URL)/$(1)-$$($(1)_VER)-latest.diff.gz -O $$($(1)_PATCH).gz || rm -f $$($(1)_PATCH).gz
        # test is a patch is available and apply
	( cd $$($(1)_DIR) ; cat $$($(1)_PATCH).gz | gunzip | patch -p 0 )
	touch $$@

$$($(1)_TGZ): $$($(1)_STAMP)-wget
	touch $$@

$$($(1)_STAMP)-conf: $$($(1)_DIR) $$($(1)_STAMP)-patch $$($(1)_DEPS)
	mkdir -p $$($(1)_BDIR)
	( cd $$($(1)_BDIR) ; LD_LIBRARY_PATH=$$(PREFIX)/lib $$($(1)_DIR)/configure --prefix=$$(PREFIX) $$($(1)_CONF) ) && touch $$@

$$($(1)_STAMP)-build: $$($(1)_STAMP)-conf
	LD_LIBRARY_PATH=$$(PREFIX)/lib make $$(BLDMAKE_OPTS) -C $$($(1)_BDIR) && touch $$@

$$(PREFIX)/$$($(1)_TESTBIN): $$($(1)_STAMP)-build
	LD_LIBRARY_PATH=$$(PREFIX)/lib make -C $$($(1)_BDIR) -j1 install && touch $$@

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
$(eval $(call TOOL_template,bochs))
$(eval $(call TOOL_template,qemu))
$(eval $(call TOOL_template,testwrap))

cleanup:
	rm -rf $(CLEANUP_FILES)
