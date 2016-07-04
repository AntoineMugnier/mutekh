# -*- python-mode -*-

#
# Usage notice :
#
# This file describes architectures where code are tested on. Depending on test
# definitions, the MutekH source code is configured, build and/or executed on
# architectures and platforms declared here. Update this file to add new
# supported features or new architectures or platforms.
#

import os

from mt.base.environment import *
from mt.base.match import *
from mt.base.expectation import *

###############################################################################
#
# Configurations
#
# Defines several possible configuration for each architecture and platforms
# MutekH is ported on. For each configuration a set of features are declared.
# These features are used by the test framework to eventually filter in or out
# some platforms from generated test sets.
#
###############################################################################

# test configuration          BUILD sections                             platform features

_mips32_features      = ["cpu:mips32", "waitirq", "usermode", "preempt", "bytecode"]
_arm32_features       = ["cpu:arm32", "waitirq", "usermode", "preempt"]
_arm32m_features      = ["cpu:arm32m", "waitirq", "little", "preempt", "bytecode"]
_ppc_features         = ["cpu:ppc", "waitirq", "big", "usermode", "preempt"]
_sparc_features       = ["cpu:sparc", "waitirq", "big", "usermode", "preempt"]
_nios2_features       = ["cpu:nios2", "little", "usermode", "preempt"]
_lm32_features        = ["cpu:lm32", "big", "preempt"]
_x86_features         = ["cpu:x86", "waitirq", "little"] # , "preempt", "smp", "usermode", "fpu"

##################################################
# SocLib

_soclib_features      = ["arch:soclib", "spi", "timer", "char", "minimal", "ipi"]

soclib_mips32eb      = Config("soclib-mips32eb:pf-tutorial:memcheck",   _soclib_features + _mips32_features + ["smp", "big"])
soclib_mips32el      = Config("soclib-mips32el:pf-tutorial:memcheck",   _soclib_features + _mips32_features + ["smp", "little"])
soclib_arm32         = Config("soclib-arm:pf-tutorial:memcheck",        _soclib_features + _arm32_features + ["smp", "little"])
soclib_arm32_big     = Config("soclib-armbe:pf-tutorial:memcheck",      _soclib_features + _arm32_features + ["smp", "big"])
soclib_ppc           = Config("soclib-ppc:pf-tutorial:memcheck",        _soclib_features + _ppc_features + ["smp"])
soclib_sparc         = Config("soclib-sparc:pf-tutorial:memcheck",      _soclib_features + _sparc_features + ["smp"])
soclib_nios2         = Config("soclib-nios2:pf-tutorial:memcheck",      _soclib_features + _nios2_features + ["smp"])
soclib_lm32          = Config("soclib-lm32:pf-tutorial:memcheck",       _soclib_features + _lm32_features)

##################################################
# IBM-PC (32 bits)

ibmpc_x86            = Config("ibmpc-x86",                              _x86_features + ["arch:ibmpc", "ipi"])

##################################################
# Linux (32 and 64 bits)

emu_linux_x86        = Config("emu-linux-x86",                          ["arch:emu",    "cpu:x86-emu", "smp", "little", "fpu", "ipi"])
emu_linux_x86_64     = Config("emu-linux-x86_64",                       ["arch:emu",    "cpu:x86_64-emu", "reg64", "smp", "little", "fpu", "ipi"])

##################################################
# SiLab EFM32 low-power archs and platforms

_efm32_features      = _arm32m_features + ["arch:efm32", "spi", "i2c", "pwm", "timer", "char", "usbdev", "minimal"]

efm32_zero           = Config("efm32-stk3200",                          _efm32_features)
efm32_leopard        = Config("efm32-stk3600",                          _efm32_features)
efm32_giant          = Config("efm32-stk3700",                          _efm32_features)
efm32_wonder         = Config("efm32-stk3800",                          _efm32_features)

##################################################
# Nordic nrf5x

_nrf5x_features      = _arm32m_features + ["arch:nrf5x", "spi", "i2c", "pwm", "timer", "char", "minimal"]

nrf51                = Config("nrf5x-51822-128-16:arch_drv",            _nrf5x_features)
nrf52                = Config("nrf5x-52832-512-64:arch_drv",            _nrf5x_features)

##################################################
# Microchip PIC32 (mips-based) archs and platforms

_pic32_features      = _mips32_features + ["arch:pic32", "spi", "timer", "char", "usbdev", "little"]

pic32_mz             = Config("pic32-stkmzec",                          _pic32_features)

##################################################
# Gaisler Leon3 IP

_gaisler_features    = _sparc_features + ["arch:gaisler", "timer", "char"]

gaisler_leon3        = Config("gaisler-leon3",                          _gaisler_features)

##################################################
# ST Microelectronics STM32 arch and platforms

_stm32_features      = _arm32m_features + ["arch:stm32", "spi", "i2c", "pwm", "timer", "char", "devtree", "gpio", "mem", "minimal"]

stm32_nucleof103rb   = Config("stm32-nucleof103rb",                     _stm32_features)
stm32_nucleof401re   = Config("stm32-nucleof401re",                     _stm32_features)

#################################################
# BCM2835 SoC used in raspberry pi

_bcm283x_features    = _arm32_features + ["arch:bcm283x", "little", "spi", "i2c", "timer", "char", "gpio", "minimal"]

bcm283x_raspberry    = Config("bcm283x-raspberry-b2", _bcm283x_features)

#################################################
# Xilinx Zynq

_zynq_features       = _arm32_features + ["arch:zynq", "little", "big", "char", "minimal"]

zynq                 = Config("zynq", _zynq_features)

##################################################
# Ti cc1310 and cc26x0

_cc26xx_features      = _arm32m_features + ["arch:nrf5x", "spi", "timer", "char", "gpio", "minimal"]

cc1310                = Config("cc1310-f128-rgz",            _cc26xx_features)

###############################################################################
#
# CPU's
#
# Defines a selection from configurations above that represents a minimal set
# of different cpu's provided by each platform constructors or providers.
#
###############################################################################

all_cpus             = OrMatch(soclib_mips32el,  # mips32
                               soclib_arm32,     # arm32
                               soclib_ppc,       # ppc
                               soclib_sparc,     # sparc
                               soclib_nios2,     # nios2
                               soclib_lm32,      # lm32
                               ibmpc_x86,        # x86
                               emu_linux_x86,    # x86-emu
                               emu_linux_x86_64, # x86_64-emu
#                               efm32_zero,       # arm32m cortex-m0      FIXME flash size too small
                               efm32_leopard,    # arm32m cortex-m3
                               efm32_wonder,     # arm32m cortex-m4
)

###############################################################################
#
# Architectures
#
# Defines architecture filters based on configurations above. The 'all_archs',
# 'all' and 'default' filters represent respectively a minimal superset of
# supported architectures, all supported platforms and minimal superset of
# supported platforms features (e.g. crypto, dma...). More precisely, a
# minimal superset here is defined as a set of configurations where the
# difference of each configuration feature sets is non-empty.
#
###############################################################################

all_archs            = OrMatch(soclib_mips32eb,
                               ibmpc_x86,
                               emu_linux_x86_64,
                               efm32_leopard,
                               nrf52,
                               pic32_mz,
                               gaisler_leon3,
                               stm32_nucleof401re,
                               bcm283x_raspberry,
                               zynq,
                               cc1310
)

all                  = OrMatch(soclib_mips32eb, soclib_mips32el, soclib_arm32, soclib_arm32_big,
                               soclib_ppc, soclib_sparc, soclib_nios2, soclib_lm32,
                               ibmpc_x86,
                               emu_linux_x86, emu_linux_x86_64,
#                               efm32_zero,
                               efm32_leopard, efm32_giant, efm32_wonder,
                               nrf51, nrf52,
                               pic32_mz,
                               gaisler_leon3,
                               stm32_nucleof103rb, stm32_nucleof401re,
                               bcm283x_raspberry,
                               zynq,
                               cc1310
)

default              = OrMatch(soclib_arm32,
                               soclib_mips32eb,
                               ibmpc_x86,
                               emu_linux_x86_64,
                               efm32_leopard)

###############################################################################

env_filter = os.environ.get('MUTEKH_TEST_FILTER')

if (env_filter != None):
    checks = env_filter.split(" ")
    all = all.filter_by_feature(*checks)
    all_archs = all_archs.filter_by_feature(*checks)
    all_cpus = all_cpus.filter_by_feature(*checks)
    default = default.filter_by_feature(*checks)

