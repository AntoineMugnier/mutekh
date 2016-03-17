
from mt.base.stage import *

from mutekh.tool.gcc import *
from mutekh.tool.make import *
from mutekh.tool.dtc import *
from mutekh.tool.soclib import *
from mutekh.tool.qemu import *
from mutekh.tool.bochs import *
from mutekh.tool.unix_emu import *

# define tools
gcc_mips32   = Gcc(min_version = "4.9.3", prefix = "mipsel-unknown-elf-")
gcc_arm      = Gcc(min_version = "4.9.3", prefix = "arm-mutekh-eabi-")
gcc_ppc      = Gcc(min_version = "4.9.3", prefix = "powerpc-unknown-elf-")
gcc_sparc    = Gcc(min_version = "4.9.3", prefix = "sparc-unknown-elf-")
gcc_nios2    = Gcc(min_version = "4.9.3", prefix = "nios2-unknown-elf-")
gcc_lm32     = Gcc(min_version = "4.9.3", prefix = "lm32-unknown-elf-")
gcc_i686     = Gcc(min_version = "4.9.3", prefix = "i686-unknown-elf-")
gcc_x86_64   = Gcc(min_version = "4.9.3", prefix = "x86_64-unknown-elf-")

make         = Make(min_version = "3.81")

make_mips32  = Make(min_version = "3.81", deps = [ gcc_mips32 ])
make_arm     = Make(min_version = "3.81", deps = [ gcc_arm ])
make_ppc     = Make(min_version = "3.81", deps = [ gcc_ppc ])
make_sparc   = Make(min_version = "3.81", deps = [ gcc_sparc ])
make_nios2   = Make(min_version = "3.81", deps = [ gcc_nios2 ])
make_lm32    = Make(min_version = "3.81", deps = [ gcc_lm32 ])
make_i686    = Make(min_version = "3.81", deps = [ gcc_i686 ])
make_x86_64  = Make(min_version = "3.81", deps = [ gcc_x86_64 ])

soclib_catch_except = Soclib("mutekh_kernel_tutorial-system.x", "SOCLIB_MEMCHK_REPORT=0xfffffffe", "SOCLIB_MEMCHK=TX", "SOCLIB_GDB=TE", "SOCLIB_TTY=TERM")
soclib_no_except    = Soclib("mutekh_kernel_tutorial-system.x", "SOCLIB_MEMCHK_REPORT=0xfffffffa", "SOCLIB_MEMCHK=X", "SOCLIB_GDB=", "SOCLIB_TTY=TERM")
qemu_x86_64  = Qemu(cpu = 'x86_64')
bochs_x86    = Bochs()
linux_x86    = UnixEmu(host = 'Linux', cpu = 'i686')
linux_x86_64 = UnixEmu(host = 'Linux', cpu = 'x86_64')


# define backends for each test Stage

make_vars    = dict(CONF = "%(config)s",
                    BUILD = "%(build)s",
                    BUILD_NAME = "%(target)s")

configure = Configure([
    ( []               , make("config", **make_vars) )
    ])

build = Build([
#   'arch-cpu-cpucount-simulator'
    ( [ "+cpu:mips32" ]  , make_mips32(**make_vars) ),
    ( [ "+cpu:arm32" ]   , make_arm(**make_vars) ),
    ( [ "+cpu:arm32m" ]  , make_arm(**make_vars) ),
    ( [ "+cpu:ppc" ]     , make_ppc(**make_vars) ),
    ( [ "+cpu:sparc" ]   , make_sparc(**make_vars) ),
    ( [ "+cpu:nios2" ]   , make_nios2(**make_vars) ),
    ( [ "+cpu:lm32" ]    , make_lm32(**make_vars) ),
    ( [ "+cpu:x86" ]     , make_i686(**make_vars) ),
    ( [ "+cpu:x86_64" ]  , make_x86_64(**make_vars) ),
    ( [ "+cpu:x86-emu" ]     , make_i686(**make_vars) ),
    ( [ "+cpu:x86_64-emu" ]  , make_x86_64(**make_vars) ),

    ])

execute = Execute([
    ( [ "+arch:soclib", "+cpu:mips32", "-noexcept", "+little" ]  , soclib_catch_except("mips32el:%(cpu_count)s", "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:mips32", "-noexcept", "+big" ]     , soclib_catch_except("mips32eb:%(cpu_count)s", "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:arm32", "-noexcept" ]              , soclib_catch_except("arm:%(cpu_count)s" ,     "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:ppc", "-noexcept" ]                , soclib_catch_except("ppc:%(cpu_count)s" ,     "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:sparc", "-noexcept" ]              , soclib_catch_except("sparc:%(cpu_count)s" ,   "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:nios2", "-noexcept" ]              , soclib_catch_except("nios2:%(cpu_count)s" ,   "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:lm32", "-noexcept" ]               , soclib_catch_except("lm32:%(cpu_count)s" ,    "%(kernel)s") ),

    ( [ "+arch:soclib", "+cpu:mips32", "+noexcept", "+little" ]  , soclib_no_except("mips32el:%(cpu_count)s", "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:mips32", "+noexcept", "+big" ]     , soclib_no_except("mips32eb:%(cpu_count)s", "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:arm32", "+noexcept" ]              , soclib_no_except("arm:%(cpu_count)s" ,     "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:ppc", "+noexcept" ]                , soclib_no_except("ppc:%(cpu_count)s" ,     "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:sparc", "+noexcept" ]              , soclib_no_except("sparc:%(cpu_count)s" ,   "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:nios2", "+noexcept" ]              , soclib_no_except("nios2:%(cpu_count)s" ,   "%(kernel)s") ),
    ( [ "+arch:soclib", "+cpu:lm32", "+noexcept" ]               , soclib_no_except("lm32:%(cpu_count)s" ,    "%(kernel)s") ),

#    ( [ "+arch:ibmpc",  "+cpu:x86" ]                , qemu_x86_64("-smp", "%(cpu_count)s") ),
#    ( [ "+arch:ibmpc",  "+cpu:x86_64" ]             , qemu_x86_64("-smp", "%(cpu_count)s") ),
    ( [ "+arch:ibmpc",  "+cpu:x86" ]                , bochs_x86("cpu: count=%(cpu_count)s, quantum=1") ),

    ( [ "+arch:emu",    "+cpu:x86-emu" ]                , linux_x86() ),
    ( [ "+arch:emu",    "+cpu:x86_64-emu" ]             , linux_x86_64() ),

    ])

