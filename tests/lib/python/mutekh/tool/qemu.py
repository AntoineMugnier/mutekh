
from mt.base import tool

class Qemu(tool.Tool):

    def __init__(self, cpu, min_version = 0):
        self.available = Qemu.in_path("tar", "bzip2", "mkisofs", "testwrap", "qemu")
        # FIXME test "%(srcdir)s/tools/iso.tar.bz2" availability

        self.__cpu = cpu
        # FIXME launch qemu to get supported cpu and machine list

    def __call__(self, *qemu_args):
        t = tool.ToolUsage()

        if self.available:
            t.add("tar", "--directory", "obj-%(target)s", "-xjf", "%(srcdir)s/tools/iso.tar.bz2")
            t.add("cp", "%(kernel)s", "obj-%(target)s/tmp_iso/boot/kernel-ibmpc-x86.out")
            t.add("mkisofs", "-R", "-b", "boot/grub/stage2_eltorito",
                  "-no-emul-boot", "-boot-load-size", "4",
                  "-boot-info-table", "-o", "%(target)s.iso",
                  "obj-%(target)s/tmp_iso")
            t.add("testwrap", "-s9", "-a%(timeout)s", "-i",
                  "qemu-system-" + self.__cpu, "-debugcon", "file:%(target)s_%(action)s.qemu", "-no-reboot", "-nographic",
                  "-boot", "d", "-cdrom", "%(target)s.iso",
                  *qemu_args)
            t.add("grep", "-q", "%(success_grep)s", "%(target)s_%(action)s.qemu")

        return t;

