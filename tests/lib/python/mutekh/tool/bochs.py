
from mt.base import tool

class Bochs(tool.Tool):

    def __init__(self, min_version = 0):
        self.available = Bochs.in_path("testwrap", "grep", "tar", "bzip2", "mkisofs", "bochs")
        # FIXME test "%(srcdir)s/tools/iso.tar.bz2" availability
        # FIXME test bochs configuration

    def __call__(self, *bochs_args):
        t = tool.ToolUsage()

        if self.available:
            t.add("tar", "--directory", "obj-%(target)s", "-xjf", "%(srcdir)s/tools/iso.tar.bz2")
            t.add("cp", "%(kernel)s", "obj-%(target)s/tmp_iso/boot/kernel-ibmpc-x86.out")
            t.add("mkisofs", "-R", "-b", "boot/grub/stage2_eltorito",
                  "-no-emul-boot", "-boot-load-size", "4",
                  "-boot-info-table", "-o", "%(target)s.iso",
                  "obj-%(target)s/tmp_iso")
            t.add("testwrap", "-s9", "-a%(timeout)s", "-i",
                  "bochs", "-qf", "/dev/null",
                  'display_library: nogui',
                  'log: %(target)s.bochs.log',
                  'ata0-master: type=cdrom, path=%(target)s.iso, status=inserted',
                  'boot: cdrom',
                  'port_e9_hack: enabled=1',
                  *bochs_args)
            t.add("grep", "-q", "%(success_grep)s", "%(target)s_%(action)s.log")

        return t;

