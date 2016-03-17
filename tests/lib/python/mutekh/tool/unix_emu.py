
from mt.base import tool

class UnixEmu(tool.Tool):

    def __init__(self, host, cpu):
        import os
        import re
        u = os.uname()
        self.available = ((host == u[0])
            and (cpu == u[4] or (u[4] == 'x86_64' and re.match('i\d86', cpu)))
            and UnixEmu.in_path("testwrap", "killall", "grep"))

    def __call__(self):
        t = tool.ToolUsage()

        if self.available:
            t.add("testwrap", "-s9", "-a%(timeout)s", "./%(kernel)s")
            t.add("testwrap", "-i", "killall", "-9", "./%(kernel)s")
            t.add("grep", "-q", "%(success_grep)s", "%(target)s_%(action)s.log")

        return t
