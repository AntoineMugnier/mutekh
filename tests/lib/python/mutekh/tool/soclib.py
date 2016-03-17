
from mt.base import tool

class Soclib(tool.Tool):

    def __init__(self, binary, *simvars):
        self.available = Soclib.in_path("testwrap", "grep", binary)
        self.__binary = binary
        self.__simvars = simvars

        # FIXME check version

    def __call__(self, *simu_args):
        t = tool.ToolUsage()

        if self.available:
            t.add(*(self.__simvars + ("testwrap", "-s9", "-a%(timeout)s",
                  self.__binary) + simu_args))
            t.add("grep", "-q", "%(success_grep)s", "%(target)s_%(action)s.log")

        return t
