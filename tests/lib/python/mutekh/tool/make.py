
from mt.base import tool

class Make(tool.Tool):

    def __init__(self, min_version, deps = []):
        self.available = Make.in_path("make")
        self.available = reduce(lambda x, y : x and y.is_available(), deps, self.available)

    def __call__(self, *args, **kwargs):
        t = tool.ToolUsage()

        if self.available:
            argv = map(lambda x:"%s=%s"%x, kwargs.items()) + list(args)
            t.add("make", "-j", *argv);

        return t
