
from mt.base import tool

class Gcc(tool.Tool):

    def __init__(self, prefix, min_version):
        self.available = Gcc.in_path(prefix + "gcc")

