
from mt.base import tool

class Dtc(tool.Tool):

    def __init__(self, min_version):
        self.available = Dtc.in_path("dtc")

