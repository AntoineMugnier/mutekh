
class ToolUsage:

    def add(self, *argv):
        self.__cmds.append(argv)

    def __init__(self, *argv):
        self.__cmds = []
        if (len(argv)):
            self.add(*argv)

    def run(self, context):
        """
        Retrieve the commands list required to run the tool
        """
        return map(lambda x: map(lambda x: x % context, x), self.__cmds)

class Tool:

    @staticmethod
    def in_path(*cmds):
        def __in_path(cmd):
            import os
            for p in os.environ["PATH"].split(os.pathsep):
                f = os.path.join(p, cmd)
                if os.path.exists(f) and os.access(f, os.X_OK):
                    return True
            print "warning: The '%s' command is not available in PATH." % cmd
            return False;
        return reduce(lambda x, y: x and __in_path(y), cmds, True)

    def is_available(self):
        return self.available

    def get_version(self):
        """
        Retrieve the tool version
        """
        raise NotImplementedError()

    def __call__(self, *args, **env):
        """
        :returns: a :py:class:`~mt.base.tool.ToolUsage` object
        """
        raise NotImplementedError()
