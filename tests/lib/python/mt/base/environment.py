
import operator

class Point:
    
    def __init__(self, *coords):
        self.__coords = coords

    def __contains__(self, coord):
        return coord in self.__coords

    def __str__(self):
        return ":".join(filter(lambda s:s!="", map(str, self.__coords)))

    def get_features_config(self):
        b = []
        for i in self.__coords:
            if i.get_features():
                b.append(i)
        if not b:
            raise ValueError("No feature list found in any dimension for: %s" % self)
        return b

class EnvIterator:
    def __init__(self, dimensions, excludes):
        self.__dimensions = dimensions
        self.__excludes = excludes
        self.__state = [0]*len(self.__dimensions)
        self.__finished = False

    def next(self):
        if self.__finished:
            raise StopIteration

        while True:
            coords = map(lambda x: operator.getitem(*x), zip(self.__dimensions, self.__state))

            for i in range(len(self.__state)):
                if coords[i] == -1:
                    raise StopIteration
                self.__state[i] += 1
                if self.__state[i] != len(self.__dimensions[i]):
                    break
                self.__state[i] = 0
            else:
                self.__finished = True

            coords = filter(None, coords)
            p = Point(*coords)
            excluded = reduce(operator.or_, map(lambda x: x.matches(p), self.__excludes), False)
            if not excluded:
                return p
            
class Environment:
    """
    The multidimensional environment to explore.
    """
    
    def __init__(self, name, test_space, actions, features = [],
                 rules = [], covered_files = [], timeout = 30, success_grep = ""):
        """
        :param str name: a name
    
        :type test_space: :py:class:`~mt.base.environment.Dimension`
                          or :py:class:`~mt.base.environment.Exclude`
                          iterable

        :param test_space: the test space
    
        :param rules: build steps
        """
        self.__name = name
        self.__dimensions = filter(lambda x:isinstance(x, Dimension), test_space)
        self.__excludes = filter(lambda x:isinstance(x, Exclude), test_space)
        self.__rules = rules
        self.__covered_files = covered_files
        self.__actions = actions
        self.__path = "."
        self.__timeout = timeout
        self.__success_grep = success_grep

        # stats
        self.__total_tests_count = 0
        self.__complete_tests_count = 0
        self.__empty_tests_count = 0

    def __iter__(self):
        return EnvIterator(self.__dimensions, self.__excludes)

    def set_path(self, path):
        self.__path = path

    @classmethod
    def _targetify(cls, name):
        import re
        return re.sub('\W+', '_', name);

    def add_targets(self, mf):
        import makefile
        for t in self:
            target = self._targetify("TEST-"+self.__name+'_'+str(t))
            commands = [
                "@echo ============== %s =============="% str(t),
                ]

            context = dict(
                target = target,
                config = self.__path + "/config",
                build = t,
                kernel = target+'.out',
                srcdir = ".",
                timeout = self.__timeout,
                success_grep = self.__success_grep,
                action = "",
                cpu_count = 1,
                )

            self.__total_tests_count += 1
            complete = True
            empty = True

            i=0                 # action index

            for a in self.__actions:
                cmd_done = dict()   # command
                for b in a.get_stageusage_list(t, context):

                    redir = " > ";
                    context['action'] = a.__class__.__name__+str(i)
                    i += 1
                    cmds = b.get_command(context)

                    if len(cmds) == 0:
                        complete = False
                        commands.append("# test is not complete")
                        break;

                    for c in cmds:
                        def __escape(str):
                            import re
                            if re.search("\s", str):
                                return "'"+str+"'"
                            return str

                        cstr = " ".join(map(__escape, c))
                        if cstr not in cmd_done:
                            commands.append(cstr + redir + target+'_'+context['action'] + ".log 2>&1")
                            redir = " >> ";
                            cmd_done[cstr] = True
                            empty = False

            if complete:
                self.__complete_tests_count += 1
            if empty:
                self.__empty_tests_count += 1

            commands.append("touch "+target)

            mf.append(makefile.Rule([target], [], commands))

    def print_stats(self):
        print "  Total tests:     %i" % self.__total_tests_count
        print "  Complete tests:  %i" % self.__complete_tests_count
        print "  Partial tests:   %i" % (self.__total_tests_count - self.__empty_tests_count - self.__complete_tests_count)
        print "  Skipped tests:   %i" % self.__empty_tests_count

    def get_name(self):
        return self.__name

class Exclude:
    """
    An exclusion in the Environment test space
    """
    def __init__(self, match):
        """
        :type match: :py:class:`~mt.base.match.Match`
        :param match: the match for exclusion
        """
        self.__match = match

    def matches(self, point):
        return self.__match.matches(point)

class Dimension:
    def __init__(self, *configs):
        self.__configs = configs

    def __getitem__(self, index):
        if index >= len(self.__configs):
            return -1
        return self.__configs[index]

    def __len__(self):
        return len(self.__configs)

