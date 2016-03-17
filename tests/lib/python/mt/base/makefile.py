
class Makefile:
    def __init__(self):
        self.__rules = []

    def append(self, rule):
        self.__rules.append(rule)

    def insert(self, index, rule):
        self.__rules.insert(index, rule)

    def generate(self, output):
        out = open(output, "w")
        if (out):
            for r in self.__rules:
                out.write(r.generate())
                out.write("\n")
            out.close()

    def __iter__(self):
        all = reduce(list.__add__, map(lambda x:x.targets, self.__rules), [])
        return iter(all)

class Rule:
    def __init__(self, targets, prereq, commands):
        self.__targets = targets
        self.__prereq = prereq        
        self.__commands = commands

    @property
    def targets(self):
        return self.__targets

    def generate(self):
        return """
%s: %s
	%s
""" % ( ' '.join(self.__targets),
        ' '.join(self.__prereq),
        '\n\t'.join(self.__commands), )
