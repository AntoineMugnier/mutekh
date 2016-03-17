import operator
import copy

class Match:
    """
    An object defining a point match criteria

    Matches can be composited with usual unary (``~``) and binary
    (``&|^``) operators.
    """
    
    def matches(self, point):
        """
        Tells whether the point matches the criteria
        """
        raise NotImplementedError

    def __invert__(self):
        return NotMatch(self)

    def __and__(self, other):
        return AndMatch(self, other)

    def __or__(self, other):
        return OrMatch(self, other)

    def __xor__(self, other):
        return XorMatch(self, other)

    def filter_by_feature(self, checks):
        return self

    def feature_test(self, checks):
        return True

    def get_features(self):
        return []

    def feature_test(self, checks):
        features = self.get_features()

        if not features:
            return True

        for c in checks:
            if c[0] == "+":
                if not c[1:] in features:
                    return False
            elif c[0] == "-":
                if c[1:] in features:
                    return False
            else:
                raise ValueError("Expected '-' or '+' as filter string prefix")

        return True

class MultiMatch(Match):
    def __init__(self, *matches):
        self.__matches = matches

    def matches(self, point):
        return reduce(self._op, map(lambda x:x.matches(point), self.__matches), self._default)

    def filter_by_feature(self, *checks):
        c = copy.copy(self);
        c.__matches = map(lambda x:x.filter_by_feature(checks), filter(lambda x:x.feature_test(checks), c.__matches))
        return c

    def expand(self):
        return self.__matches

    def get_features(self):
        l = []
        for f in self.__matches:
            l = l + f.get_features()
        return l

def config_filter(config, *checks):
    c = copy.copy(config);
    

class AndMatch(MultiMatch):
    _op = operator.and_
    _default = True

class OrMatch(MultiMatch):
    _op = operator.or_
    _default = False

class XorMatch(MultiMatch):
    _op = operator.xor
    _default = False

class NotMatch(Match):
    def __init__(self, match):
        self.__match = match

    def matches(self, point):
        return not self.__match.matches(point)

class Config(Match):
    """
    A configuration in the test space.

    A configuration may correspond to a backend, if relevant.
    """
    
    def __init__(self, section, features = []):
        """
        Creates a new config.
        
        :param str section: A section in the configuration file
        :param str backend: A backend identifier
        """
        self.__section = section
        self.__features = features

    def matches(self, point):
        """
        Returns whether the point matches this config

        :param Point point: A point
        :returns: a bool
        """
        return self in point

    def __str__(self):
        return self.__section

    def get_features(self):
        return self.__features
