
class Expectation:

    def __init__(self, stage, match):
        self.__stage = stage
        self.__match = match
    

class Xfail(Expectation):
    pass

class Rules:

    def __init__(self, *expect):
        self.__expect = expect

