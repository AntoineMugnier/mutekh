
from mt.base.match import OrMatch

class StageUsage:

    def __init__(self, toolusage, context):
        self.__toolusage = toolusage
        self.__context = context

    def get_command(self, context):
        context.update(self.__context)
        return self.__toolusage.run(context)

class Stage:

    def __init__(self, backends):
        self.__backends = backends

    def get_stageusage_list(self, point, base_context):
        l = []
        for backends in self.__backends:
            configs_with_feature = OrMatch(*point.get_features_config())
            if configs_with_feature.feature_test(backends[0]):
                context = dict();
                for f in configs_with_feature.get_features():
                    fl = f.split(':', 1)
                    if len(fl) == 2:
                        context[fl[0]] = fl[1]
                l.append(StageUsage(backends[1], context))
        return l

class Configure(Stage):
    pass

class Execute(Stage):
    pass

class Build(Stage):
    pass

