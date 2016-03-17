import sys
import os
import argparse
from base import makefile

def standalone(*envs):
    """
    Main entry point for standalone test execution
    """
    parser = argparse.ArgumentParser(description = "MutekH-test standalone generator")

    parser.add_argument('--output', metavar = "OUTPUT",
                        type = str, default = "tests.mk",
                        help = 'Output Makefile name')

    args = parser.parse_args()

    for env in envs:
        env.set_path(dirname)
        env.add_targets(m)

    m = makefile.Makefile()
    self.add_targets(m)
    m.generate(args.output)
    self.print_stats()

def main():
    """
    Main entry point for batch test execution
    """
    parser = argparse.ArgumentParser(description = "MutekH-test generator")

    parser.add_argument('tests', metavar = 'INPUTS',
                        type = str, nargs = '*',
                        help = 'Test directories or descriptions')

    parser.add_argument('--output', metavar = "OUTPUT",
                        type = str, default = "tests.mk",
                        help = 'Output Makefile name')

    args = parser.parse_args()

    m = makefile.Makefile()

    for filename in args.tests:
        if not os.path.isfile(filename):
            filename = os.path.join(filename, "test")

        if not os.path.isfile(filename):
            raise ValueError("Not a description file: '%s'." % filename)

        l = {}
        exec open(filename) in l

        dirname = os.path.dirname(filename)

        envs = []

        if 'env' in l:
            envs.append(l['env'])

        if 'env_list' in l:
            envs += l['env_list']

        if not envs:
            raise ValueError("No environment in '%s'." % filename)

        for env in envs:
            env.set_path(dirname)
            env.add_targets(m)
            print "Test: %s" % env.get_name()
            env.print_stats()

    targets = list(m)
    m.insert(0, makefile.Rule(["all"], targets, []))
    m.generate(args.output)

if __name__ == "__main__":
    main()
