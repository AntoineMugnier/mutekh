from distutils.core import setup

setup(
    name = "mutekh-test",
    version = "0.0.1",
    description = "MutekH Test runner",
    author = "nipo",
    author_email = "nipo@ssji.net",
    license = "MIT",
    classifiers = [
        "Development Status :: 4 - Beta",
        "Programming Language :: Python",
    ],
    use_2to3 = False,
    packages = ["mt", "mutekh"],
    package_dir = {'mt': 'lib/python/mt',
                   'mutekh': 'lib/python/mutekh'}
)
