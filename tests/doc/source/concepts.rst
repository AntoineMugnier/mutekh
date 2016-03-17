
====================
 Testsuite concepts
====================

Overview
========

This testsuite is oriented towards the MutekH build system.  MutekH
build uses a configuration made of:

* an application configuration file, with optional conditional parts,

* a ``BUILD=`` argument defining the conditional parts to use.

The resulting configuration defines:

* the build environment (toolchain to use, target CPU, target
  architecture),

* the compiled-in application code.

As we have an automated build-and-test goal, we need to know what maps
the build to a test.  This testsuite tool uses the following concepts:

A *test space*
  User enumerates a set of ``BUILD=`` options to iterate through (with
  automated cartesian product and optional exclusions).
A mapping to *steps*
  map disjoints subsets of the *test space* to commands to run.  This
  can involve building, testing, reporting, etc.

Prefered test environment is buildbot_.  All buils to run are to be
dispatched through buidl slaves.

.. _buildbot: http://www.buildbot.net/
