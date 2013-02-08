 # -*- coding: utf-8 -*-

from distutils.core import setup

setup(name='pymorse',
      version='1.0',
      license='BSD 3-clauses',
      description='Python bindings for the Modular OpenRobots Simulation Engine (MORSE)',
      author='Séverin Lemaignan',
      author_email='severin.lemaignan@laas.fr',
      url='http://morse.openrobots.org',
      package_dir = {'':'src'},
      packages=['pymorse'],
      )
