 # -*- coding: utf-8 -*-

from distutils.core import setup

setup(name='pymorse',
      version='1.4',
      license='BSD 3-clauses',
      description='Python bindings for the Modular OpenRobots Simulation Engine (MORSE)',
      author='Séverin Lemaignan, Pierrick Koch',
      author_email='severin.lemaignan@plymouth.ac.uk, pierrick.koch@laas.fr',
      classifiers=[
        'Development Status :: 5 - Production/Stable',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.2',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Topic :: Scientific/Engineering',
      ],
      url='http://morse.openrobots.org',
      package_dir = {'':'src'},
      packages=['pymorse'],
      requires=['futures'],
      )
