"""
MORSE
=====

__The Modular OpenRobots Simulation Engine__

MORSE is a Blender-based robotic simulator.
"""
import os
from setuptools import setup, find_packages, Extension

def get_data_files(prefix='share/morse/data', data_dir='data'):
    dir_files = []
    data_files = []
    for f in os.listdir(data_dir):
        path = os.path.join(data_dir, f)
        if os.path.isfile(path):
            dir_files.append(path)
        else:
            data_files.extend(get_data_files(os.path.join(prefix, f), path))
    data_files.append((prefix, dir_files))
    return data_files

data_files = get_data_files('share/morse/data', 'data')
data_files.extend(get_data_files('share/morse/examples', 'examples'))
data_files.extend(get_data_files('share/morse/addons', 'addons'))

setup(
    name = 'MORSE',
    version = '0.6',
    packages = find_packages('src'),
    package_dir = {'morse': 'src/morse'},
    ext_modules = [
        Extension('morse.modifiers.gaussian', ['src/morse/modifiers/gaussian.c']),
        Extension('morse.sensors.zBuffTo3D', ['src/morse/sensors/zBuffTo3Dmodule.c'])
    ],
    # TODO package_data requires data to be in the src/morse folder
    #package_data = {'': ['data/*/*.blend']},
    # distutils data_files seems to work with setuptools
    data_files = data_files,
    # TODO morse.main generate /usr/local/bin/morse script
    #entry_points = {'console_scripts': ['morse = morse.main:main']},
    # Optional
    author = 'LAAS-CNRS',
    author_email = 'morse-dev@laas.fr',
    platforms = 'any',
    url = 'http://morse.openrobots.org',
    download_url = 'https://github.com/laas/morse',
    keywords = ['MORSE'],
    classifiers = [
        # As from http://pypi.python.org/pypi?%3Aaction=list_classifiers
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python",
        "Topic :: Scientific/Engineering"
    ],
    description = "The Modular OpenRobots Simulation Engine",
    long_description = __doc__,
    license = 'BSD'
)
