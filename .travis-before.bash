#!/bin/bash
sudo apt-get install cmake python3-dev python3.2-dev libsdl1.2debian 
wget http://download.blender.org/release/Blender2.61/blender-2.61-linux-glibc27-i686.tar.bz2
tar jxf blender-2.61-linux-glibc27-i686.tar.bz2
export MORSE_BLENDER=$(pwd)/blender-2.61-linux-glibc27-i686/blender
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages
export MORSE_ROOT=/usr/local
alias morsemake="rm -rf build && mkdir build && cd build && cmake .. && make && fakeroot make package && sudo dpkg -i openrobots-morse-*-Linux.deb"
alias morsetest="$MORSE_BLENDER -setaudio NULL -noglsl -noaudio -nojoystick --background /usr/local/share/morse/data/morse_default.blend --python $(pwd)/.travis-test.py && ls saved.blend"

