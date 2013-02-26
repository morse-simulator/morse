#!/bin/bash
echo "MORSE installation and builder test"
echo "========================================"
lsb_release -a; uname -a; file $(which ls)
[[ -z "$(uname -p | grep 64)" ]] && arch="i686" || arch="x86_64"
BLENDER="blender-2.66-linux-glibc211-$arch"
echo "Download Blender ${BLENDER}"
(wget -q http://download.blender.org/release/Blender2.66/${BLENDER}.tar.bz2
tar jxf ${BLENDER}.tar.bz2 )& blenderpid=$!
echo "Download CMake Python3 libSDL"
sudo apt-get update -q
sudo apt-get install -mq cmake python3.2-dev libsdl1.2debian
echo "========================================"
echo "Check if libSDL was installed"
ls -l /usr/lib/i386-linux-gnu/libSDL*
echo "Setup MORSE env"
export MORSE_BLENDER=$(pwd)/${BLENDER}/blender
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.2/site-packages
export MORSE_ROOT=/usr/local
echo "Build and install MORSE"
mkdir build && cd build && cmake -DPYMORSE_SUPPORT=ON .. && make && sudo make install
wait $blenderpid # Blender should be ready now
echo "Run Blender in background mode (travis=headless). Test the builder."
echo "========================================"
morselib=${MORSE_ROOT}/share/morse
$MORSE_BLENDER -setaudio NULL -b ${morselib}/data/morse_default.blend -P ${morselib}/examples/scenarii/travis-save.py
echo "========================================"
ls -l travis-succeed.blend && echo "Bravo ! MORSE installation completed."
