#!/bin/bash -e
echo "MORSE ci setup and test"
echo "======================="
lsb_release -a; uname -a

echo "Setup MORSE env"
# Avoid ALSA errors on system without sound card
echo -e "#! /bin/sh\nblender -setaudio NULL \$@" > blender
chmod +x blender
export MORSE_BLENDER=$(pwd)/blender

MORSE_PREFIX=~/morse_install

mkdir -p ${MORSE_PREFIX}

echo "Build and install MORSE"
mkdir build && cd build
cmake -DPYTHON_EXECUTABLE=$(which python3.4) -DCMAKE_INSTALL_PREFIX=${MORSE_PREFIX} ..
make install

export PATH=${PATH}:${MORSE_PREFIX}/bin
export PYTHONPATH=${MORSE_PREFIX}/lib/python3/dist-packages:$PYTHONPATH

morse_test() {
    echo "Run $1"
    echo "========================================"
    touch $2
    tail -f $2 &
    xvfb-run --auto-servernum --server-args="-screen 0 160x120x16" python3.4 ../testing/$1
    kill % # kill tail
}

morse_test base/gps_testing.py GPSTest.log
morse_test base/pose_testing.py PoseTest.log
morse_test base/sick_testing.py Sick_Test.log
