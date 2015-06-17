#!/bin/bash -e
echo "MORSE ci setup and test"
echo "======================="
lsb_release -a; uname -a

echo "Setup MORSE env"
# Avoid ALSA errors on system without sound card
echo -e "#! /bin/sh\nblender -setaudio NULL \$@" > blender
chmod +x blender
export MORSE_BLENDER=$(pwd)/blender

workspace=$(pwd)
echo "Install virtualenv and numpy for python3.4"
version=13.1.0
wget https://github.com/pypa/virtualenv/archive/${version}.tar.gz
tar xf ${version}.tar.gz
cd virtualenv-${version}
export PATH=${PATH}:${HOME}/.local/bin
python3.4 setup.py install --user # install in ~/.local

virtualenv pyvenv
source pyvenv/bin/activate
pip install numpy
cd ${workspace}

export PYTHONPATH=${workspace}/pyvenv/lib/python3.4/site-packages:$PYTHONPATH

echo "Build and install MORSE"
mkdir build && cd build
cmake -DPYTHON_EXECUTABLE=$(which python3.4) -DPYMORSE_SUPPORT=ON -DCMAKE_INSTALL_PREFIX=${workspace}/pyvenv ..
make install

export PATH=${PATH}:${workspace}/pyvenv/bin

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
