#!/bin/bash -e
echo "MORSE installation and builder test"
echo "========================================"
lsb_release -a; uname -a
[[ -z "$(uname -p | grep 64)" ]] && arch="i686" || arch="x86_64"
BLENDER="blender-2.64a-linux-glibc27-$arch"

echo "Download Blender ${BLENDER}"
(wget -q http://download.blender.org/release/Blender2.64/${BLENDER}.tar.bz2
tar jxf ${BLENDER}.tar.bz2 )& blenderpid=$!

echo "Setup MORSE env"
echo -e "#! /bin/sh\n$(pwd)/${BLENDER}/blender -setaudio NULL \$@" > blender
chmod +x blender
export MORSE_BLENDER=$(pwd)/blender
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.2/site-packages

echo "Build and install MORSE"
mkdir build && cd build
cmake -DPYMORSE_SUPPORT=ON ..
make
sudo make install
wait $blenderpid # Blender should be ready now

echo "Run testing/base/pose_testing.py"
echo "========================================"

touch PoseTest.log
tail -f PoseTest.log &
xvfb-run --server-args="-screen 0 1x21x16" python3 ../testing/base/pose_testing.py
kill % # kill tail
