#!/bin/bash
(wget -q http://download.blender.org/release/Blender2.64/blender-2.64a-linux-glibc27-i686.tar.bz2
tar jxf blender-2.64a-linux-glibc27-i686.tar.bz2 )& blenderpid=$!
sudo apt-get install cmake python3.2-dev libsdl-sge python3-sphinx
export MORSE_BLENDER=$(pwd)/blender-2.64a-linux-glibc27-i686/blender
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages:/usr/local/lib/python3.2/site-packages
export MORSE_ROOT=/usr/local
# Build and install MORSE
mkdir build && cd build && cmake -DPYMORSE_SUPPORT=ON .. && make && sudo make install
wait $blenderpid # Blender should be ready now
# Run Blender in background mode (travis=headless). Test the builder.
$MORSE_BLENDER -setaudio NULL -noglsl -noaudio -nojoystick -b /usr/local/share/morse/data/morse_default.blend -P /usr/local/share/morse/examples/tutorials/tutorial-1-sockets.py
echo "Bravo ! MORSE installation completed."
