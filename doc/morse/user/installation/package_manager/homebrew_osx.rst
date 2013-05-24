With ``homebrew`` on OSX
+++++++++++++++++++++++++

`Homebrew <http://mxcl.github.io/homebrew/>`_ is a package manager for OSX.
It can be installed in OSX with a single Ruby command: 
``ruby -e "$(curl -fsSL https://raw.github.com/mxcl/homebrew/go)"``


.. Note::
    Homebrew requires XCode and the command line tools to be installed.

Installing MORSE
-----------------

#. Download Blender from <http://www.blender.org/download/get-blender/> and
   copy the apps to /Applications.  Blender is not currently available 
   as a recipe in Homebrew. 
   
#. Install Python3 using Homebrew::
    
    brew install python3

#. Install MORSE using Homebrew::

    brew tap davidhodo/homebrew-morse
    brew install morse
      
Note: The Python3 version installed must exactly match the Python version 
used by Blender.  The latest Homebrew Python3 formula installs 
Python 3.3.1 while the latest Blender version uses 3.3.0.  To install
3.3.0 run the following commands before installing python3::

  cd /usr/local
  git checkout 864e9f1 /usr/local/Library/Formula/python3.rb

The provided hash can be found by running::

  brew versions python3



Formula Options
-----------------

The default Homebrew formula configures and installs MORSE with support
for only the sockets middleware.  The following formula options are 
available to enable support for other middlewares:
 
- --with-ros
- --with-moos
- --with-pocolibs
- --with-yarp2

Additionally documentation generating documenation and HLS support can
be enabled with the following flags:

- --with-doc
- --with-hla

E.g. to install MORSE with generated documenation and support for ROS::

  brew install morse --with-doc --with-ros

Updating
-----------------


To update to the latest version of MORSE after installation::

  brew update
  brew install --upgrade morse

Uninstalling
-----------------


To uninstall MORSE::

  brew uninstall morse
  

ROS Installation Issues
-----------------------

ROS for OSX can be installed from source using Homebrew with the 
instructions given at 
<http://www.ros.org/wiki/groovy/Installation/OSX/Homebrew/Source>.

In addition to the instructions provided on the ROS website, a Python 3
version of rospkg must be installed for MORSE to operate properly::

  sudo pip3 install rospkg
