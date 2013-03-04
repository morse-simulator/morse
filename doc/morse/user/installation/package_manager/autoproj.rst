With ``autoproj``
+++++++++++++++++

Autoproj is a package manager for robotic software.
It is part of the `Rock project <http://rock-robotics.org/>`_, and is the 
standard installation process for the `Orocos toolchain <http://www.orocos.org/toolchain>`_.

.. Note::
    If you already have an autoproj installation ready, skip directly to step 2.

#. Install an empty autoproj tree by following the `Autoproj Bootstraping <http://rock-robotics.org/documentation/autoproj/bootstrap.html>`_.
   On Debian-based systems, Autoproj can manage OS dependencies if you have 
   sudo permissions: answer ``all`` to the dependencies question.
   On other systems, it is advised to manage OS dependencies out of Autoproj
   (by answering ``ruby``).
   
#. Edit the ``autoproj/manifest`` file and add to the relevant sections::
    
    package_sets:
      - type: git
        url: git://gitorious.org/robotis/morse_package_set.git
        branch: 1.0
    
    layout:
      - morse

#. If you have already installed some packages out of Autoproj, you can tell 
   Autoproj not to install them by adding to the manifest file, for instance::
    
    ignore_packages:
      - blender
      - yarp

#. Type ``source env.sh``.
#. Type ``autoproj update morse``; Autoproj will ask you wether you want to 
   install YARP and HLA support for MORSE. Il will also download all required
   dependencies.
#. Type ``autoproj build morse``; Autoproj will compile for you all the
   required dependencies, including Blender.
#. Source the ``env.sh`` file again; MORSE is ready for use (type ``morse check``).
