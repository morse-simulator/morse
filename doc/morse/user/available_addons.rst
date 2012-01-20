Blender add-ons for MORSE
=========================

Blender supports the concept of add-ons, as a collection of Python scripts
that implement additional functionality, such as loading of particular file
formats or the creation of specific object types.

A couple of these add-ons have been developed to import/export data from
MORSE into Blender, independently of the robotics functionality. These are
meant to help in visualising data and simplifying the work with the MORSE
Builder API.

Installing add-ons
------------------

The `official Blender documentation on add-ons <http://wiki.blender.org/index.php/Doc:2.5/Manual/Extensions/Python/Add-Ons>`_
explains how to install the Python scripts, either by hand or using the ``User preferences`` GUI.

The add-ons provided by MORSE are located in the source directory 
(where you downloaded the rest of MORSE), under the folder ``addons``,
or are installed in ``${PREFIX}/share/morse/addons``.

Currently available add-ons
---------------------------

.. toctree::
    :glob:
    :maxdepth: 1

    addons/*
