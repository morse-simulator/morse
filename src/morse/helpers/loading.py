"""Loading
Helpers for 'classpath' configuration
"""
import logging; logger = logging.getLogger("morse." + __name__)
import sys

def get_class(classpath):
    """ Returns the class object from a full classpath (like toto.tata.MyTata)
    """
    module_name, class_name = classpath.rsplit('.', 1)
    klass = load_module_attribute(module_name, class_name)

    if not klass:
        logger.error("Could not load the class %s in %s"% (class_name, module_name))
        return None

    return klass


def load_module_attribute(module_name, attribute_name):
    """Dynamically import a Python attribute."""
    try:
        __import__(module_name)
    except ImportError as detail:
        logger.error("Module not found: %s" % detail)
        return None
    module = sys.modules[module_name]
    # Create an instance of the object class
    try:
        attribute = getattr(module, attribute_name)
    except AttributeError as detail:
        logger.error("Module attribute not found: %s" % detail)
        return None
    return attribute

def create_instance(classpath, *args, **kwargs):
    """Creates an instances of a class."""

    klass = get_class(classpath)

    if not klass:
        logger.error("Could not create an instance of %s"%str(classpath))
        return None

    return klass(*args, **kwargs)


def create_instance_level(classpath, level, *args, **kwargs):
    """Creates an instances of a class from a component abstration level."""

    klass = get_class(classpath)

    if not klass:
        logger.error("Could not create an instance of %s"%str(classpath))
        return None

    # Component abstration levels
    if level:

        if not hasattr(klass, "_levels"):
            if level != "default":
                logger.error("Class <%s> does not define abstraction levels. You can not use them here." % str(classpath))
                return None

        if level == "default":
            # iterate over levels to find the one with the default flag
            for key, value in klass._levels.items():
                if value[2] == True:
                    level = key
                    logger.info("Using default level <%s> for component <%s>" % (level, classpath))
                    break

            if level == "default":
                logger.error("Class <%s> does not define a default abstraction level. You must explicitely set one with <cmpt>.level(<level>). Check the component documentation." % classpath)
                return None

        if level not in klass._levels:
            logger.error("Class <%s> does not define the abstraction level <%s>. Check your scene." % (classpath, level))
            return None

        # The level may define a custom classpath to implement the component
        # behaviour, or 'None' if the parent class is to be used.
        if klass._levels[level][0]:
            return create_instance(klass._levels[level][0], *args, **kwargs)

    return klass(*args, **kwargs)
