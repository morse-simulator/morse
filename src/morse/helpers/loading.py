"""Loading
Helpers for 'classpath' configuration
"""
import logging; logger = logging.getLogger("morse." + __name__)
import sys

def load_module_attribute(module_name, attribute_name):
    """Dynamically import a Python class."""
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

def create_instance(classpath, level, *args, **kwargs):
    """Creates an instances of a class."""

    module_name, class_name = classpath.rsplit('.', 1)
    klass = load_module_attribute(module_name, class_name)

    if level and not level == "default":
        if not hasattr(klass, "_levels"):
            logger.error("Class <%s> does not define abstraction levels. You can not use them here." % str(classpath))
            return None

        if level not in klass._levels:
            logger.error("Class <%s> does not define the abstraction level <%s>. Check your scene." % (classpath, level))
            return None

        return create_instance(klass._levels[level][0], None, *args, **kwargs)

    if not klass:
        logger.error("Could not create an instance of %s"%str(classpath))
        return None
    try:
        return klass(*args, **kwargs)
    except ValueError:
        return None
