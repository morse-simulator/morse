"""Loading
Helpers for 'classpath' configuration
"""
import logging; logger = logging.getLogger("morse." + __name__)
import sys

def get_class(module_name, class_name):
    """Dynamically import a Python class."""
    try:
        __import__(module_name)
    except ImportError as detail:
        logger.error("Module not found: %s" % detail)
        return None
    module = sys.modules[module_name]
    # Create an instance of the object class
    try:
        klass = getattr(module, class_name)
    except AttributeError as detail:
        logger.error("Module attribute not found: %s" % detail)
        return None
    return klass

def create_instance(classpath, *args, **kwargs):
    """Creates an instances of a class."""
    module_name, class_name = classpath.rsplit('.', 1)
    klass = get_class(module_name, class_name)
    if not klass:
        logger.error("Could not create an instance of %s"%str(classpath))
        return None
    return klass(*args, **kwargs)
