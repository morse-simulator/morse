from collections import OrderedDict
import inspect


def add_data(name, default_value, type = "", doc = "(no documentation available yet)"):

    curframe = inspect.currentframe()
    try:
        calframe = inspect.getouterframes(curframe, 2)
        try:
            cls_locals =  calframe[1][0].f_locals
            if not "_data_fields" in cls_locals:
               cls_locals["_data_fields"] = OrderedDict()
            cls_locals["_data_fields"][name] = (default_value, type, doc)
        finally:
            del calframe
    finally:
        del curframe


def add_property(python_name, default_value, name, type = "", doc = "(no documentation available yet)"):
    """ Add a property to the current class of component

    :param python_name: name of the Python variable. It will be
    dynamically added to the component Python script.
    :param default_value: the default value
    :param string name: the name of the property. If used in 
    the Blender logic bricks, it must match the Blender name.
    :param type: type of the property, for documentation
    :param doc: description of the property.
    """
    curframe = inspect.currentframe()
    try:
        calframe = inspect.getouterframes(curframe, 2)
        try:
            cls_locals =  calframe[1][0].f_locals
            if not "_properties" in cls_locals:
               cls_locals["_properties"] = OrderedDict()
            cls_locals["_properties"][name] = (default_value, type, doc, python_name)
        finally:
            del calframe
    finally:
        del curframe

