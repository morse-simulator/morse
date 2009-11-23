### Functions to access properties of objects in Blender
import Blender

# Read the value of the property in an object
def read_property(object, property_name):
    property = object.getProperty(property_name)
    type = property.getType()
    value = property.getData()
    print "Property: ", object.name, ".", property_name, " = ", value, " (", type, ")"
    return value

# Change the value of a property in an object
def write_property(object, property_name, value):
    property = object.getProperty(property_name)
    # print "Going to store amount: '{0}'".format(value)
    property.setData(value)


# Check whether an object has a specific property
def has_property(object, property_name):
	try:
		data = object[property_name]
	except KeyError:
		return False

	return True


# End of script
