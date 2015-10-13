import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.sensor import Sensor
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi

class Collision(Sensor):
    """
    Sensor to detect objects colliding with the current object.

    It allows to select which objects are sensed for collision by setting
    the property ``only_objects_with_property`` (see the Examples section for
    an example).

    This sensor is a wrapper around Blender's own `Collision
    <https://www.blender.org/manual/game_engine/logic/sensors/collision.html>`_
    sensor.

    .. example::

        from morse.builder import *

        # adds a default robot (the MORSE mascott!)
        robot = Morsy()

        # create and add a Collision sensor
        collision = Collision()

        # place it on the front side of the robot
        collision.translate(0.43,0,0.3)

        # only detect collision with objects which have the property 'Object'.
        # see the documentation of `Passive Objects` for details:
        # http://www.openrobots.org/morse/doc/latest/user/others/passive_objects.html
        collision.properties(only_objects_with_property="Object")

        robot.append(collision)
        # for this example, we use the socket interface
        collision.add_interface("socket")

        # we also add a keyboard actuator to be able to move
        # around our robot to test collisions
        keyboard = Keyboard()
        robot.append(keyboard)

        # the 'sandbox' test environment offers plenty of objects to test
        # collisions. These objects have all the property 'Object' already set.
        env = Environment('sandbox')

        # Copy this code to a script, and run it with `morse run <script>`!

    .. example::

        # This is a sample *client* code that uses pymorse to count the
        # collisions. Copy this code to a different script, start it with
        # `python3 <script>`, and move the robot in the simulator with the
        # arrow keys: when you collide with an object, it is printed on the
        # console.

        import pymorse

        nb_collisions = 0

        def counter(data):
            global nb_collisions

            if data["collision"]:
                nb_collisions += 1
                print("Collision with %s! In total, %d collisions occured" % (data["objects"], nb_collisions))


        with pymorse.Morse() as morse:

            morse.robot.collision.subscribe(counter)

            print("Press ctrl+C to stop")
            while True:
                morse.sleep(10)

    :noautoexample:
    """
    _name = "Collision"
    _short_desc = "Detect objects colliding with the current object."

    add_data('collision', False, "bool", "objects colliding with the current object")
    add_data('objects', "", "string", "A list of colliding objects.")

    # These properties are not used directly in the logic, but are used
    # in the builder to create the radar properly.
    # These value cannot be changed dynamically in bge.
    add_property('_collision_property', "", 'only_objects_with_property', 'string',
                 'Only report collision with objects that have this property, '
                 'default "" (all objects)')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Sensor.__init__(self, obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Is currently in collision """
        controller  = blenderapi.controller()
        sensor = controller.sensors[-1]

        # see hitObjectList and hitObject for last collided object(s)
        # http://www.blender.org/api/blender_python_api_2_76_release/bge.types.KX_TouchSensor.html
        self.local_data['collision'] = sensor.positive
        self.local_data['objects'] = ','.join([o.name for o in sensor.hitObjectList])
