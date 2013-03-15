import sys
import os
import logging
import re
import shutil

try:
    # If Pygments is installed, use it to nicely render Python code.
    from pygments import highlight
    from pygments.lexers import PythonLexer
    from pygments.formatters import Terminal256Formatter

    def pyprint(code):
        print(highlight(code, PythonLexer(), Terminal256Formatter(style="monokai")))

except ImportError:

    def pyprint(code):
        print(code)




from morse.core.exceptions import MorseEnvironmentError

logger = logging.getLogger('morse')
##


substr = re.compile("@(.*?)@")

DEFAULT_TEMPLATES_PATH = "share/morse/data/templates"

# no good way to know if a file is binary or not. So use a list of extensions
BINARY_FILE_EXT = [".blend"] 

joinpth = os.path.join # little shortcut...

###################

BASIC = ["default.py", 
         "scripts/@env@_client.py",
         "src/@env@/__init__.py", 
         "src/@env@/builder/__init__.py"]

ROBOT = ["src/@env@/robots/@name@.py", 
         "src/@env@/robots/__init__.py", 
         "src/@env@/builder/robots/@name@.py", 
         "src/@env@/builder/robots/__init__.py", 
         "data/@env@/robots/@name@.blend"]

SENSOR = ["src/@env@/sensors/@name@.py", 
          "src/@env@/sensors/__init__.py", 
          "src/@env@/builder/sensors/@name@.py",
          "src/@env@/builder/sensors/__init__.py"]

ACTUATOR = ["src/@env@/actuators/@name@.py", 
            "src/@env@/actuators/__init__.py", 
            "src/@env@/builder/actuators/@name@.py",
            "src/@env@/builder/actuators/__init__.py"]

###################

NEW_ROBOT_MSG = ["""
A template for a new robot called <{name}> has been added to the 
<{env}> environment.

----------------------------------------------------------
To complete the equipment of your robot, edit:
{prefix}/src/builder/robots/{name}.py

You can also modify its Blender mesh:
{prefix}/data/robots/{name}.blend

For advanced usage, you may also edit its internal 
definition here:
{prefix}/src/robots/{name}.py
----------------------------------------------------------

To use the robot in your simulation script, add the following
lines:

""", ("""from {env}.builder.robots import {classname}

# add a new {name} robot in the simulation
{name} = {classname}()
#{name}.translate(...)
#{name}.add_default_interface('socket')

""", 'python'),
"""----------------------------------------------------------
Happy simulation!
"""]

class Environment():

    def __init__(self, morse_prefix, env_name, env_path = None):
        """
        :param morse_prefix" the installation 
            prefix of MORSE
        :param env_name: the name of the environment
            we want to modify.
        :param env_path: the relative or absolute 
            path to the environment. Default to `env_name`.
        """
        self.morse_prefix = morse_prefix
        self.env = env_name
        self.path = env_path or env_name
        self.abspath = os.path.abspath(self.path)

        self.tpls = joinpth(os.path.normpath(self.morse_prefix), \
                            os.path.normpath(DEFAULT_TEMPLATES_PATH))

    def _make_safe_name(self, name):
        tmp = "".join(c for c in name if c.isalnum() or c==' ').rstrip()
        tmp.replace(" ", "_")
        if tmp[0].isdigit():
            tmp = "_" + tmp
        return tmp

    def _substitute_str(self, instr, **kwargs):
        res = instr
        for kw in substr.findall(instr):
            if kw in kwargs:
                res = res.replace("@%s@" % kw, kwargs[kw])
        return res

    def _configure(self, origin, target, **kwargs):
        """ Create a temporary file which is the configured version of
        ``origin``: every @keyword@ present in ``origin`` is replaced by
        the value passed as named argument.

        The configured file is saved as ``target``

        For instance:

        test.tpl:
            print("@text@")

        -> invokation:

        _configure("test.tpl", "test.py", text = "Hello World")

        """

        # do not configure binary files
        if os.path.splitext(target)[1] in BINARY_FILE_EXT:
            shutil.copy(origin, target)
            return

        with open(origin, 'r') as infile:
            content = infile.readlines()
            if content and "#APPEND" in content[0]:
                with open(target, 'a') as outfile:
                    for l in content[1:]:
                        outfile.write(self._substitute_str(l, **kwargs))

            else:
                with open(target, 'w') as outfile:
                    for l in content:
                        outfile.write(self._substitute_str(l, **kwargs))

    def check_writable(self, path):
        if not os.access(path, os.W_OK):
            raise MorseEnvironmentError("You do not have write permission "
                                        "in <%s>!" % self.path)

    def check_env_exists(self):
        if not os.access(self.path, os.F_OK):
            raise MorseEnvironmentError("<%s> (expected in \"%s\") does not "
                                        "exist! You may want to create first "
                                        "the environment with 'morse init "
                                        "<env>'." % (self.env, self.path))

        self.check_writable(self.path)

    def _install_files(self, files, **kwargs):

        for f in files:
            path, name = os.path.split(f)

            newpath = self._substitute_str(path, **kwargs)
            newname = self._substitute_str(name, **kwargs)

            try:
                os.makedirs(joinpth(self.path, newpath))
            except OSError: # path already exists
                pass

            self._configure(joinpth(self.tpls, path, name + ".tpl"), \
                            joinpth(self.path, newpath, newname), \
                            **kwargs)


    def _print_info_msg(self, msg, **kwargs):
        for part in msg:
            if isinstance(part, tuple) and part[1] == 'python':
                part = part[0].format(**kwargs)
                pyprint(part)
            else:
                part = part.format(**kwargs)
                print(part)

    def create(self, force = False):
        """ Initializes a new environment content.
        """

        self.check_writable(".")

        if os.access(self.path, os.F_OK):
            if force:
                shutil.rmtree(self.path)
            else:
                logger.error("A directory called \"%s\" already "
                             "exists!" % self.path)
                sys.exit()

        self._install_files(BASIC, env = self.env)

    def add_component(self, cmpttype, name):

        self.check_env_exists()

        safename = self._make_safe_name(name)
        if safename != name:
            logger.warning("Replace name <%s> by suitable identifier: "
                           "<%s>" % (name, safename))

        if cmpttype == "robot":
            self._install_files(ROBOT, \
                                name = safename, \
                                classname = safename.capitalize(), \
                                env = self.env)

            self._print_info_msg(NEW_ROBOT_MSG, \
                                 prefix= self.abspath, \
                                 name = safename, \
                                 classname = safename.capitalize(), \
                                 env = self.env)
        elif cmpttype == "sensor":
            self._install_files(SENSOR, name = safename)
        elif cmpttype == "actuator":
            self._install_files(ACTUATOR, name = safename)
        else:
            raise MorseEnvironmentError("Unknown component type %s" % cmpttype)
