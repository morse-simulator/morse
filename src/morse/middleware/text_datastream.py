import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import re
from morse.core.datastream import *
from morse.middleware.abstract_datastream import AbstractDatastream

class BasePublisher(AbstractDatastream):
    def initialize(self):
        self.filename = self._get_filename()
        self.file = open(self.filename, 'wb')
        self.index = 0

        line = self.header()
        self.file.write(line.encode())

    def finalize(self):
        self.file.close()

    def _get_filename(self):
        if 'file' in self.kwargs:
            return self.kwargs['file']
        else:
            filename = re.sub(r'\.([0-9]+)', r'\1', self.component_name)
            return filename + '.txt'

    def default(self, ci):
        line = self.encode_data()
        self.index = self.index + 1
        self.file.write(line.encode())
        self.file.flush()

    def header(self):
        return ""

    def encode_data(self):
        return ""

class Publisher(BasePublisher):

    _type_name = "key = value format with timestamp and index value"
    def header(self):
        lines = []
        lines.append('ROBOT %s || SENSOR %s\n' % 
                (self.component_instance.robot_parent.name(),
                 self.component_name))
        lines.append('(distance, globalVector(3), localVector(3))\n')
        lines.append(repr(self.component_instance.relative_position) + '\n\n')
        return ''.join(lines)

    def encode_data(self):
        parent_position = self.component_instance.robot_parent.position_3d
        lines = []
        lines.append('==> Data at X,Y,Z: [%.6f %.6f %.6f]'
                     'yaw,pitch,roll: [%.6f %.6f %.6f] | index %d | time %.2f\n'
           % (parent_position.x, parent_position.y, parent_position.z,
              parent_position.yaw, parent_position.pitch, parent_position.roll,
              self.index, blenderapi.persistantstorage().current_time))
        for variable, data in self.data.items():
            if isinstance(data, float):
                lines.append("\t%s = %.6f\n" % (variable, data))
            else:
                lines.append("\t%s = %s\n" % (variable, repr(data)))
        return ''.join(lines)

class CSVPublisher(BasePublisher):
    _type_name = "CSV like : values separated by semi-column"

    def header(self):
        lines = []
        lines.append('ROBOT %s || SENSOR %s\n' % 
                (self.component_instance.robot_parent.name(),
                 self.component_name))
        lines.append('(distance, globalVector(3), localVector(3))\n')
        lines.append(repr(self.component_instance.relative_position) + '\n\n')
        return ''.join(lines)

    def encode_data(self):
        lines = []
        for variable, data in self.data.items():
            if isinstance(data, float):
                lines.append("%.6f;" % data)
            else:
                lines.append("%s;" % repr(data))
        return ''.join(lines) + '\n'

class Text(Datastream):
    """ Produce text files as output for the components """

