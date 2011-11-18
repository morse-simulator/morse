#! /usr/bin/env python

import unittest
from morse.testing.testing import MorseTestRunner

from base.base_testing import BaseTest
suite = unittest.TestLoader().loadTestsFromTestCase(BaseTest)

# Comment these lines if you do not want to test ROS support
from middlewares.ros.ros_testing import RosTest
suite.addTests(unittest.TestLoader().loadTestsFromTestCase(RosTest))

MorseTestRunner().run(suite)

