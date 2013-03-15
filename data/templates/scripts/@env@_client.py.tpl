#! /usr/bin/env python3
"""
Test client for the <@env@> simulation environment.

This simple program shows how to control a robot from Python.

For real applications, you may want to rely on a full middleware,
like ROS (www.ros.org).
"""

import sys

try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

print("Use WASD to control the robot")

with Morse() as simu:

  motion = simu.robot.motion
  pose = simu.robot.pose

  v = 0.0
  w = 0.0

  while True:
      key = input("WASD?")

      if key.lower() == "w":
          v += 0.1
      elif key.lower() == "s":
          v -= 0.1
      elif key.lower() == "a":
          w += 0.1
      elif key.lower() == "d":
          w -= 0.1
      else:
          continue

      # here, we call 'get' on the pose sensor: this is a blocking
      # call. Check pymorse documentation for alternatives, including
      # asynchronous stream subscription.
      print("The robot is currently at: %s" % pose.get())

      motion.publish({"v": v, "w": w})
