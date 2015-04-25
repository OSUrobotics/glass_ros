#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    package_dir={'': 'src'},
    scripts=['scripts/face_frame.py', 'scripts/glassSensorBridge.py', 'scripts/kalman_node.py', 'scripts/move_head.py', 'scripts/start_glass', 'scripts/stop_glass'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg']
)

setup(**d)
