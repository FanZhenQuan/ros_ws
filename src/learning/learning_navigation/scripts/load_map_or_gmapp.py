#!/usr/bin/env python

import subprocess
import sys


if __name__ == '__main__':
    path_to_file = '/home/davide/ros_ws/src/learning/learning_navigation/maps/mymap.yaml'

    try:
        open(path_to_file, 'r')

        subprocess.call(['roslaunch', 'turtlebot3_gazebo', 'amcl_with_map.launch'])
        sys.exit(1)
    except IOError:
        subprocess.call(['roslaunch', 'turtlebot3_gazebo', 'gmapping.launch'])
    except KeyboardInterrupt:
        sys.exit(1)
