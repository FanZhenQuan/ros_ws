#!/usr/bin/env python

import subprocess
import os


if __name__ == '__main__':
    path_to_file = '/home/davide/ros_ws/src/learning/learning_navigation/maps/mymap.yaml'

    try:
        open(path_to_file, 'r')

        subprocess.call(['roslaunch', 'turtlebot3_gazebo', 'amcl_with_map.launch'])
        os._exit(1)
    except IOError:
        subprocess.call(['roslaunch', 'turtlebot3_gazebo', 'gmapping.launch'])
    except KeyboardInterrupt:
        os._exit(1)
