#!/usr/bin/env python

# import rospy
from topological_map import TopologicalMap

if __name__ == '__main__':
    map_path = '/home/davide/ros_ws/src/learning/learning_toponav/maps/mymap.yaml'
    topomap = TopologicalMap(filename=map_path)
    
    topomap.debug()