#!/usr/bin/env python

import rospy
import yaml
import argparse
import networkx as nx

from pprint import pprint
from topological_node import TopologicalNode
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from learning_toponav.msg import *
# TODO: implementare un planner sciente, invia goal di sua volonta sapendo lo stato del robot


class Planner(object):
    def __init__(self, adjlist, yaml):
        self.graph = nx.read_adjlist(adjlist, delimiter=', ', nodetype=str)
        self.yaml = yaml
        self.nodes = rospy.get_param(yaml['interest_points'])
        
    def _find_node_by_name(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n
            
    def _find_posit_by_nodename(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n['pose']['position']
            
    def _find_verts_by_nodename(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n['verts']
        
    def find_path(self, source, dest):
        """
        :param source: source of planning (type: str, indicates a WayPoint)
        :param dest: destination of planning (type: str, indicates a WayPoint)
        :return: list of WayPoints (type:str) to traverse in order to reach the goal
        """
        return nx.dijkstra_path(self.graph, source, dest)
    
    def listen_navrequests(self):
        nav_request = rospy.Subscriber(self.yaml['planner_requests'], RobotNavRequest, self._on_nav_request)
    
    def _on_nav_request(self, request):
        try:
            path = self.find_path(request.source, request.dest)
            toponav_ipoints = []
            for p in path:
                node = self._find_node_by_name(p)
                ipoint = self.__build_ipoint_msg(node)
                toponav_ipoints.append(ipoint)
            toponav_ipoints.pop(0)  # removes the start point (source)

            topopath = RobotTopopath(toponav_ipoints)

            self.publish_path(topopath, request.robot_name)
            
            rospy.loginfo('Topoplanner: navrequest from %s fulfilled, path sent' % request.robot_name)
        except nx.NetworkXNoPath as e:
            rospy.logerr(str(e))
            
    def publish_path(self, path, target_robot):
        pub = rospy.Publisher('/' + target_robot + self.yaml['robot_topopath'], RobotTopopath, queue_size=10)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        pub.publish(path)
            
    def __build_ipoint_msg(self, node):
        position = Point(
            float(node['pose']['position']['x']),
            float(node['pose']['position']['y']),
            float(node['pose']['position']['z'])
        )
        orientation = Quaternion(0, 0, 0, 1)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        
        pose = PoseStamped()
        pose.header = header
        pose.pose = Pose(position, orientation)
        
        return ToponavIpoint(pose, node['name'])
    
    
def parse_yaml(dir):
    f = open(dir, 'r')
    return yaml.load(f)

    
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--adjlist', type=str, required=True)
    parser.add_argument('--source', type=str, required=False)
    parser.add_argument('--dest', type=str, required=False)
    parser.add_argument('--yaml', type=str, help='File where used topics_yaml are saved')
    args, unknown = parser.parse_known_args()
    
    return args


if __name__ == '__main__':
    rospy.init_node('topoplanner')
    
    args = parse_args()
    yaml = parse_yaml(args.yaml)
    
    planner = Planner(args.adjlist, yaml=yaml)
    planner.listen_navrequests()
    
    rospy.spin()