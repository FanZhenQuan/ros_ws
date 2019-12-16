#!/usr/bin/env python

import rospy
import argparse
import networkx as nx

from pprint import pprint
from toponodes_publisher import INTEREST_POINTS
from topological_node import TopologicalNode
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from learning_toponav.msg import RobotNavRequest, RobotTopopath, ToponavIpoint


PLANNER_REQUESTS_TOPIC = '/topoplanner/nav_request'
ROBOT_TOPOPATH_TOPIC = '/topopath'


class Planner(object):
    def __init__(self, edgelist):
        self.graph = nx.read_adjlist(edgelist, delimiter=', ', nodetype=str)
        self.nodes = rospy.get_param(INTEREST_POINTS)
        self.nav_request = rospy.Subscriber(PLANNER_REQUESTS_TOPIC, RobotNavRequest, self._on_nav_request)
        
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
    
    def _on_nav_request(self, request):
        try:
            path = self.find_path(request.source, request.dest)
            toponav_ipoints = []
            for p in path:
                node = self._find_node_by_name(p)
                ipoint = self.__build_msg(node)
                toponav_ipoints.append(ipoint)
                
            toponav_ipoints.pop(0)  # removes the start point (source)

            topopath = RobotTopopath(toponav_ipoints)

            pub = rospy.Publisher('/'+request.robot_name+ROBOT_TOPOPATH_TOPIC, RobotTopopath, queue_size=10)
            while pub.get_num_connections() < 1:
                rospy.sleep(0.1)
            pub.publish(topopath)
            
            rospy.loginfo('Topoplanner: navrequest from %s fulfilled, path sent' % request.robot_name)
        except nx.NetworkXNoPath as e:
            rospy.logerr(str(e))
            
    def __build_msg(self, node):
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

    
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--edgelist', type=str, required=True)
    parser.add_argument('--source', type=str, required=False)
    parser.add_argument('--dest', type=str, required=False)
    args, unknown = parser.parse_known_args()
    
    return args


if __name__ == '__main__':
    rospy.init_node('topoplanner')
    
    args = parse_args()
    
    planner = Planner(args.edgelist)
    
    if args.source and args.dest:
        source = 'WayPoint' + args.source
        dest = 'WayPoint' + args.dest
        
        path = planner.find_path(source, dest)
        rospy.loginfo(str(path))
    else:
        rospy.logwarn('Planner: no source and destination provided at launch')
        pass
    
    rospy.spin()