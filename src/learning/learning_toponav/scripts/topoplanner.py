#!/usr/bin/env python

import rospy
from threading import Thread
import yaml
import random
import argparse
import networkx as nx

from pprint import pprint
from topological_node import TopologicalNode
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from learning_toponav.msg import *


class Planner(object):
    def __init__(self, adjlist, yaml):
        self.graph = nx.read_adjlist(adjlist, delimiter=', ', nodetype=str)
        
        self.destinations = []
        for n in list(self.graph.nodes()):
            d = {
                'name': n,
                'available_robots': True
            }
            self.destinations.append(d)
        
        self.nodes = rospy.get_param(yaml['interest_points'])
        self.yaml = yaml

        colors = rospy.get_param('/colors/')
        self.namespaces = ['/' + ns for color, ns in colors.items()]
        self.__availability_checker = Thread(target=self.find_available_robots, args=(self.namespaces,))
        self.__availability_checker.start()
        
    def on_shutdown(self):
        self.__availability_checker.join()
        
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
            
    def find_available_robots(self, robots):
        self.available_robots = []
        
        while not rospy.is_shutdown():
            for r in robots:
                topic = r + self.yaml['robot_state']
                msg = rospy.wait_for_message(topic, RobotState)
            
                if msg.state == 'ready':
                    self.available_robots.append(r)
                    self.update_dests_availability(add=msg.latest_goal)
                else:
                    self.update_dests_availability(remove=msg.current_goal)
        
            rospy.sleep(1)

    def get_robot_state(self, robot_ns):
        topic = robot_ns + self.yaml['robot_state']
        msg = rospy.wait_for_message(topic, RobotState)
    
        return msg.state

    def get_robot_afference(self, robot_ns):
        topic = robot_ns + self.yaml['afference']
        msg = rospy.wait_for_message(topic, RobotAfference)
    
        return msg.ipoint_name

    def update_dests_availability(self, add=None, remove=None):
        for d in self.destinations:
            if add and d['name'] == add:
                d['available'] = True
            
            if remove and d['name'] == remove:
                d['available'] = False
    
    def find_path(self, source, dest):
        """
        :param source: source of planning (type: str, indicates a WayPoint)
        :param dest: destination of planning (type: str, indicates a WayPoint)
        :return: list of WayPoints (type:str) to traverse in order to reach the goal
        """
        try:
            return nx.dijkstra_path(self.graph, source, dest)
        except nx.NetworkXNoPath as e:
            rospy.logerr(str(e))
            return None

    def build_ipoint_msg(self, node):
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

    def build_topopath(self, path):
        """
        :param path: il path, i cui membri sono stringhe del tipo "WayPointX"
        :return: topopath, ovvero il path in formato messaggio
        """
        toponav_ipoints = []
        for p in path:
            node = self._find_node_by_name(p)
            ipoint = self.build_ipoint_msg(node)
            toponav_ipoints.append(ipoint)
        toponav_ipoints.pop(0)  # removes the start point (source)
    
        return RobotTopopath(toponav_ipoints)

    def dispatch_goals(self):
        robots_num = len(self.namespaces)
        
        rate = rospy.Rate(robots_num)
        i = 0
        while not rospy.is_shutdown():
            if not self.available_robots:
                rospy.sleep(1)
            else:
                robot = self.available_robots.pop(0)
                source = self.get_robot_afference(robot)
                dest = self.choose_destination(robot)
                path = self.find_path(source=source, dest=dest)
                
                topopath = self.build_topopath(path)
                self.publish_path(topopath, robot)
                rospy.loginfo('Topoplanner: navrequest from %s fulfilled, path sent' % robot)
                
                if i == robots_num - 1:
                    i = 0
                else:
                    i += 1

    def choose_destination(self, robot):
        # per ora, si ignora una destinazione "intelligente" per il robot
        availables = []
        for dest in self.destinations:
            if dest['available_robots']:
                availables.append(dest['name'])
        
        return random.choice(availables)

    def listen_navrequests(self):
        nav_request = rospy.Subscriber(self.yaml['planner_requests'], RobotNavRequest, self._on_nav_request)
    
    def _on_nav_request(self, request):
        path = self.find_path(request.source, request.dest)
        topopath = self.build_topopath(path)

        self.publish_path(topopath, request.robot_name)
        
        rospy.loginfo('Topoplanner: navrequest from %s fulfilled, path sent' % request.robot_name)
            
    def publish_path(self, path, target_robot):
        pub = rospy.Publisher('/' + target_robot + self.yaml['robot_topopath'], RobotTopopath, queue_size=10)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        pub.publish(path)
    
    
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
    args = parse_args()
    yaml = parse_yaml(args.yaml)
    
    rospy.init_node('topoplanner')
    
    planner = Planner(args.adjlist, yaml=yaml)
    
    # rospy.on_shutdown(planner.on_shutdown)
    
    # planner.listen_navrequests()
    planner.dispatch_goals()
    
    # rospy.spin()