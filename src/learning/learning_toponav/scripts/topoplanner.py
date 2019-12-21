#!/usr/bin/env python

import rospy
import yaml
import random
import argparse
import networkx as nx
import json

from threading import Thread
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
                'available': True
            }
            self.destinations.append(d)
        
        self.nodes = rospy.get_param(yaml['interest_points'])
        self.yaml = yaml

        colors = rospy.get_param('/colors/')
        self.robot_namespaces = ['/' + ns for color, ns in colors.items()]
        
        self.available_robots = []
        self.__availability_checker = Thread(
            target=self.find_available_robots
        )
        self.__availability_checker.start()
        
        # ---------------
        dests_logger = Thread(target=self.destinations_log)
        dests_logger.start()
        
    def debug(self):
        # crash test
        path_rbt1 = self.find_path('WayPoint2', 'WayPoint1')
        path_rbt2 = self.find_path('WayPoint1', 'WayPoint2')
        
        tp_rbt1 = self.build_topopath(path_rbt1)
        tp_rbt2 = self.build_topopath(path_rbt2)
        
        while not self.available_robots:
            rospy.sleep(1)
        
        self.publish_path(tp_rbt1, '/robot_1')
        self.publish_path(tp_rbt2, '/robot_2')
        
        rospy.loginfo('Goal debug consegnati')
        
        rospy.spin()

    def _get_node_by_name(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n
            
    def _get_posit_by_nodename(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n['pose']['position']
            
    def _get_verts_by_nodename(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n['verts']

    def get_robot_state(self, robot_ns):
        # TODO: check usage
        topic = robot_ns + self.yaml['robot_state']
        msg = rospy.wait_for_message(topic, RobotState)
    
        return msg.state

    def get_robot_afference(self, robot_ns):
        topic = robot_ns + self.yaml['afference']
        msg = rospy.wait_for_message(topic, RobotAfference)
    
        return msg.ipoint_name

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
            node = self._get_node_by_name(p)
            ipoint = self.build_ipoint_msg(node)
            toponav_ipoints.append(ipoint)
        toponav_ipoints.pop(0)  # removes the start point (source)
    
        return RobotTopopath(toponav_ipoints)

    def find_available_robots(self):
        try:
            while not rospy.is_shutdown():
                for r in self.robot_namespaces:
                    state_topic = r + self.yaml['robot_state']
                    msg = rospy.wait_for_message(state_topic, RobotState)
                
                    if msg.state == 'ready':
                        self.available_robots.append(r)
                        self.update_available_dests(add=msg.latest_goal)
                    else:
                        self.update_available_dests(remove=msg.current_goal)
                        
                # LEAVE THIS AS IS, prevents multiple
                # goals being sent simultaneously at launch
                rospy.sleep(2)
        except rospy.ROSInterruptException:
            pass

    def update_available_dests(self, add=None, remove=None):
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

    def dispatch_goals(self):
        robots_num = len(self.robot_namespaces)
        
        rate = rospy.Rate(robots_num)
        i = 0
        
        try:
            while not rospy.is_shutdown():
                if not self.available_robots:
                    rospy.sleep(1)
                else:
                    robot = self.available_robots.pop(0)
                    source = self.get_robot_afference(robot)
                    dest = self.choose_destination(robot, source)
                    path = self.find_path(source=source, dest=dest)
                    
                    topopath = self.build_topopath(path)
                    self.publish_path(topopath, robot)
                    rospy.loginfo('Topoplanner: path from %s to %s sent to %s' % (source, dest, robot))
                    
                    if i == robots_num - 1:
                        i = 0
                    else:
                        i += 1
        except rospy.ROSInterruptException:
            pass

    def choose_destination(self, robot, source):
        # per ora, si ignora una destinazione "intelligente" per il robot
        availables = []
        for dest in self.destinations:
            if dest['available'] and dest['name'] != source:
                availables.append(dest['name'])
        
        return random.choice(availables)
    
    def destinations_log(self):
        pub = rospy.Publisher(self.yaml['destinations_log'], DestinationDebug, queue_size=30)
        rate = rospy.Rate(10)
        
        try:
            while not rospy.is_shutdown():
                for d in self.destinations:
                    msg = DestinationDebug()
                    msg.available = d['available']
                    msg.name = d['name']
                    
                    pub.publish(msg)
                    rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def listen_navrequests(self):
        nav_request = rospy.Subscriber(self.yaml['planner_requests'], RobotNavRequest, self._on_nav_request)
    
    def _on_nav_request(self, request):
        path = self.find_path(request.source, request.dest)
        topopath = self.build_topopath(path)

        self.publish_path(topopath, '/'+request.robot_name)
        
        rospy.loginfo('Topoplanner: path from %s to %s sent to %s' % (request.source, request.dest, request.robot_name))
        
    def publish_path(self, path, target_robot):
        pub = rospy.Publisher(target_robot + self.yaml['robot_topopath'], RobotTopopath, queue_size=10)
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
    rospy.init_node('topoplanner')
    
    args = parse_args()
    yaml = parse_yaml(args.yaml)
    
    planner = Planner(args.adjlist, yaml=yaml)
    # planner.listen_navrequests()
    # planner.debug()
    planner.dispatch_goals()
