#!/usr/bin/env python

import argparse
import random
import yaml
import rospy
import networkx as nx

from pprint import pprint
from termcolor import colored
from robot import Robot
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from learning_toponav.msg import *
from destination import Destination, DestinationStatLogger
from std_msgs.msg import Header


class Planner(object):
    def __init__(self, adjlist, environment, yaml, logging):
        self.graph = nx.read_adjlist(adjlist, delimiter=', ', nodetype=str)
        self.yaml = yaml
        self.environment = environment  # house, office ...
        self.logging = logging  # bool, whether to print colored logs or not
        # self.logging = False
        self.available_robots = []
        
        self.destinations = []
        for n in rospy.get_param(yaml['interest_points']):
            d = Destination(
                name=n['name'],
                pose=Pose(
                    Point(
                        n['pose']['position']['x'],
                        n['pose']['position']['y'],
                        n['pose']['position']['z']
                    ),
                    Quaternion(0, 0, 0, 1)
                )
            )
            self.destinations.append(d)

        self.robots = []
        colors = rospy.get_param('/colors/')
        for color, ns in colors.items():
            if ns.startswith('/'):
                self.robots.append(Robot(ns=ns, color=color))
            else:
                self.robots.append(Robot(ns='/' + ns, color=color))

        # ---------------
        self.start_threads()
        self.update_robot_state()  # threaded in self.start_threads()
        
    def start_threads(self):
        dests_logger = Thread(target=self.destinations_log)
        dests_logger.start()
        
        # robot_state_updater = Thread(target=self.update_robot_state)
        # robot_state_updater.start()
        
    def update_robot_state(self):
        for r in self.robots:
            rospy.Subscriber(r.ns+self.yaml['robot_state'], RobotState, self.on_robot_state)
        
    def on_robot_state(self, msg):
        temp = Robot(ns=msg.robot_name,
                     state=msg.state,
                     c_goal=msg.current_goal,
                     l_goal=msg.latest_goal,
                     aff=msg.afference,
                     dist=msg.distance
                     )

        for r in self.robots:
            if r.ns == temp.ns:
                r.state = temp.state
                r.current_goal = temp.current_goal
                r.latest_goal = temp.latest_goal
                r.afference = temp.afference
                r.distance = temp.distance
                
        if temp.state == 'ready' and temp not in self.available_robots:
            self.available_robots.append(temp)
            self.log('%s is available' % temp.ns, 'blue', attrs=['bold'])

        # update destinations
        if msg.current_goal != 'None':
            if msg.latest_goal != 'None':
                self.update_available_dests(add=msg.latest_goal, remove=msg.current_goal)
            else:
                self.update_available_dests(remove=msg.current_goal)
        
    def debug(self):
        # crash test
        # start_1 = 'WayPoint2'; goal_1 = 'WayPoint1'
        # start_2 = 'WayPoint1'; goal_2 = 'WayPoint2'
        start_1 = 'WayPoint2'; goal_1 = 'WayPoint4'
        start_2 = 'WayPoint1'; goal_2 = 'WayPoint7'
        
        path_rbt1 = self.find_path(start_1, goal_1)
        path_rbt2 = self.find_path(start_2, goal_2)
        
        tp_rbt1 = self.build_topopath(path_rbt1)
        tp_rbt2 = self.build_topopath(path_rbt2)
        
        while not self.available_robots:
            rospy.sleep(1)
        
        self.publish_path(tp_rbt1, '/robot_1')
        self.publish_path(tp_rbt2, '/robot_2')
        
        rospy.loginfo('Goal debug consegnati')
        
        rospy.spin()

    def _get_node_by_name(self, name):
        for n in self.destinations:  # TODO: refactor
            if n.name == name:
                return n

    @staticmethod
    def build_ipoint_msg(node):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
    
        pose = PoseStamped()
        pose.header = header
        pose.pose = node.pose
    
        return ToponavIpoint(pose, node.name)

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

    def update_available_dests(self, add=None, remove=None):
        for d in self.destinations:
            if add and d.name == add:
                d.available = True
                continue
                
            if remove and d.name == remove and remove != add:
                d.available = False
    
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
        robots_num = len(self.robots)
        i = 0

        try:
            while not rospy.is_shutdown():
                if not self.available_robots:
                    rospy.sleep(1)
                else:
                    robot = self.available_robots.pop(0)
                    source = robot.afference
                    dest = self.choose_destination(source)
                    path = self.find_path(source=source, dest=dest.name)
                    
                    topopath = self.build_topopath(path)
                    self.publish_path(topopath, robot.ns)
                    self.log('%s: %s -> %s' % (robot.ns, source, dest.name), 'red')
                    
                    if i == robots_num - 1:
                        i = 0
                    else:
                        i += 1
        except rospy.ROSInterruptException:
            pass

    def choose_destination(self, source):
        """
        :param (str) source: afference of the robot
        :return: (Destination) destination
        """
        dest = None
        curr_idl = -1
        selected = []
        for d in self.destinations:
            if d.available and d.name != source and d.get_idleness() >= curr_idl:
                dest = d
                selected.append(d.name)
                curr_idl = d.get_idleness()

        # self.log('Destinations: %s, len: %s' % (', '.join(selected), len(selected)), 'cyan')
        # dest.available = False
        return dest
        # return random.choice(selected)
    
    def destinations_log(self):
        pub = rospy.Publisher(self.yaml['destinations_log'], DestinationDebug, queue_size=30)
        rate = rospy.Rate(10)
        
        try:
            while not rospy.is_shutdown():
                for d in self.destinations:
                    msg = DestinationDebug()
                    msg.available = d.available
                    msg.name = d.name
                    msg.idleness = d.get_idleness()
                    
                    pub.publish(msg)
                    rate.sleep()
        except rospy.ROSInterruptException:
            pass
        
    def publish_path(self, path, target_robot):
        pub = rospy.Publisher(target_robot + self.yaml['robot_topopath'], RobotTopopath, queue_size=10)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        pub.publish(path)
        
    def log(self, msg, color, attrs=None):
        if self.logging:
            print colored(msg, color=color, attrs=attrs)
        
    def on_shutdown(self):
        dest_logger = DestinationStatLogger(dest_list=self.destinations,
                                            robots_num=len(self.robots), environment=self.environment)
        confirm_save = dest_logger.show_confirm_gui()
        if confirm_save:
            dest_logger.write_statfile()
            rospy.loginfo('Destination idlenesses have been wrote to %s' % dest_logger.path)
        else:
            rospy.logwarn('Destination idlenesses have NOT been saved')
    
    
def parse_yaml(dir):
    f = open(dir, 'r')
    return yaml.load(f)

    
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--adjlist', type=str, required=True)
    parser.add_argument('--environment', type=str, required=True)
    parser.add_argument('--yaml', type=str, help='File where used topics_yaml are saved')
    parser.add_argument('--logging', type=bool, default=True, help="Show colored debugging msgs")
    args, unknown = parser.parse_known_args()
    
    return args


if __name__ == '__main__':
    rospy.init_node('topoplanner')
    
    args = parse_args()
    yaml = parse_yaml(args.yaml)
    
    planner = Planner(args.adjlist, environment=args.environment, yaml=yaml, logging=args.logging)
    rospy.on_shutdown(planner.on_shutdown)  # dumps idlenesses of destinations
    rospy.Timer(period=rospy.Duration(60*3), callback=planner.on_shutdown, oneshot=True)
    # planner.debug()
    planner.dispatch_goals()
