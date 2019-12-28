#!/usr/bin/env python

import argparse
import random
import yaml
import rospy
import networkx as nx

from termcolor import colored
from robot import Robot
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from learning_toponav.msg import *
from std_msgs.msg import Header


class Planner(object):
    def __init__(self, adjlist, yaml):
        self.graph = nx.read_adjlist(adjlist, delimiter=', ', nodetype=str)
        
        colors = rospy.get_param('/colors/')
        self.robots = [Robot(ns='/'+ns, color=color) for color, ns in colors.items()]
        self.nodes = rospy.get_param(yaml['interest_points'])
        self.yaml = yaml
        self.available_robots = self.busy_robots = []
        self.destinations = []
        for n in list(self.graph.nodes()):
            d = {
                'name': n,
                'available': True
            }
            self.destinations.append(d)

        # ---------------
        self.start_threads()
        self.update_robot_state()
        
    def start_threads(self):
        dests_logger = Thread(target=self.destinations_log)
        dests_logger.start()
        
        # robot_state_updater = Thread(target=self.update_robot_state)
        # robot_state_updater.start()
        
    def update_robot_state(self):
        for r in self.robots:
            rospy.Subscriber(r.ns + yaml['robot_state'], RobotState, self.on_robot_state)
        
    def on_robot_state(self, msg):
        upd = {
            'robot_name': msg.robot_name,
            'state': msg.state,
            'current_goal': msg.current_goal,
            'latest_goal': msg.latest_goal,
            'afference': msg.afference,
            'distance': msg.distance
        }
        
        for index, r in enumerate(self.robots):
            if r.ns == upd['robot_name']:
                self.robots[index].state = upd['state']
                self.robots[index].current_goal = upd['current_goal']
                self.robots[index].latest_goal = upd['latest_goal']
                self.robots[index].afference = upd['afference']
                self.robots[index].distance = upd['distance']
                
            if r.state == 'ready' and r not in self.available_robots:
                self.available_robots.append(r)
                print colored('%s is available' % r.ns, 'green', attrs=['bold'])
        
        if msg.state == 'ready':
            self.update_available_dests(
                add=[msg.latest_goal, msg.current_goal]
                # add=msg.latest_goal
            )
        else:
            self.update_available_dests(
                # add=[msg.latest_goal, msg.current_goal],
                add=msg.latest_goal,
                remove=msg.current_goal
            )

        # rospy.sleep(2)
        
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
        for n in self.nodes:
            if n['name'] == name:
                return n
            
    def _get_robot_by_ns(self, ns):
        for r in self.robots:
            if r.ns == ns:
                return r

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

    def update_available_dests(self, add=None, remove=None):
        for d in self.destinations:
            if add:
                if add[0] or add[1]:
                    if add[0] and d['name'] == add[0]:
                        d['available'] = True
                        # rospy.logwarn(add[0])
                    elif add[1] and d['name'] == add[1]:
                        d['available'] = True
                        # rospy.logwarn(add[1])
            # if add and d['name'] == add:
            #     d['available'] = True
            
            if (
                remove and
                d['name'] == remove and
                remove != add
            ):
                d['available'] = False
                # rospy.logerr(remove)
    
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
        rate = rospy.Rate(.5)
        i = 0

        print colored('Topoplanner starting soon...', 'cyan', attrs=['bold'])
        try:
            while not rospy.is_shutdown():
                if not self.available_robots:
                    rospy.sleep(1)
                else:
                    robot = self.available_robots.pop(0)
                    source = robot.afference
                    dest = self.choose_destination(robot, source)
                    path = self.find_path(source=source, dest=dest)
                    
                    topopath = self.build_topopath(path)
                    self.publish_path(topopath, robot.ns)
                    rospy.loginfo('Path from %s to %s sent to %s' % (source, dest, robot.ns))
                    
                    if i == robots_num - 1:
                        i = 0
                    else:
                        i += 1
                        
                    # rate.sleep()
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
