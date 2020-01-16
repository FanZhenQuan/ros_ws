#!/usr/bin/env python

import rospy
import yaml
import math
import numpy as np
import argparse

from robot import Robot
from termcolor import colored
from threading import Thread
from prettytable import PrettyTable
from learning_toponav.msg import *
from std_msgs.msg import Header, String
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion


class Toponavigator(object):
    READY = 'ready'
    BUSY = 'busy'
    GOAL_DISTANCE_BIAS = 0.4
    STATE_RATE = 1
    
    def __init__(self, robot, yaml=None):
        self.yaml = yaml
        
        if robot.startswith('/'):
            name = robot
        else:
            name = '/'+robot
        self.robot = Robot(ns=name, state=self.READY)
        self.goal_reached = None
        
        self.state_publisher = self.movebase_goal_pub = None
        
    def start_threads(self):
        # --- subscribers
        rospy.Subscriber(self.robot.ns+self.yaml['robot_topopath'], RobotTopopath, self.on_topopath)
        rospy.Subscriber(self.robot.ns+self.yaml['pose_topic'], PoseWithCovarianceStamped, self.on_amcl)
        
        # -- publishers
        self.movebase_goal_pub = rospy.Publisher(self.robot.ns+self.yaml['goal_topic'], PoseStamped, queue_size=10)
        self.state_publisher = rospy.Publisher(self.robot.ns+self.yaml['robot_state'],
                                               RobotState, queue_size=self.STATE_RATE*2)
        t = Thread(target=self.publish_state)
        t.start()
        # for some reason, doesn't need to be .join()-ed
        # upon receiving rospy.is_shutdown() flag ...
    
    def publish_state(self):
        try:
            rate = rospy.Rate(self.STATE_RATE)
            while not rospy.is_shutdown():
                while(
                    not self.robot.state or
                    not self.robot.afference
                ):
                    rospy.sleep(0.1)
                
                msg = RobotState()
                msg.robot_name = self.robot.ns
                msg.state = self.robot.state
                msg.afference = self.robot.afference
                msg.distance = self.robot.distance
        
                if self.robot.latest_goal is None:
                    msg.latest_goal = 'None'
                else:
                    msg.latest_goal = self.robot.latest_goal
        
                if self.robot.current_goal is None:
                    msg.current_goal = 'None'
                else:
                    msg.current_goal = self.robot.current_goal.name

                self.state_publisher.publish(msg)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
    
    def on_topopath(self, path):
        self.robot.state = self.BUSY
        while self.movebase_goal_pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        
        for ipoint in path.path:
            self.robot.current_goal = ipoint
            self.goal_reached = False
            
            self.movebase_goal_pub.publish(ipoint.pose)
            while not self.goal_reached:
                rospy.sleep(1)
            
            self.robot.latest_goal = self.robot.current_goal.name
        
        print colored('%s: end goal reached' % self.robot.ns, 'green')
        self.robot.state = self.READY
    
    def on_amcl(self, amcl_pose):
        # -- distance-to-goal calc
        amcl_posit = amcl_pose.pose.pose.position
        if self.robot.current_goal:
            goal_posit = self.robot.current_goal.pose.pose.position
        
            # if amcl_pose is inside a circle built around
            # goal, goal is reached, else isn't
            distance = math.hypot(
                amcl_posit.x - goal_posit.x,
                amcl_posit.y - goal_posit.y
            )
            if distance <= self.GOAL_DISTANCE_BIAS:
                self.goal_reached = True
            else:
                self.goal_reached = False
    
        # -- afference calc
        # TODO: choose one or the other
        eucl = self.eucl_afference(amcl_posit=amcl_posit)
        
        pose_stamped = PoseStamped(amcl_pose.header, amcl_pose.pose.pose)
        movebase = self.movebase_afference(amcl_pose=pose_stamped)
        
        self.debug_aff(amcl_posit, eucl, movebase)
        
    def eucl_afference(self, amcl_posit):
        while not rospy.has_param(self.yaml['interest_points']):
            rospy.sleep(0.1)
        int_points = rospy.get_param(self.yaml['interest_points'])
        
        afference = None
        afference_dist = float('inf')
        ip_pos_debug = None  # TODO: debug
        for ipoint in int_points:
            ipoint_posit = {
                'x': ipoint['pose']['position']['x'],
                'y': ipoint['pose']['position']['y'],
                'z': ipoint['pose']['position']['z'],
            }
            rbt_posit = {
                'x': amcl_posit.x,
                'y': amcl_posit.y,
                'z': amcl_posit.z
            }
            e_dist = math.hypot(
                ipoint_posit['x'] - rbt_posit['x'],
                ipoint_posit['y'] - rbt_posit['y']
            )
        
            # assign minimum distance
            if e_dist < afference_dist:
                afference_dist = e_dist
                afference = ipoint['name']
                ip_pos_debug = Point(ipoint_posit['x'], ipoint_posit['y'], ipoint_posit['z'])
                
        # self.robot.afference = afference
        # self.robot.distance = afference_dist
        
        return {
            'mode': 'euclidean',
            'afference': afference,
            'ip_posit': ip_pos_debug,
            'dist': afference_dist
        }

    def movebase_afference(self, amcl_pose):
        while not rospy.has_param(self.yaml['interest_points']):
            rospy.sleep(0.1)
        ipoints = rospy.get_param(self.yaml['interest_points'])
    
        afference_dist = float('inf')
        afference = None
        ip_pos_debug = None  # TODO: debug
        for ip in ipoints:
            ip_pose = self.get_ip_posestamp(ip)
            
            rospy.wait_for_service(self.robot.ns+self.yaml['make_plan'])
            try:
                make_plan = rospy.ServiceProxy(self.robot.ns+self.yaml['make_plan'], GetPlan)
                response = make_plan(amcl_pose, ip_pose, self.GOAL_DISTANCE_BIAS)
            
                path_length = self.get_plan_len(response.plan)
                if path_length <= afference_dist:
                    afference_dist = path_length
                    afference = ip['name']
                    ip_pos_debug = ip_pose.pose.position
            except rospy.ServiceException as e:
                print "Make_plan call failed: %s" % e
    
        self.robot.distance = afference_dist
        self.robot.afference = afference

        return {
            'mode': 'movebase',
            'afference': afference,
            'ip_posit': ip_pos_debug,
            'dist': afference_dist
        }
        
    def debug_aff(self, amcl_posit, eucl, movebase):
        # pt = PrettyTable(['Robot', 'Mode', 'Afference', 'Distance'])
        # pt.add_row([self.robot.ns, eucl['mode'], eucl['afference'], eucl['dist']])
        # pt.add_row([self.robot.ns, movebase['mode'], movebase['afference'], movebase['dist']])
        #
        # print pt
        if self.robot.ns == '/robot_1':
            msg = AfferenceDebug()
            msg.robot_posit = amcl_posit
            msg.eucl_afference = eucl['ip_posit']
            msg.mvbs_afference = movebase['ip_posit']
            
            pub = rospy.Publisher('/afference_debug', AfferenceDebug, queue_size=10)
            while pub.get_num_connections() < 1:
                rospy.sleep(0.1)
            pub.publish(msg)
        
    @staticmethod
    def get_plan_len(plan):
        path_length = 0
        for i in range(len(plan.poses) - 1):
            position_a_x = plan.poses[i].pose.position.x
            position_b_x = plan.poses[i + 1].pose.position.x
            position_a_y = plan.poses[i].pose.position.y
            position_b_y = plan.poses[i + 1].pose.position.y
        
            path_length += np.sqrt(
                np.power((position_b_x - position_a_x), 2) + np.power((position_b_y - position_a_y), 2))
            
        return path_length
    
    @staticmethod
    def get_ip_posestamp(ip):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
    
        ip_x = ip['pose']['position']['x']
        ip_y = ip['pose']['position']['y']
        ip_z = ip['pose']['position']['z']
    
        ip_pose = PoseStamped()
        ip_pose.header = header
        ip_pose.pose = Pose(Point(ip_x, ip_y, ip_z), Quaternion(0, 0, 0, 1))
        
        return ip_pose


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, help='Robot namespace')
    parser.add_argument('--yaml', type=str, help='File where used topics_yaml are saved')
    args, unknown = parser.parse_known_args()
    
    return args


def parse_yaml(dir):
    f = open(dir, 'r')
    return yaml.load(f)


if __name__ == '__main__':
    rospy.init_node('toponavigator')
    
    args = parse_args()
    yaml = parse_yaml(args.yaml)
    
    navigator = Toponavigator(args.robot, yaml=yaml)
    navigator.start_threads()
    
    rospy.spin()