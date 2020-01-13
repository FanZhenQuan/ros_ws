#!/usr/bin/env python

import rospy
import yaml
import math
import numpy as np
import argparse
from robot import Robot

from termcolor import colored
from threading import Thread
from learning_toponav.msg import *
from nav_msgs.srv import GetPlan
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped


class Toponavigator(object):
    READY = 'ready'
    BUSY = 'busy'
    GOAL_DISTANCE_BIAS = 0.4
    STATE_RATE = 1
    
    def __init__(self, robot, yaml):
        self.yaml = yaml if yaml else None
        
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
        rospy.Subscriber(self.robot.ns+'/amcl_pose', PoseWithCovarianceStamped, self.on_amcl)
        
        # -- publishers
        self.movebase_goal_pub = rospy.Publisher(self.robot.ns+self.yaml['goal_topic'], PoseStamped, queue_size=10)
        self.state_publisher = rospy.Publisher(self.robot.ns+self.yaml['robot_state'], RobotState, queue_size=self.STATE_RATE*2)
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
                    # if self.robot.current_goal is None:
                    #     msg.latest_goal = 'None'
                    # else:
                    #     msg.latest_goal = self.robot.current_goal.name
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
        # TODO: implementare consegna random di un punto nell'area di pertinenza (metrica)
        if self.robot.state == self.BUSY:
            rospy.logwarn('Goal ignored: robot %s is busy' % self.robot.ns)
        else:
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
            
            # rospy.loginfo('Toponavigator: %s end goal reached' % self.robot.ns)
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
        # self.eucl_afference(amcl_posit=amcl_posit)
        
        if self.robot.current_goal:
            source = amcl_pose.pose
            dest = self.robot.current_goal.pose
            self.movebase_afference(amcl_pose=source, goal_pose=dest)
        
    def eucl_afference(self, amcl_posit):
        while not rospy.has_param(self.yaml['interest_points']):
            rospy.sleep(0.1)
        int_points = rospy.get_param(self.yaml['interest_points'])
        closest_ipoint = None
        min_dist = float('inf')
    
        for ipoint in int_points:
            ipoint_pose = {
                'x': ipoint['pose']['position']['x'],
                'y': ipoint['pose']['position']['y'],
                'z': ipoint['pose']['position']['z'],
            }
            rbt_pose = {
                'x': amcl_posit.x,
                'y': amcl_posit.y,
                'z': amcl_posit.z
            }
            e_dist = math.hypot(
                ipoint_pose['x'] - rbt_pose['x'],
                ipoint_pose['y'] - rbt_pose['y']
            )
        
            # assign minimum distance
            if e_dist < min_dist:
                min_dist = e_dist
                closest_ipoint = ipoint['name']
                
        self.robot.afference = closest_ipoint
        self.robot.distance = min_dist
        
    def movebase_afference(self, amcl_pose, goal_pose):
        while not rospy.has_param(self.yaml['interest_points']):
            rospy.sleep(0.1)
        ipoints = rospy.get_param(self.yaml['interest_points'])

        min_dist = float('inf')
        afference = None
        for ip in ipoints:
            header = Header()
            header.stamp = rospy.Time.now()
            
            ip_x = ip['pose']['position']['x']
            ip_y = ip['pose']['position']['y']
            ip_z = ip['pose']['position']['z']
            
            ip_pose = PoseStamped()
            ip_pose.header = header
            ip_pose.pose = ((ip_x, ip_y, ip_z), (0, 0, 0, 1))
            
            rospy.wait_for_service(self.robot.ns+'/move_base/NavfnROS/make_plan')
            try:
                make_plan = rospy.ServiceProxy(self.robot.ns+'/move_base/NavfnROS/make_plan', GetPlan)
                plan = make_plan(amcl_pose, ip_pose)
                
                path_length = 0
                for i in range(len(plan.poses) - 1):
                    position_a_x = plan.poses[i].pose.position.x
                    position_b_x = plan.poses[i + 1].pose.position.x
                    position_a_y = plan.poses[i].pose.position.y
                    position_b_y = plan.poses[i + 1].pose.position.y
    
                    path_length += np.sqrt(
                        np.power((position_b_x - position_a_x), 2) + np.power((position_b_y - position_a_y), 2))
                    
                    if path_length <= min_dist:
                        min_dist = path_length
                        afference = ip['name']
                        
                self.robot.distance = path_length
                self.robot.afference = afference
            except rospy.ServiceException, e:
                print "Make_plan call failed: %s" % e


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