#!/usr/bin/env python

import rospy
import yaml
import math
import argparse

from learning_toponav.msg import *
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class Toponavigator(object):
    READY = 'ready'
    BUSY = 'busy'
    
    def __init__(self, robot, yaml):
        self.robot_ns = robot
        self.yaml = yaml
        self.current_goal = None
        self.goal_reached = None
        self.state = self.READY

    def wait_topopaths(self):
        if self.state == self.READY:
            rospy.Subscriber('/'+self.robot_ns+self.yaml['robot_topopath'], RobotTopopath, self.on_topopath)
        else:
            rospy.loginfo('Toponavigator: robot %s busy, discarding requested topopath' % self.robot_ns)
            rospy.sleep(1)
        
    def on_topopath(self, path):
        move_base_goal_topic = '/'+self.robot_ns+'/move_base_simple/goal'
        # TODO: implementare consegna random di un punto nell'area di pertinenza (metrica)
        
        pub = rospy.Publisher(move_base_goal_topic, PoseStamped, queue_size=30)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        
        for ipoint in path.path:
            goal = ipoint.pose
            self.current_goal = goal
            self.goal_reached = False
            self.state = self.BUSY
            
            pub.publish(goal)
            
            self.has_reached_goal()
            
            while not self.goal_reached:
                rospy.sleep(1)
                
        rospy.loginfo('Toponavigator: end goal goal_reached')
        self.state = self.READY
            
    def has_reached_goal(self):
        rospy.Subscriber('/'+self.robot_ns+'/amcl_pose', PoseWithCovarianceStamped, self.on_amcl)

    def on_amcl(self, amcl_pose):
        amcl_posit = amcl_pose.pose.pose.position
        goal_posit = self.current_goal.pose.position
        
        # if amclpose is inside a circle built around
        # goal, return true, else false
        distance = math.hypot(
            amcl_posit.x - goal_posit.x,
            amcl_posit.y - goal_posit.y
        )
        
        tolerance = 0.4
        
        if distance <= tolerance:
            self.goal_reached = True
        else:
            self.goal_reached = False


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
    navigator.wait_topopaths()
    
    rospy.spin()