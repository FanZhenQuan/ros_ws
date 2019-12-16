#!/usr/bin/env python

import rospy
import math
import argparse

from topoplanner import ROBOT_TOPOPATH_TOPIC
from learning_toponav.msg import *
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class Toponavigator(object):
    def __init__(self, robot):
        self.robot_ns = robot
        self.current_goal = None
        self.goal_reached = None

        rospy.Subscriber('/'+self.robot_ns+ROBOT_TOPOPATH_TOPIC, RobotTopopath, self.on_topopath)
        
    def on_topopath(self, path):
        move_base_goal_topic = '/'+self.robot_ns+'/move_base_simple/goal'
        rate = rospy.Rate(15)
        
        pub = rospy.Publisher(move_base_goal_topic, PoseStamped, queue_size=30)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        
        for ipoint in path.path:
            goal = ipoint.pose
            self.current_goal = goal
            self.goal_reached = False
            
            pub.publish(goal)
            
            self.has_reached_goal()
            
            while not self.goal_reached:
                rospy.sleep(1)
                
        rospy.loginfo('Toponavigator: end goal goal_reached')
            
    def has_reached_goal(self):
        # if amclpose is inside a circle built around
        # goal, return true, else false
        rospy.Subscriber('/'+self.robot_ns+'/amcl_pose', PoseWithCovarianceStamped, self.on_amcl)
        
        rospy.loginfo('Iscritto a amcl')

    def on_amcl(self, amcl_pose):
        amcl_posit = amcl_pose.pose.pose.position
        goal_posit = self.current_goal.pose.position
        
        distance = math.hypot(
            amcl_posit.x - goal_posit.x,
            amcl_posit.y - goal_posit.y
        )
        
        tolerance = 0.3
        
        # rospy.loginfo('Toponavigator: distance: %s tolerance: %s' %(distance, tolerance))
        
        if distance <= tolerance:
            self.goal_reached = True
        else:
            self.goal_reached = False


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, help='Robot namespace')
    args, unknown = parser.parse_known_args()

    return args


if __name__ == '__main__':
    rospy.init_node('toponavigator')
    
    args = parse_args()
    
    navigator = Toponavigator(args.robot)
    
    rospy.spin()