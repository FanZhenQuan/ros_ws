#!/usr/bin/env python

import rospy
import yaml
import math
import argparse

from threading import Thread
from learning_toponav.msg import RobotTopopath, RobotState
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class Toponavigator(object):
    READY = 'ready'
    BUSY = 'busy'
    GOAL_DISTANCE_BIAS = 0.4
    
    def __init__(self, robot, yaml):
        self.robot_ns = robot
        self.yaml = yaml
        self.current_goal = self.latest_goal = None
        self.goal_reached = None
        self.state = self.READY
        
        self.state_publisher = rospy.Publisher('/'+robot+yaml['robot_state'], RobotState, queue_size=10)
        self.publisher_thread()
    
    def publisher_thread(self):
        t = Thread(target=self.publish_state)
        t.start()
        # for some reason, doesn't need to be .join()-ed
        # upon receiving rospy.is_shutdown() flag ...
    
    def publish_state(self):
        try:
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                msg = RobotState()
                msg.robot_name = self.robot_ns
                msg.state = self.state
        
                if self.latest_goal is None:
                    if self.current_goal is None:
                        msg.latest_goal = 'None'
                    else:
                        msg.latest_goal = self.current_goal.name
                    # msg.latest_goal = 'None'
                else:
                    msg.latest_goal = self.latest_goal
        
                if self.current_goal is None:
                    msg.current_goal = 'None'
                else:
                    msg.current_goal = self.current_goal.name
        
                self.state_publisher.publish(msg)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
    
    def wait_topopaths(self):
        rospy.Subscriber('/'+self.robot_ns+self.yaml['robot_topopath'], RobotTopopath, self.on_topopath)
    
    def on_topopath(self, path):
        # TODO: implementare consegna random di un punto nell'area di pertinenza (metrica)
        move_base_goal_topic = '/'+self.robot_ns+'/move_base_simple/goal'
        
        pub = rospy.Publisher(move_base_goal_topic, PoseStamped, queue_size=30)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        
        for ipoint in path.path:
            self.current_goal = ipoint
            self.goal_reached = False
            self.state = self.BUSY
            
            pub.publish(ipoint.pose)
            
            self.has_reached_goal()
            
            while not self.goal_reached:
                rospy.sleep(1)
        
        rospy.loginfo('Toponavigator: %s end goal reached' % self.robot_ns)
        self.state = self.READY
        self.latest_goal = self.current_goal.name
        
    def has_reached_goal(self):
        rospy.Subscriber('/'+self.robot_ns+'/amcl_pose', PoseWithCovarianceStamped, self.on_amcl)
    
    def on_amcl(self, amcl_pose):
        amcl_posit = amcl_pose.pose.pose.position
        goal_posit = self.current_goal.pose.pose.position
        
        # if amclpose is inside a circle built around
        # goal, return true, else false
        distance = math.hypot(
            amcl_posit.x - goal_posit.x,
            amcl_posit.y - goal_posit.y
        )
        
        if distance <= self.GOAL_DISTANCE_BIAS:
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