#!/usr/bin/env python

import rospy
import argparse
import yaml
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from learning_toponav.msg import RobotAfference


class Topolocaliser(object):
    def __init__(self, robot, yaml):
        self.robot_ns = robot
        self.yaml = yaml
        self.afference_pub_rate = rospy.Rate(15)
        
        while not rospy.has_param(self.yaml['interest_points']):
            rospy.sleep(0.1)
        self.int_points = rospy.get_param(self.yaml['interest_points'])
        
    def sleep(self):
        self.afference_pub_rate.sleep()

    def read_amcl_pose(self):
        s = rospy.Subscriber('/' + self.robot_ns + '/amcl_pose', PoseWithCovarianceStamped, self.on_amcl_pose)
        
        while s.get_num_connections() < 1:
            rospy.sleep(0.1)
        
    def on_amcl_pose(self, robot_pose):
        # checks the closest interest point
        min_dist = float('inf')
        closest_ipoint = None
        
        for ipoint in self.int_points:
            ipoint_pose = {
                'x': ipoint['pose']['position']['x'],
                'y': ipoint['pose']['position']['y'],
                'z': ipoint['pose']['position']['z'],
            }
            rbt_pose = {
                'x': robot_pose.pose.pose.position.x,
                'y': robot_pose.pose.pose.position.y,
                'z': robot_pose.pose.pose.position.z
            }
            e_dist = math.hypot(
                ipoint_pose['x'] - rbt_pose['x'],
                ipoint_pose['y'] - rbt_pose['y']
            )
            
            # assign minimum distance
            if e_dist < min_dist:
                min_dist = e_dist
                closest_ipoint = ipoint['name']
        
        self.publish_robot_afference(ipoint=closest_ipoint, distance=min_dist)
    
    def publish_robot_afference(self, ipoint, distance):
        afference = RobotAfference()
        afference.ipoint_name = ipoint
        afference.distance = distance
        afference.robot_name = self.robot_ns
        
        pub = rospy.Publisher('/' + self.robot_ns + self.yaml['afference'], RobotAfference, queue_size=30)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.3)
        pub.publish(afference)
    
    
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
    rospy.init_node('topolocaliser')
    
    args = parse_args()
    yaml = parse_yaml(args.yaml)
    localiser = Topolocaliser(robot=args.robot, yaml=yaml)

    try:
        while not rospy.is_shutdown():
            localiser.read_amcl_pose()
            localiser.sleep()
    except rospy.ROSInterruptException:
        pass
