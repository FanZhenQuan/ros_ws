#!/usr/bin/env python

import rospy
import argparse
import scipy.spatial.distance as scipy

from toponodes_publisher import INTEREST_POINTS
from geometry_msgs.msg import PoseWithCovarianceStamped
from learning_toponav.msg import RobotAfference
from visualization_msgs.msg import Marker, MarkerArray


AFFERENCES_TOPIC = '/afferences'


class Topolocaliser(object):
    def __init__(self, robot):
        self.robot_ns = robot
        self.int_points = rospy.get_param(INTEREST_POINTS)

    @staticmethod
    def eucl_dist(ipoint_pose, robot_pose):
        return scipy.euclidean(ipoint_pose, robot_pose)

    def read_amcl_pose(self):
        rospy.Subscriber('/' + self.robot_ns + '/amcl_pose', PoseWithCovarianceStamped, self.on_amcl_pose)
        
    def on_amcl_pose(self, robot_pose):
        # checks the closest interest point
        min_dist = float('inf')
        closest_ipoint = None
        
        for ipoint in self.int_points:
            # the two different style signatures have to be kept!
            ipoint_pose = [
                ipoint['pose']['position']['x'],
                ipoint['pose']['position']['y'],
                ipoint['pose']['position']['z']
            ]
            rbt_pose = [
                robot_pose.pose.pose.position.x,
                robot_pose.pose.pose.position.y,
                robot_pose.pose.pose.position.z
            ]
            
            e_dist = self.eucl_dist(ipoint_pose, rbt_pose)
            
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
        
        pub = rospy.Publisher(AFFERENCES_TOPIC, RobotAfference, queue_size=15)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.3)
        pub.publish(afference)
    

def shutdown():
    rospy.loginfo('Topolocaliser shutting down')
    
    
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, help='Robot namespace')
    args, unknown = parser.parse_known_args()
    
    return args
    

if __name__ == '__main__':
    rospy.init_node('topolocaliser')
    rospy.on_shutdown(shutdown)
    
    args = parse_args()
    localiser = Topolocaliser(robot=args.robot)

    while not rospy.is_shutdown():
        localiser.read_amcl_pose()