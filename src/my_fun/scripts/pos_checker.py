#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose


def log(msg):
    rospy.loginfo(msg)


def main():
    rospy.init_node('pos_checker')

    rospy.Subscriber('robot_pose', Pose, log)

    rospy.spin()


if __name__ == '__main__':
    main()