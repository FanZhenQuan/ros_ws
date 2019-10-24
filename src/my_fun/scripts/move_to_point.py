#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry as odom
from geometry_msgs.msg import Point, Twist, Vector3


THRESHOLD = 0.8
GOAL_POS = Point(4, 4, 0)
RATE = None


def check_if_goal(new_pos, goal_pos):
    if (
        goal_pos.x + THRESHOLD >= new_pos.x >= goal_pos.x - THRESHOLD and
        new_pos.y + THRESHOLD >= new_pos.y >= goal_pos.y - THRESHOLD
    ):
        return True
    else:
        return False


def drive_to_goal(new_pos, goal_pos):
    global RATE

    driver = rospy.Publisher('cmd_vel', Twist)

    pass
    # YOU NEED TO STUDY


def on_pose_change(msg):
    global RATE

    while not rospy.is_shutdown():
        new_pos = msg.pose.pose.position
        arrived = check_if_goal(new_pos, GOAL_POS)

        if not arrived:
            drive_to_goal(new_pos, GOAL_POS)
        else:
            rospy.loginfo('You have arrived at destination')
            break

        RATE.sleep()


def on_shutdown():
    rospy.loginfo('Exiting...')


if __name__ == '__main__':
    global RATE

    rospy.init_node('driver')
    rospy.on_shutdown(on_shutdown)
    RATE = rospy.Rate(10)
    init_pose = rospy.wait_for_message('odom', odom).pose.pose.position

    rospy.Subscriber('odom', odom, on_pose_change)