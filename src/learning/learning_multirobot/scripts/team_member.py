#!/usr/bin/env python

import rospy
import sys
from learning_multirobot.msg import RobotStatus
from std_msgs.msg import Header


class Robot(object):
    id = None

    def __init__(self):
        self.id = sys.argv[1]
        rospy.init_node('robot' + self.id)

    def publish_ready_state(self, topic):
        header = Header()
        header.stamp = rospy.Time.now()

        msg = RobotStatus()
        msg.is_ready = True
        msg.robot_id = self.id
        msg.header = header

        pub = rospy.Publisher(topic, RobotStatus, queue_size=10)
        pub.publish(msg)


if __name__ == '__main__':
    robot = Robot()
    robot.publish_ready_state(topic='team_status')

    rospy.spin()