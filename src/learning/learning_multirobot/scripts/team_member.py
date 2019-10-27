#!/usr/bin/env python

import rospy
import sys
from learning_multirobot.msg import RobotStatus
from std_msgs.msg import Header, String


class Robot(object):
    id = None

    def __init__(self):
        rospy.init_node('robot' + sys.argv[1])
        self.id = int(sys.argv[1])

    def publish_ready_state(self, topic):
        header = Header()
        header.stamp = rospy.Time.now()

        ready_msg = RobotStatus()
        ready_msg.is_ready = True
        ready_msg.robot_id = self.id
        ready_msg.header = header

        pub = rospy.Publisher(topic, RobotStatus, queue_size=10)

        # finche' non c'e' un subscriber, aspetta
        while pub.get_num_connections() == 0:
            rospy.sleep(0.5)

        pub.publish(ready_msg)

    def wait_for_start(self, topic):
        rospy.Subscriber(topic, String, self.start_mission)

    def start_mission(self, msg):
        if msg.data == 'Ready':
            rospy.loginfo('Robot %s: started mission' % self.id)


if __name__ == '__main__':
    robot = Robot()
    robot.publish_ready_state(topic='team_status')
    robot.wait_for_start(topic='start_mission')

    rospy.spin()