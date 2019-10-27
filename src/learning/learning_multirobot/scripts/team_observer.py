#!/usr/bin/env python

import rospy
from learning_multirobot.msg import RobotStatus
from std_msgs.msg import String


TEAM_SIZE = 4


class Observer(object):
    members_ready = 0

    def __init__(self):
        rospy.init_node('observer')

    def wait_for_members(self, listen_topic):
        rospy.Subscriber(listen_topic, RobotStatus, self.update_team_status)

    def update_team_status(self, msg):
        self.members_ready += 1

        if self.members_ready == TEAM_SIZE:
            rospy.loginfo('Observer: all members are ready, starting mission!')
            self.start_mission('start_mission')
        else:
            rospy.loginfo('Observer: %s members are ready, waiting...' % self.members_ready)

    @staticmethod
    def start_mission(mission_topic):
        pub = rospy.Publisher(mission_topic, String, queue_size=5)

        while pub.get_num_connections() < TEAM_SIZE:
            rospy.sleep(0.1)

        pub.publish('Ready')


if __name__ == '__main__':
    observer = Observer()
    observer.wait_for_members(listen_topic='team_status')

    rospy.spin()