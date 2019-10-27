#!/usr/bin/env python

import rospy
from learning_multirobot.msg import RobotStatus
from std_msgs.msg import String


class Observer(object):
    members_ready = team_size = 0

    def __init__(self, team_size):
        self.team_size = team_size
        rospy.init_node('observer')

    def wait_for_members(self, listen_topic):
        rospy.Subscriber(listen_topic, RobotStatus, self.update_team_status)

    def update_team_status(self, msg):
        self.members_ready += 1

        if self.members_ready == self.team_size:
            rospy.loginfo('Observer: all members are ready, starting mission!')
            self.start_mission('start_mission')
        else:
            rospy.loginfo('Observer: %s members are ready, waiting...' % self.members_ready)

    def start_mission(self, mission_topic):
        pub = rospy.Publisher(mission_topic, String, queue_size=5)

        while pub.get_num_connections() < self.team_size:
            rospy.sleep(0.1)

        pub.publish('Ready')


if __name__ == '__main__':
    observer = Observer(team_size=4)
    observer.wait_for_members(listen_topic='team_status')

    rospy.spin()