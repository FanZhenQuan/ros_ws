#!/usr/bin/env python

import rospy
import random
from learning_multirobot.msg import RobotStatus, Mission
from std_msgs.msg import String


class Observer(object):
    members_ready = team_size = 0
    assignement_rate = None

    def __init__(self, team_size=None, assignment_rate=None):
        if team_size is None or assignment_rate is None:
            raise TypeError('Observer takes two arguments: team_size and assignement_rate')
        else:
            rospy.init_node('observer')
            rospy.on_shutdown(self.abort_mission)
    
            self.team_size = team_size
            self.assignment_rate = rospy.Rate(assignment_rate)

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

    def assign_missions(self, missions_topic):
        dispatcher = rospy.Publisher(missions_topic, Mission, queue_size=self.team_size * 2)

        current_robot_assignment = 1
        mission_id = 0

        while dispatcher.get_num_connections() < self.team_size:
            rospy.sleep(0.5)

        rospy.loginfo('-----------------------------')
        rospy.loginfo('Observer: assigning missions')

        while not rospy.is_shutdown():
            mission = Mission()
            task = Task()  # genera un Task random, vedi classe definita sotto

            mission.robot_id = current_robot_assignment
            mission.mission_id = mission_id
            mission.name = task.name
            mission.action = task.action

            dispatcher.publish(mission)

            # aumenta di 1 il contatore del robot, cosi' da dare
            # un incarico al robot successivo
            if current_robot_assignment == self.team_size:
                current_robot_assignment = 1
            else:
                current_robot_assignment += 1

            mission_id += 1
            self.assignment_rate.sleep()

    def abort_mission(self):
        rospy.loginfo('-----------------------------')
        rospy.loginfo('Observer: aborting mission...')


class Task(object):
    __names = ['walk', 'rotate', 'report']
    __directions = [1, -1]  # avanti | indietro
    __steer_angle = range(-180, 181)

    def __init__(self):
        self.name = random.choice(self.__names)

        if self.name == 'walk':
            self.action = random.choice(self.__directions)
        elif self.name == 'rotate':
            self.action = random.choice(self.__steer_angle)
        elif self.name == 'report':
            self.action = float('-inf')


if __name__ == '__main__':
    try:
        observer = Observer(team_size=4, assignment_rate=4)
        observer.wait_for_members(listen_topic='team_status')

        observer.assign_missions(missions_topic='missions')

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
