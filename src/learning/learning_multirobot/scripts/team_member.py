#!/usr/bin/env python

import rospy
import sys
from learning_multirobot.msg import RobotStatus, Mission
from std_msgs.msg import Header, String
from geometry_msgs.msg import Twist, Vector3


class Robot(object):
    id = None
    completed_missions = []

    def __init__(self):
        rospy.init_node('robot' + sys.argv[1])  # il nome viene rimpiazzato in ogni caso dal tag 'name' del file .launch
        self.id = int(sys.argv[1])

    def publish_ready_state(self, topic):
        header = Header()
        header.stamp = rospy.Time.now()

        ready_msg = RobotStatus()
        ready_msg.is_ready = True
        ready_msg.robot_id = self.id
        ready_msg.header = header

        pub = rospy.Publisher(topic, RobotStatus, queue_size=10)

        # finche' non c'e' un observer, aspetta
        while pub.get_num_connections() == 0:
            rospy.sleep(0.5)

        pub.publish(ready_msg)

    def wait_for_start(self, topic):
        rospy.Subscriber(topic, String, self.start_mission)

    def start_mission(self, msg):
        if msg.data == 'Ready':
            rospy.loginfo('Robot %s: started mission' % self.id)

    def handle_mission(self, mission_topic):
        rospy.Subscriber(mission_topic, Mission, self.act)

    def act(self, mission):
        self.completed_missions.append(mission)

        if mission.robot_id == self.id:
            if mission.name == 'walk':
                self.walk(mission.action)
            elif mission.name == 'rotate':
                self.rotate(mission.action)
            elif mission.name == 'report':
                self.report_mission()
        else:
            rospy.sleep(0.1)

    def walk(self, direction):
        linear = Vector3(direction*2, 0, 0)
        angular = Vector3(0, 0, 0)

        debug_msg = 'Robot %s: spostamento di valore %s' % (self.id, direction*2)
        print debug_msg  # rospy.loginfo(debug_msg)

        tw = Twist(linear, angular)

        self.publish_mission(tw)

    def rotate(self, steer_angle):
        PI = 3.141592

        radians_angle = steer_angle*PI/180
        linear = Vector3(0, 0, 0)
        angular = Vector3(0, 0, radians_angle)

        debug_msg = 'Robot %s: ruoto di %s gradi' % (self.id, steer_angle)
        print debug_msg  # rospy.loginfo(debug_msg)

        tw = Twist(linear, angular)

        self.publish_mission(tw)

    def publish_mission(self, mission):
        topic = '/turtle%s/turtle1/cmd_vel' % self.id
        pub = rospy.Publisher(topic, Twist, queue_size=5)

        rospy.sleep(0.3)  # tempo utile per distogliere lo sguardo dalla console verso il turtle
        pub.publish(mission)

    def report_mission(self):
        print 'Robot %s: %s missioni completate' % (self.id, len(self.completed_missions))


if __name__ == '__main__':
    robot = Robot()
    robot.publish_ready_state(topic='team_status')
    robot.wait_for_start(topic='start_mission')

    robot.handle_mission(mission_topic='missions')

    rospy.spin()