#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class Echoer(object):
    def __init__(self, name):
        rospy.init_node(name)

        self.publ_2 = rospy.Publisher('ns2/turtle1/cmd_vel', Twist, queue_size=10)
        self.publ_3 = rospy.Publisher('ns3/turtle1/cmd_vel', Twist, queue_size=10)

    def listen(self, topic):
        rospy.Subscriber(topic, Twist, self.echo)

    def echo(self, msg):
        self.publ_2.publish(msg)
        self.publ_3.publish(msg)


if __name__ == '__main__':
    echoer = Echoer(name='echoer')

    echoer.listen('/ns1/turtle1/cmd_vel')

    rospy.spin()