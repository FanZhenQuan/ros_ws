#!/usr/bin/env python

import rospy
import sys
from learning_multirobot.msg import RobotStatus
from std_msgs.msg import String


def main():
    try:
        robotname = sys.argv[1]
        rospy.init_node('robot' + robotname)
    except:
        rospy.logerr('Errore: id del robot mancante')

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.Publisher('bo', String, queue_size=5).publish('Args [1]: '+ sys.argv[1] )

        rate.sleep()


if __name__ == '__main__':
    main()