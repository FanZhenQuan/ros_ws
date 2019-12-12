#!/usr/bin/env python

import rospy
import signal


def signal_handler(signal, frame):
    rospy.loginfo('allah akbar')
    rospy.signal_shutdown('Hai premuto CTRL+C')


if __name__ == '__main__':
    rospy.init_node('sigint_test')

    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.spin()