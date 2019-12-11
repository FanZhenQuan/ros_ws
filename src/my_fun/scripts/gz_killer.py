#!/usr/bin/env python

import rospy
import subprocess
import signal


def kill(signal, frame):
    pop = subprocess.Popen(['pkill', '-SIGTERM', 'gzserver'])
    pop.communicate()
    
    rospy.logwarn('gzserver killed with SIGTERM')
    rospy.signal_shutdown('gzserver killed')


if __name__ == '__main__':
    rospy.init_node('gz_killer')
    
    signal.signal(signal.SIGINT, kill)
    
    rospy.spin()