#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from pynput.keyboard import Key
from pynput import keyboard





def on_press(key):
    try:
        if key == Key.ctrl:
            msg = rospy.wait_for_message('scan', LaserScan)
            study_distance(msg)
        elif key == Key.alt_l:
            return False
    except AttributeError:
        pass


def study_distance(data):
    array = data.ranges
    print array[:20]


if __name__ == '__main__':
    infos = '''
*************************************
*    Cattura una scansione: CTRL    *
*    Termina il programma: ALT-L    *
*************************************
    '''

    rospy.init_node('distance_checker', anonymous=True)
    rospy.loginfo(infos)

    with keyboard.Listener(
            on_press=on_press,
    ) as listener:
        listener.join()
