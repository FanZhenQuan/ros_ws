#!/usr/bin/env python

import rospy
import subprocess


def save_map():
    subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', 'mappa']).communicate()


if __name__ == '__main__':
    rospy.init_node('gmapping_and_mapsaver')
    rospy.on_shutdown(save_map)

    pop = subprocess.Popen(['roslaunch', 'learning_navigation', 'gmapping.launch'])

    try:
        pop.communicate()
    except rospy.ROSInterruptException:
        pop.terminate()
        save_map()
    except KeyboardInterrupt:
        pop.terminate()
        save_map()
