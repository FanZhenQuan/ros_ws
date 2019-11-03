#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node('not_found_thing')

    while not rospy.is_shutdown():
        rospy.loginfo('sas')