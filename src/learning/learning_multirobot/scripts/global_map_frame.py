#!/usr/bin/env python
import rospy
import sys
import tf


if __name__ == '__main__':
    rospy.init_node('fixed_map_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    robot_map_frame = sys.argv[1]
    
    while not rospy.is_shutdown():
        # br.sendTransform((0.0, 0.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  robot_map_frame,
        #                  "world")
        #
        # br.sendTransform((0.0, 0.0, 0.0),
        #                  (0.0, 0.0, 0.0, 1.0),
        #                  rospy.Time.now(),
        #                  "world",
        #                  "map")

        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         robot_map_frame,
                         "map")
        rate.sleep()
