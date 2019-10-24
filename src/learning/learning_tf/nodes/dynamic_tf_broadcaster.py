#!/usr/bin/env python
import roslib
import rospy
import tf
import math
roslib.load_manifest('learning_tf')


if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() * math.pi

        # instead of defining a fixed offset from turtle1,
        # we are using a sin and cos function based on the
        # current time to cause the definition of the frame's offset
        br.sendTransform(
            (2.0 * math.sin(t), 2.0 * math.cos(t), 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "carrot1",
            "turtle1"
        )

        rate.sleep()