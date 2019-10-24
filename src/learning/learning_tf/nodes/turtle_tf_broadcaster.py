#!/usr/bin/env python

import roslib
import rospy
import tf
import tf.transformations
import turtlesim.msg
roslib.load_manifest('learning_tf')


def handle_turtle_pose(msg, turtlename):
    # broadcasts this turtle's translation and rotation,
    # and publishes it as a transform from frame "world"
    # to frame "turtlename"

    br = tf.TransformBroadcaster()
    br.sendTransform(
        (msg.x, msg.y, 0),
        tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        rospy.Time.now(),
        turtlename,
        "world"
    )


if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')

    # from the parameter server, retrieves the
    # turtle name, e.g. "turtle1" or "turtle2"
    turtlename = rospy.get_param('~turtle')

    rospy.loginfo('Broadcaster turtle name: ' + turtlename)

    # subscribes to the __turtlename__/pose topic and
    # runs handle_turtle_pose() on every turtlesim.msg.Pose
    # msg received
    rospy.Subscriber(
        '/%s/pose' % turtlename,
        turtlesim.msg.Pose,
        handle_turtle_pose,
        turtlename
    )
    
    rospy.spin()