#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Twist
import turtlesim.srv
roslib.load_manifest('learning_tf')

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # We want to apply the transform from the '/turtle1' frame
        # to the '/turtle2' frame at a specific time. Providing
        # rospy.Time(0) will just get us the latest available transform.
        try:
            # (trans,rot) = listener.lookupTransform(
            #                 '/turtle2',
            #                 '/turtle1',
            #                 rospy.Time(0)
            #             )

            # uncomment if you want the turtle to chase the invisible carrot
            # located 2 meters above turtle1
            (trans, rot) = listener.lookupTransform("/turtle2", "/carrot1", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()