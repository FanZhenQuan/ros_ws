#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3


RATE = 10


class Robottino(object):
    speed = 0.5

    def __init__(self):
        rospy.init_node('robot_driver')
        rospy.on_shutdown(self.stop)
        self.volante_publ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.laser_subs = rospy.Subscriber('scan', LaserScan, self.scan)
        self.rate = rospy.Rate(RATE)

    def drive(self):
        while not rospy.is_shutdown():
            marcia = Vector3(self.speed, 0, 0)
            void = Vector3(0, 0, 0)

            move_dir = Twist(
                marcia,
                void
            )

            self.volante_publ.publish(move_dir)
            self.rate.sleep()

    def stop(self):
        void = Vector3(0, 0, 0)

        move_dir = Twist(
            void,
            void
        )

        self.volante_publ.publish(move_dir)
        rospy.loginfo('Exiting...')

    def scan(self, data):
        front_array = list(data.ranges[:10])
        back_array = list(data.ranges[175:185])

        change_drive_dir = False

        if self.speed > 0:
            for f in front_array:
                rospy.loginfo('Measuring distance from front')
                if f < 0.8:
                    self.speed *= -1
                    change_drive_dir = True
                    rospy.loginfo('Cambio senso di marcia: ' + str(self.speed))
                    break

        if not change_drive_dir and self.speed < 0:
            rospy.loginfo('Measuring distance from behind')
            for b in back_array:
                if b < 0.8:
                    self.speed *= -1
                    rospy.loginfo('Cambio senso di marcia: '+str(self.speed))
                    break


if __name__ == '__main__':
    r = Robottino()

    try:
        r.drive()
    except rospy.ROSInterruptException:
        pass
