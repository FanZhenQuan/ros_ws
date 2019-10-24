#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3


class Robottino(object):

	def __init__(self):
		rospy.init_node('acceleratore', anonymous=True)
		rospy.on_shutdown(self.stop)
		self.rate= rospy.Rate(10)

	def run(self):
		self.publ= rospy.Publisher('cmd_vel', Twist, queue_size=10)

		while not rospy.is_shutdown():
			v1= Vector3(0.3, 0.0, 0.0)
			v2= v1
			tw= Twist(v1, v2)

			self.publ.publish(tw)

			self.rate.sleep()


	def stop(self):
		s1= Vector3(0.0, 0.0, 0.0)
		s2= s1
		_break= Twist(s1, s2)

		self.publ.publish(_break)
		rospy.loginfo('Stopping...')


if __name__ == '__main__':
	robot= Robottino()

	try:
		robot.run()
	except rospy.ROSInterruptException:
		pass