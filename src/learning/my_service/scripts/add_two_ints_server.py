#!/usr/bin/env python

from my_service.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
	print "Returning [%s + %s = %s]"%(req.a, req.b, req.a+req.b)
	return AddTwoIntsResponse(req.a + req.b)
	# return req.a + req.b funziona comunque

def add_two_ints_server():
	rospy.init_node('add_two_ints_server')
	s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
	print "Ready to add two ints."
	rospy.spin()

if __name__ == "__main__":
	add_two_ints_server()