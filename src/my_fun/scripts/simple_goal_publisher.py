#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('dummy')
    
    goal = PoseStamped()
    
    goal.header.seq = 0
    goal.header.frame_id = 'map'
    
    goal.pose.position.x = -0.7
    goal.pose.position.y = 3.3
    goal.pose.position.z = 0
    
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0.76
    goal.pose.orientation.w = 0.64
    
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    
    rospy.loginfo('Publishing goal in 3 seconds')
    rospy.sleep(3)
    
    pub.publish(goal)
    
    rospy.loginfo('Messaggio pubblicato')
    
    rospy.spin()