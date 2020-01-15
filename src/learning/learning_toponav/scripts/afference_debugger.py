#!/usr/bin/env python

import rospy

from learning_toponav.msg import AfferenceDebug
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Vector3


def build_marker(start, goal, color):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    
    ns = ''
    type = 0  # arrow
    action = 0
    # pose = Pose(robot_posit, Quaternion(0, 0, 0, 1))
    points = [start, goal]
    lifetime = rospy.Duration(1)
    frame_locked = False
    scale = Vector3(0.1, 0.1, 0.1)
    
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.type = type
    marker.action = action
    marker.points = points
    marker.color = color
    marker.lifetime = lifetime
    marker.frame_locked = frame_locked
    marker.scale = scale
    
    return marker


def on_afference(msg):
    robot_posit = msg.robot_posit
    eucl_posit = msg.eucl_afference
    mvbs_posit = msg.mvbs_afference
    
    arrow_to_eucl = build_marker(robot_posit, eucl_posit, color=ColorRGBA(1, 0, 0, 1))
    arrow_to_mvbs = build_marker(robot_posit, mvbs_posit, color=ColorRGBA(0, 1, 0, 1))
    
    publish(arrow_to_eucl, '/eucl_aff')
    publish(arrow_to_mvbs, '/mvbs_aff')
    

def publish(msg, topic):
    pub = rospy.Publisher(topic, Marker, queue_size=10)
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('afference_debugger')
    
    rospy.Subscriber('/afference_debug', AfferenceDebug, on_afference)
    
    rospy.spin()