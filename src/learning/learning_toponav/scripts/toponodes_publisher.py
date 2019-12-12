#!/usr/bin/env python

import rospy
import argparse

from std_msgs.msg import Header, String, ColorRGBA
from geometry_msgs.msg import Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray

# from topological_node import TopologicalNode
from topological_map import TopologicalMap


INTEREST_POINTS = 'interest_points'


def node_to_marker(node, marker_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    
    ns = ''
    id = marker_id
    type = 2  # sphere
    action = 0

    npose = node.get_position()
    norient = node.get_orientation()
    pose = Pose(npose, norient)
    
    scale = Vector3(0.25, 0.25, 0.25)

    color = ColorRGBA()
    color.r = 51 / 255
    color.g = 255 / 255
    color.b = 51 / 255
    color.a = 1.0

    frame_locked = False
    
    # ---------COSTRUZIONE MARKER--------
    marker = Marker()
    
    marker.header = header
    marker.ns = ns
    marker.id = id
    marker.type = type
    marker.action = action
    marker.pose = pose
    marker.scale = scale
    marker.color = color
    marker.frame_locked = frame_locked
    
    return marker


def build_marker_array(topomap):
    markers = []
    id = 0
    for node in topomap.nodes:
        m = node_to_marker(node=node, marker_id=id)
        markers.append(m)
        id += 1
        
        rospy.loginfo('marker %s/%s created' % (str(id), str(len(topomap.nodes))))
        
    return MarkerArray(markers)


def publish_marker_array(topic):
    pub = rospy.Publisher(topic, MarkerArray, queue_size=5)
    # wait for a subscriber (Rviz)
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(marker_array)
        
        
def shutdown():
    rospy.loginfo('toponodes_publisher: Shutting down')


if __name__ == '__main__':
    global INTEREST_POINTS
    
    rospy.init_node('toponodes_publisher')
    rospy.on_shutdown(shutdown)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--topomap', type=str, help='Topological map')
    args, unknown = parser.parse_known_args()

    topomap = TopologicalMap(filename=args.topomap)
    rospy.set_param(INTEREST_POINTS, topomap.nodes)
    
    marker_array = build_marker_array(topomap)

    publish_marker_array(topic=INTEREST_POINTS)

    rospy.spin()