#!/usr/bin/env python

import rospy
import numpy as np
import random
import cv2
import argparse
from termcolor import colored
from geometry_msgs.msg import Point

# guide @ https://www.learnopencv.com/delaunay-triangulation-and-voronoi-diagram-using-opencv-c-python/


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, help='Image of the map to segmentate')
    args, unknown = parser.parse_known_args()
    
    return args


if __name__ == '__main__':
    rospy.init_node('map_segmentation')
    
    while not rospy.has_param('/interest_points'):
        rospy.sleep(0.1)

    points = []
    for i in rospy.get_param('/interest_points'):
        points.append(Point(
            i['pose']['position']['x'],
            i['pose']['position']['y'],
            0
        ))

    args = parse_args()
    
    img = cv2.imread(args.map)
    size = img.shape
    
    rect = (0, 0, size[1], size[0])
    subdiv = cv2.Subdiv2D(rect)
    for p in points:
        subdiv.insert((p.x+10, p.y+10))
    
    facets, centers = subdiv.getVoronoiFacetList([])
    
    for i in xrange(0, len(facets)):
        facets_arr = [f for f in facets[i]]
        
        ifacet = np.array(facets_arr, np.int)
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        
        cv2.fillConvexPoly(img, ifacet, color, cv2.LINE_AA, 0)
        ifacets = np.array([ifacet])
        cv2.polylines(img, ifacets, True, (0, 0, 0), 1, cv2.LINE_AA, 0)
        cv2.circle(img, (centers[i][0], centers[i][1]), 3, (0, 0, 0), -1, cv2.LINE_AA, 0)
        
    cv2.imshow('skidoodle', img)
    
    rospy.spin()