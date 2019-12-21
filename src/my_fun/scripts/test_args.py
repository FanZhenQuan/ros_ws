#!/usr/bin/env python

import rospy
import sys
from pprint import pprint

if __name__ == "__main__":
    rospy.init_node('args')
    
    pprint(sys.argv)