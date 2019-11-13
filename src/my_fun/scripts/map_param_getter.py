#!/usr/bin/env python

import rospy


if __name__ == '__main__':
    rospy.init_node('getter')
    
    map_is_loaded = rospy.get_param('map_is_loaded')
    
    if map_is_loaded:
        rospy.loginfo('Mappa trovata')
    else:
        rospy.logerr('Mappa non trovata')