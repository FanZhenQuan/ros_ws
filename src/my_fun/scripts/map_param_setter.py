#!/usr/bin/env python

import rospy
import sys


if __name__ == '__main__':
    rospy.init_node('setter')
    
    bool = False
    
    try:
        map_dir = sys.argv[1]
        
        open(map_dir, 'r')
        
        bool = True
    except IndexError as e:
        str_err = str(e)
        
        if str_err == 'list index out of range':
            rospy.logerr("Manca il path della risorsa come argomento")
        else:
            rospy.logerr(str_err)
    except IOError as e:
        rospy.logerr('La risorsa data in argomento non esiste')

    rospy.set_param('map_is_loaded', bool)