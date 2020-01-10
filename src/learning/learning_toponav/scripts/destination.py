#!/usr/bin/env python

import rospy
import sys
import os
path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(path)


class Destination(object):
    
    def __init__(self, name, available=True):
        self.name = name
        self.idleness = 0
        self.__latest_available = True
        self.__available = available
        self.__latest_usage = rospy.Time.now()
        
    @property
    def available(self):
        return self.__available
    
    @available.setter
    def available(self, value):
        self.__latest_available = self.__available
        self.__available = value
        if(
            self.__latest_available is False and
            self.__available is True
        ):
            self.reset()
        
    def get_idleness(self, travel_time=None):
        """
        :return: seconds elapsed since latest usage (type: float)
        """
        now = rospy.Time.now()
        if not travel_time:
            return (now - self.__latest_usage).to_sec()
        elif type(travel_time) == float or type(travel_time) == int:
            return (now - self.__latest_usage).to_sec() + travel_time
        else:
            msg = 'travel_time argument is of type %s, should be either float or int' % type(travel_time)
            raise Exception(msg)
        
    def reset(self):
        self.idleness = 0
        self.__latest_usage = rospy.Time.now()
        
    def __str__(self):
        return "Name: %s, status: %s, idleness: %s" %(self.name, self.available, self.idleness)
    
    def __eq__(self, other):
        return self.name == other.name